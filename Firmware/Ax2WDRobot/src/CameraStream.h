#ifndef _CAMERA_STREAM_H
#define _CAMERA_STREAM_H

// MJPEG stream pushed by a dedicated task, so that the (blocking) camera frame grab
// and the frame transmission never run inside the async_tcp task.
// The HTTP response only sends the headers and then hands the connection to the streamer task.
// AsyncClient::write() is thread safe in AsyncTCP; the client pointer itself is protected by a
// mutex and cleared from the disconnect callback before the client object is deleted.

#include "Arduino.h"
#include "esp_camera.h"
#include "ESPAsyncWebServer.h"

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %ld.%06ld\r\n\r\n";

class CameraStreamer
{
public:
    // Make `client` the (single) active stream receiver; a previous one is closed.
    static void Attach(AsyncClient* client)
    {
        EnsureStarted();
        xSemaphoreTake(mutex, portMAX_DELAY);
        if(activeClient && activeClient != client)
        {
            activeClient->close();
        }
        activeClient = client;
        xSemaphoreGive(mutex);
        xTaskNotifyGive(task);
    }

    static void Detach(AsyncClient* client)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        if(activeClient == client)
        {
            activeClient = nullptr;
        }
        xSemaphoreGive(mutex);
    }

private:
    static inline SemaphoreHandle_t mutex = nullptr;
    static inline TaskHandle_t task = nullptr;
    static inline AsyncClient* activeClient = nullptr;

    static void EnsureStarted()
    {
        if(task)
        {
            return;
        }
        mutex = xSemaphoreCreateMutex();
        xTaskCreatePinnedToCore(TaskMain, "camstream", 4096, nullptr, 2, &task, 1);
    }

    static bool HasClient()
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        bool has = activeClient != nullptr;
        xSemaphoreGive(mutex);
        return has;
    }

    // Writes everything, waiting for TCP send buffer space outside of the mutex.
    // Returns false if the client is gone or made no progress for too long (then it is closed).
    static bool WriteAll(const uint8_t* data, size_t len)
    {
        size_t sent = 0;
        uint32_t lastProgress = millis();
        while(sent < len)
        {
            size_t written = 0;
            xSemaphoreTake(mutex, portMAX_DELAY);
            AsyncClient* c = activeClient;
            if(!c)
            {
                xSemaphoreGive(mutex);
                return false;
            }
            size_t remaining = len - sent;
            size_t room = c->space();
            // Avoid tiny segments: wait until a full segment (or the rest) fits
            if(room >= (remaining < 1436 ? remaining : 1436))
            {
                written = c->write((const char*)data + sent, remaining < room ? remaining : room);
            }
            if(!written && millis() - lastProgress > 1000)
            {
                c->close();
                activeClient = nullptr;
                xSemaphoreGive(mutex);
                return false;
            }
            xSemaphoreGive(mutex);

            if(written)
            {
                sent += written;
                lastProgress = millis();
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
        return true;
    }

    static void TaskMain(void*)
    {
        char header[160];
        for(;;)
        {
            if(!HasClient())
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                continue;
            }

            camera_fb_t* fb = esp_camera_fb_get();
            if(!fb)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if(fb->format == PIXFORMAT_JPEG)
            {
                int hlen = snprintf(header, sizeof(header), "%s", STREAM_BOUNDARY);
                hlen += snprintf(header + hlen, sizeof(header) - hlen, STREAM_PART,
                                 (unsigned)fb->len, (long)fb->timestamp.tv_sec, (long)fb->timestamp.tv_usec);
                if(WriteAll((const uint8_t*)header, hlen))
                {
                    WriteAll(fb->buf, fb->len);
                }
            }
            esp_camera_fb_return(fb);
        }
    }
};

class AsyncJpegStreamResponse: public AsyncWebServerResponse {
public:
    AsyncJpegStreamResponse()
    {
        _code = 200;
        _contentLength = 0;
        _contentType = STREAM_CONTENT_TYPE;
        _sendContentLength = false;
        _chunked = false;
        addHeader("Connection", "close");
        addHeader("Cache-Control", "no-store");
    }

    bool _sourceValid() const override { return true; }

    void _respond(AsyncWebServerRequest *request) override
    {
        AsyncClient* client = request->client();
        String head = _assembleHead(request->version());
        // Stays in RESPONSE_CONTENT (never "finished") until the client disconnects
        _state = RESPONSE_CONTENT;
        client->write(head.c_str(), head.length());
        request->onDisconnect([client]() { CameraStreamer::Detach(client); });
        CameraStreamer::Attach(client);
    }

    size_t _ack(AsyncWebServerRequest *request, size_t len, uint32_t time) override { return 0; }
};

#endif
