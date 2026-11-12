#ifndef _CAMERA_STREAM_H
#define _CAMERA_STREAM_H

// MJPEG stream pushed by a dedicated task, so that the (blocking) camera frame grab
// and the frame transmission never run inside the async_tcp task.
// The HTTP response only sends the headers and then hands the connection to the streamer task.
// AsyncClient::write() is thread safe in AsyncTCP; the client pointer itself is protected by a
// mutex and cleared from the disconnect callback before the client object is deleted.
// Clients are reset with abort() rather than close(): close() runs the disconnect callback
// (-> Detach, request deletion) synchronously in the calling task, and a graceful close of a
// stalled connection keeps it (with its unsent data) alive in lwIP and in the browser for minutes.
// abort() only queues an error event; the async_tcp task then runs Detach and deletes the client,
// which cannot happen before we release the mutex. The mutex is recursive as a safety net.

#include "Arduino.h"
#include "esp_camera.h"
#include "ESPAsyncWebServer.h"

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %ld.%06ld\r\n\r\n";

// Phones regularly pause ACKs for 1-2s (WiFi power save, retransmissions); only give up after this
static const uint32_t STALL_TIMEOUT_MS = 5000;

class CameraStreamer
{
public:
    // Make `client` the (single) active stream receiver; a previous one is closed.
    static void Attach(AsyncClient* client)
    {
        EnsureStarted();
        Serial.printf("[stream] %lu attach %s\n", millis(), client->remoteIP().toString().c_str());
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        if(activeClient && activeClient != client)
        {
            Serial.println("[stream] aborting previous client");
            activeClient->abort();
        }
        activeClient = client;
        xSemaphoreGiveRecursive(mutex);
        xTaskNotifyGive(task);
    }

    static void Detach(AsyncClient* client)
    {
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        if(activeClient == client)
        {
            Serial.printf("[stream] %lu detach\n", millis());
            activeClient = nullptr;
        }
        xSemaphoreGiveRecursive(mutex);
    }

    // Diagnostic counters
    static inline volatile uint32_t DiagFrames = 0;
    static inline volatile uint32_t DiagBytes = 0;
    static inline volatile uint32_t DiagStalls = 0;

    static bool IsActive()
    {
        return activeClient != nullptr;
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
        mutex = xSemaphoreCreateRecursiveMutex();
        xTaskCreatePinnedToCore(TaskMain, "camstream", 4096, nullptr, 2, &task, 1);
    }

    static bool HasClient()
    {
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        bool has = activeClient != nullptr;
        xSemaphoreGiveRecursive(mutex);
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
            xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
            AsyncClient* c = activeClient;
            if(!c)
            {
                xSemaphoreGiveRecursive(mutex);
                return false;
            }
            size_t remaining = len - sent;
            size_t room = c->space();
            // Avoid tiny segments: wait until a full segment (or the rest) fits
            if(room >= (remaining < 1436 ? remaining : 1436))
            {
                written = c->write((const char*)data + sent, remaining < room ? remaining : room);
            }
            if(!written && millis() - lastProgress > STALL_TIMEOUT_MS)
            {
                DiagStalls++;
                Serial.printf("[stream] %lu no TX progress for %lums (space=%u), aborting\n", millis(), (unsigned long)STALL_TIMEOUT_MS, (unsigned)room);
                c->abort();
                activeClient = nullptr;
                xSemaphoreGiveRecursive(mutex);
                return false;
            }
            xSemaphoreGiveRecursive(mutex);

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
                    if(WriteAll(fb->buf, fb->len))
                    {
                        DiagFrames++;
                        DiagBytes += fb->len;
                    }
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
        // The web server's short rx/ack timeouts are meant for normal requests; a stream receives
        // nothing and may pause, so stalls are handled (with abort) by WriteAll instead
        client->setRxTimeout(0);
        client->setAckTimeout(STALL_TIMEOUT_MS + 2000);
        client->write(head.c_str(), head.length());
        request->onDisconnect([client]() { CameraStreamer::Detach(client); });
        CameraStreamer::Attach(client);
    }

    size_t _ack(AsyncWebServerRequest *request, size_t len, uint32_t time) override { return 0; }
};

#endif
