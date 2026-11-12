/*
  Diagnostics.cpp - Periodic health monitor on the serial console
  (c) Abrantix AG
*/

#include "Diagnostics.h"
#include <WiFi.h>
#include "esp_heap_caps.h"
#include "CameraStream.h"

#define DIAG_PERIOD_MS          5000
#define DIAG_TASKDUMP_EVERY     3       // full task dump every n periods (and on any stall)
#define DIAG_MAX_TASKS          32

extern volatile uint32_t AsyncWebServer_DiagAccepted;
extern volatile uint32_t AsyncWebServer_DiagRequests;
extern volatile uint32_t AsyncWebServer_DiagDisconnects;

static TaskStatus_t taskStatus[DIAG_MAX_TASKS];
static uint32_t prevRunTime[DIAG_MAX_TASKS];
static UBaseType_t prevTaskNumber[DIAG_MAX_TASKS];
static uint32_t prevTotalRunTime;

void Diagnostics::Start()
{
    // Above loop/camstream/async_tcp so it still reports when those hang or spin
    xTaskCreatePinnedToCore(TaskMain, "diag", 4096, nullptr, configMAX_PRIORITIES - 5, nullptr, tskNO_AFFINITY);
}

void Diagnostics::DumpTasks()
{
    uint32_t totalRunTime;
    UBaseType_t n = uxTaskGetSystemState(taskStatus, DIAG_MAX_TASKS, &totalRunTime);
    uint32_t totalDelta = totalRunTime - prevTotalRunTime;
    prevTotalRunTime = totalRunTime;

    static const char stateChar[] = "XRBSD";
    Serial.println("[diag] task           st prio stackfree cpu%");
    for(UBaseType_t i = 0; i < n; i++)
    {
        TaskStatus_t& t = taskStatus[i];
        uint32_t prev = 0;
        for(int j = 0; j < DIAG_MAX_TASKS; j++)
        {
            if(prevTaskNumber[j] == t.xTaskNumber)
            {
                prev = prevRunTime[j];
                break;
            }
        }
        uint32_t cpu = totalDelta ? (uint32_t)((uint64_t)(t.ulRunTimeCounter - prev) * 100 / totalDelta) : 0;
        Serial.printf("[diag] %-16s %c %4u %9u %3u\n", t.pcTaskName,
                      t.eCurrentState < 5 ? stateChar[t.eCurrentState] : '?',
                      (unsigned)t.uxCurrentPriority, (unsigned)t.usStackHighWaterMark, (unsigned)cpu);
    }
    for(int j = 0; j < DIAG_MAX_TASKS; j++)
    {
        prevTaskNumber[j] = j < (int)n ? taskStatus[j].xTaskNumber : 0;
        prevRunTime[j] = j < (int)n ? taskStatus[j].ulRunTimeCounter : 0;
    }
}

void Diagnostics::TaskMain(void*)
{
    uint32_t lastLoop = 0, lastAccepted = 0, lastRequests = 0, lastDisconnects = 0, lastFrames = 0, lastBytes = 0;
    uint32_t period = 0;
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(DIAG_PERIOD_MS));
        period++;

        uint32_t loops = loopCount - lastLoop;                                      lastLoop += loops;
        uint32_t accepted = AsyncWebServer_DiagAccepted - lastAccepted;             lastAccepted += accepted;
        uint32_t requests = AsyncWebServer_DiagRequests - lastRequests;             lastRequests += requests;
        uint32_t disconnects = AsyncWebServer_DiagDisconnects - lastDisconnects;    lastDisconnects += disconnects;
        uint32_t frames = CameraStreamer::DiagFrames - lastFrames;                  lastFrames += frames;
        uint32_t bytes = CameraStreamer::DiagBytes - lastBytes;                     lastBytes += bytes;
        bool streaming = CameraStreamer::IsActive();

        Serial.printf("[diag] up=%lus loop=%lu/s heap=%u(min %u, blk %u) psram=%u sta=%u "
                      "tcp acc=%lu req=%lu disc=%lu stream=%s fps=%lu kB/s=%lu stalls=%lu\n",
                      millis() / 1000, loops * 1000 / DIAG_PERIOD_MS,
                      (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                      (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                      (unsigned)ESP.getFreePsram(), (unsigned)WiFi.softAPgetStationNum(),
                      accepted, requests, disconnects, streaming ? "on" : "off",
                      frames * 1000 / DIAG_PERIOD_MS, bytes / DIAG_PERIOD_MS, (uint32_t)CameraStreamer::DiagStalls);

        bool stalled = loops == 0 || (streaming && frames == 0) || (accepted > 0 && requests == 0);
        if(stalled)
        {
            Serial.println("[diag] *** STALL suspected ***");
        }
        if(stalled || period % DIAG_TASKDUMP_EVERY == 0)
        {
            DumpTasks();
        }
    }
}
