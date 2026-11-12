/*
  Diagnostics.h - Periodic health monitor on the serial console
  (c) Abrantix AG
*/

#ifndef __DIAGNOSTICS_H
#define __DIAGNOSTICS_H

#include <Arduino.h>

class Diagnostics
{
public:
    // Starts the monitor task (prints a status line every few seconds and a task dump on stalls)
    static void Start();
    // Call once per main loop iteration
    static void LoopTick() { loopCount++; }

private:
    static inline volatile uint32_t loopCount = 0;
    static void TaskMain(void*);
    static void DumpTasks();
};

#endif
