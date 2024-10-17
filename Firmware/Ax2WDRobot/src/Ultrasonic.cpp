/* Ultrasonic.cpp */
#include "Ultrasonic.h"
#include "Trace.h"
#include "AxDisplay.h"

Ultrasonic UltrasonicInstance;

unsigned long ultrasonic_rising_micros;
unsigned long ultrasonic_falling_micros;
bool ultrasonic_measurement_ready;

// We're using an ISR to measure the pulse width coming from the RCW-0001/HC-SR04... (and similar) ultrasonic echo sensors
void IRAM_ATTR ultrasonic_echo_isr() {
    if(digitalRead(ULTRASONIC_ECHO_PIN))
    {
        ultrasonic_rising_micros = micros();
    }
    else
    {
        ultrasonic_falling_micros = micros();
        ultrasonic_measurement_ready = true;
    }
}

Ultrasonic::Ultrasonic()
{
    lastReadMillis = 0;
    distance = 888;
}

void Ultrasonic::Init()
{
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN), ultrasonic_echo_isr, CHANGE);
}

float Ultrasonic::GetMeasurement()
{
 return distance;
}

void Ultrasonic::Process()
{
    if(ultrasonic_measurement_ready)
    {
        ultrasonic_measurement_ready = false;
        // Sound travels in air at about 343 m/s at 20degrees C
        //  that is 343 / 1000000 m per microsecond
        //  that is ( 343 / 1000000  * 100) cm per microsecond
        // Note that we need to "cast" to a float here to calculate correctly
        // divide by 2 since that was a return trip
        distance = (ultrasonic_falling_micros - ultrasonic_rising_micros) * ((float) 343 / 1000000 * 100) / 2; 
        
        //TRACEPRINTF("Dst:%f cm\n", roundf(distance));
        AxDisplayInstance.UpdateDistance(roundf(distance));
    }
    if(millis() - lastReadMillis >= ULTRASONIC_INTERVAL_MILLIS)
    {
        lastReadMillis = millis();
        // Send a trigger pulse
        // Spec says 10us - but we're generous
        digitalWrite(ULTRASONIC_TRIG_PIN,HIGH);
        delayMicroseconds(20);
        digitalWrite(ULTRASONIC_TRIG_PIN,LOW);        
    }
}
