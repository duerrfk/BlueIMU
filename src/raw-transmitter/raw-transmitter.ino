#include "TimerOne.h"

// The pin where the LED output of the Bluetooth module is connected.
// This pin is used to detect the connection status.
// This pin supports level interrupts to listen for connection status changes.
#define PIN_LED 2

// Sampling period in microseconds
#define SAMPLING_PERIOD 1000000

// The baud rate of the serial connection to the Bluetooth module. 
// The MCU is running at 7.3728 MHz. Thus, 230400 baud = 7.3728/32 allows for 
// perfect timing for the serial communication. 
#define BAUD_RATE 230400

volatile enum State {disconnected, connected} state = disconnected;  

volatile bool is_sample_due = false;

void setup() 
{
    // Enable internal pull-up resistor for RX on MCU side.
    // The Bluetooth module does not seem to have push/pull drivers for TX. 
    // Without pull-up resistor, the line stays at around 1.5 V in marking
    // condition, which is too low.
    //pinMode(0, INPUT_PULLUP);
    Serial.begin(BAUD_RATE, SERIAL_8N1); 
                
    Timer1.initialize(SAMPLING_PERIOD);

    pinMode(PIN_LED, INPUT);
}

void led_isr()
{
    detachInterrupt(digitalPinToInterrupt(PIN_LED));
    
    // Detach timer interrupt to stop sampling
    Timer1.detachInterrupt();
    
    state = disconnected;
}

void sampling_timer_isr()
{
    is_sample_due = true;
}

void take_sample()
{
    Serial.println("Hello");
}

void loop() 
{
    unsigned int high_count;
    switch (state) {
    case disconnected :
        // Wait for connection. 
        // While disconnected, the LED blinks with a period of 750 ms. 
        // While connected, the LED is constantly on.
        // To detect a connection, we sample the LED with a frequency of 10 Hz.
        // If the LED remains high during an interval of 1 s, we assume that the device is connected. 
        high_count = 0;
        while (high_count < 10) {
            if (digitalRead(PIN_LED) == LOW)
                high_count = 0;
            else
                high_count++; 
            delay(100);
        }
        state = connected;
        // Start timer for sampling.
        // It's important to first attach the timer interrupt and then the level
        // interrupt, otherwise the timer interrupt might no get detached when the
        // level interrupt fires.
        Timer1.attachInterrupt(sampling_timer_isr);
        // If the LED pin goes low, the device is disconnected. 
        // To detect the disconnection, we use an interrupt 
        // since the LED is connected to a digital pin supporting interrupts. 
        attachInterrupt(digitalPinToInterrupt(PIN_LED), led_isr, LOW);
        break;
    case connected:
        if (is_sample_due) {
            take_sample();
            is_sample_due = false;
        }
        break;
    }
}
