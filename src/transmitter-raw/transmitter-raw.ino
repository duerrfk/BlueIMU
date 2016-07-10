/**
 * This file is part of BlueIMU.
 * 
 * Copyright 2016 Frank Duerr
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include <TimerOne.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// The pin where the LED output of the Bluetooth module is connected.
// This pin is used to detect the connection status.
// This pin supports level interrupts to listen for connection status changes.
#define PIN_LED 2

// Sampling period in microseconds.
// Note: Depending on the sampling interval, you should also set
// the digital low-pass filter (see DLPF definition).
#define SAMPLING_PERIOD 5000

// The baud rate of the serial connection to the Bluetooth module. 
// The MCU is running at 7.3728 MHz. Thus, 230400 baud = 7.3728/32 allows for 
// perfect timing for serial communication. 
#define BAUD_RATE 230400

// The sensitivity and value range of the accelerometer as defined by the
// following table:
//
// AFS_SEL | Full Scale Range | LSB Sensitivity
// --------+------------------+----------------
// 0       | +/- 2g           | 16384 LSB/g
// 1       | +/- 4g           | 8192 LSB/g
// 2       | +/- 8g           | 4096 LSB/g
// 3       | +/- 16g          | 2048 LSB/g
#define RANGE_ACCEL 0

// The sensitivity and value range of the gyroscope as defined by the
// following table:
//
// FS_SEL | Full Scale Range   | LSB Sensitivity
// -------+--------------------+----------------
// 0      | +/- 250 degrees/s  | 131 LSB/deg/s
// 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
// 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
// 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
#define RANGE_GYRO 0

// Configuration of digital low pass filter of IMU according to the
// following table:
//
//          |   ACCELEROMETER    |           GYROSCOPE
// DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
// ---------+-----------+--------+-----------+--------+-------------
// 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
// 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
// 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
// 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
// 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
// 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
// 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
// 7        |   -- Reserved --   |   -- Reserved --   | Reserved
//
// For instance, if you choose value 6, signals with frequency > 5 Hz
// will be cut-off.
#define DLPF 1

// Start of frame pattern.
const uint8_t start_of_frame[] = {0x5A, 0xA5};

volatile enum State {disconnected, connected} state = disconnected;  

volatile bool is_sample_due = false;

// The GY-521 board has a pull-down resistor connected to AD0,
// so the I2C address is 0x68 (default). Change to 0x69 if a pull-up 
// resistor is used instead.
//MPU6050 imu(0x69);
MPU6050 imu;

/**
 * Calculation of 16 bit checksum. 
 * 
 * This implementation uses the same algorithm as IP to calculate
 * IP header checksums. 
 * 
 * Performing the same calculation over the data *and*
 * checksum should yield 0 in case of no error.
 * 
 * The following code is adapted from Gary R. Wright, W. Richard Stevens: 
 * TCP/IP Illustrated, Volume 2 (there called a "naive implementation"; 
 * you can also find an optimized implementation there).
 */
uint16_t checksum(const uint8_t *data, size_t len)
{
     // Note that we do not need to consider byte order. From RFC 1071: 
     // "The sum of 16-bit integers can be computed in either byte order."

     uint32_t sum = 0;
     uint16_t *words = (uint16_t *) data;

     while (len > 1) {
         sum += *(words)++;
         if (sum&0x80000000)
             sum = (sum&0xFFFF) + (sum>>16);
         len -= 2;
     }

     if (len)
         sum += *((uint8_t *) words);
     
     while (sum>>16)
         sum = (sum & 0xFFFF) + (sum >> 16);

     return ~sum;
}

/**
 * Marshal data and send frame over serial connection to Bluetooth module.
 * 
 * All 16 bit values are sent in Big Endian (network) byte order.
 */
void send_frame(int16_t accelX, int16_t accelY, int16_t accelZ,
                int16_t gyroX, int16_t gyroY, int16_t gyroZ, uint32_t timestamp) 
{
    uint8_t frame[20];

    // Add start of frame
    frame[0] = start_of_frame[0];
    frame[1] = start_of_frame[1];

    // Add values in Big Endian byte order
    frame[2] = (accelX>>8);
    frame[3] = (accelX&0xFF);

    frame[4] = (accelY>>8);
    frame[5] = (accelY&0xFF);

    frame[6] = (accelZ>>8);
    frame[7] = (accelZ&0xFF);

    frame[8] = (gyroX>>8);
    frame[9] = (gyroX&0xFF);

    frame[10] = (gyroY>>8);
    frame[11] = (gyroY&0xFF);
    
    frame[12] = (gyroZ>>8);
    frame[13] = (gyroZ&0xFF);

    frame[14] = (timestamp>>24);
    frame[15] = ((timestamp>>16)&0xFF);
    frame[16] = ((timestamp>>8)&0xFF);
    frame[17] = (timestamp&0xFF);
    
    // Add checksum
    uint16_t csum = checksum(frame, 18);
    // The IP checksum algorithm works for any byte order. 
    // However, we need to sent out the 16 bit checksum in the 
    // byte order of the platform. Atmega uses Little Endian
    // byte order, so we need to send the low byte first.
    frame[18] = (csum&0xFF);
    frame[19] = (csum>>8);

    // Send frame
    Serial.write(frame, 20);
}

/**
 * Called in case of fatal error preventing continuation.
 * 
 * This function will never return and signal a fatal error over
 * Bluetooth by sending empty frames (start of frame sequence only) 
 * in an endlessly.
 */
void die()
{
    uint8_t frame[2];
    frame[0] = start_of_frame[0];
    frame[1] = start_of_frame[1];
    
    while (1) {
        // Send empty frames to signal exception
        Serial.write(frame, 2);
        delay(1000); 
    }
}

/**
 * Initial setup.
 */
void setup() 
{
    // Give components (IMU module, Bluetooth module) some time to start.
    delay(2000);
    
    Serial.begin(BAUD_RATE, SERIAL_8N1); 

    // Initialize sampling timer. The timer is only active
    // when a connection is made via Bluetooth. So we only
    // initialize the timer interval here and start the timer later.
    Timer1.initialize(SAMPLING_PERIOD);

    // With this pin, the Bluetooth module signals the connection status.
    pinMode(PIN_LED, INPUT);

    // Initialize IMU

    // Init I2C
    Wire.begin();
    
    // Initialize IMU device.
    imu.initialize();
    imu.setFullScaleGyroRange(RANGE_GYRO);
    imu.setFullScaleAccelRange(RANGE_ACCEL);
    
    // Set digital low-pass filter (DLPF)
    //          |   ACCELEROMETER    |           GYROSCOPE
    // DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
    // ---------+-----------+--------+-----------+--------+-------------
    // 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
    // 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
    // 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
    // 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
    // 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
    // 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
    // 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
    // 7        |   -- Reserved --   |   -- Reserved --   | Reserved
    imu.setDLPFMode(DLPF);

    unsigned int try_count = 0;
    while (1) {
        // Verify connection
        if (imu.testConnection()) {
            // All fine
            break;
        } else {
            try_count++;
        }
        if (try_count > 3)
            die();
        delay(1000);
    }
}

/**
 * Interrupt routing for checking communication status.
 * 
 * Disconnections are signalled by low-level on PIN_LED.
 */
void led_isr()
{
    // We are now disconneted and can deregister ISR until
    // another connection is made.
    detachInterrupt(digitalPinToInterrupt(PIN_LED));
    
    // Detach timer interrupt to stop sampling.
    Timer1.detachInterrupt();
    
    state = disconnected;
}

/**
 * Interrupt routine of sampling timer.
 */
void sampling_timer_isr()
{
    is_sample_due = true;
}

/**
 * Take a sample (accelerometer & gyro) and send
 * data to Bluetooth module for transmission.
 */
void take_sample_and_send()
{
    int16_t axi, ayi, azi;
    int16_t rxi, ryi, rzi;
    imu.getMotion6(&axi, &ayi, &azi, &rxi, &ryi, &rzi); 
    uint32_t t = millis();
    
    send_frame(axi, ayi, azi, rxi, ryi, rzi, t);  
}

void loop() 
{
    unsigned int high_count;
    switch (state) {
    case disconnected :
        // Wait for Bluetooth connection. 
        // While disconnected, the LED blinks with a period of 750 ms. 
        // While connected, the LED is constantly on.
        // To detect a connection, we sample the LED with a frequency of 10 Hz.
        // If the LED pin remains high during an interval of 1 s, we assume 
        // that the Bluetooth module is connected. 
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
        // level interrupt fires early.
        Timer1.attachInterrupt(sampling_timer_isr);
        // If the LED pin goes low, the device is disconnected. 
        // To detect the disconnection, we use an interrupt 
        // (the LED is connected to a digital pin supporting interrupts). 
        attachInterrupt(digitalPinToInterrupt(PIN_LED), led_isr, LOW);
        break;
    case connected:
        if (is_sample_due) {
            take_sample_and_send();
            is_sample_due = false;
        }
        break;
    }
}
