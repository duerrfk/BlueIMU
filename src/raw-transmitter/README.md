raw-transmitter transmits raw measurements from the accelerometer and gyroscope of the MPU6050 IMU used by the BlueIMU device via Bluetooth (serial profile). 

The implementation is based the Arduino platform and the MPU6050 code by Jeff Rowberg.

# Data Format

Raw sensor data is transmitted over a Bluetooth serial connection. The transmitted data stream consists of records of the following format:

* Byte 0..1: Start of Record: 0x5A, 0xA5
* Byte 2..3: accelerometer X (16 bit signed int)  
* Byte 4..5: accelerometer Y (16 bit signed int)
* Byte 6..7: accelerometer Z (16 bit signed int)
* Byte 8..9: gyro X (16 bit signed int)
* Byte 10..11: gyro Y (16 bit signed int)
* Byte 12..13: gyro Z (16 bit signed int)
* Byte 14..17: sample time stamp (32 bit unsigned integer) in milliseconds since boot time
* Byte 18..19: 16 bit checksum

All 16 bit words are transmitted in Big Endian format (high byte first, low byte second). 

# Interpreting Raw Values

Values from sensors are transmitted as raw signed 16 bit values. The values can be translated to m/s and deg/s, respectively, depending on the configured sensor sensitivities (+-2g, +-4g, +-8g, +-16g; +-250 deg/s, +-500 deg/s, +- 1000 deg/s, +-2000 deg/s). The following resolutions are supported by the MPU6050 IMU:

* accelerometer: 16384, 8192, 4096, 2048 LSB / (m/s)
* gyro: 131, 65.5, 32.8, and 16.5 LSB / (deg/s) 

Please have a look at the manual of the MPU6050 for further details.

# Checksum 

The checksum consists of the ones' complement of the ones' complement sum of the 16-bit words, similar to the calculation of the checksum of IP headers. For detailed information on how to calculate the checksum, please have a look at the code. 

# Sampling Rate

The sampling rate can be adjusted by the SAMPLING_PERIOD definition.

# Low-pass Filter

The MPU 6050 features a low-pass filter. You can set the filter by defining DLPF according to the following table:


              |   ACCELEROMETER    |           GYROSCOPE
    DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
    ---------+-----------+--------+-----------+--------+-------------
    0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
    1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
    2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
    3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
    4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
    5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
    6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
    7        |   -- Reserved --   |   -- Reserved --   | Reserved

