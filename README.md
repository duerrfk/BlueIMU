BlueIMU is an open-source Inertial Measurement Unit (IMU) with Bluetooth connectivity (serial port profile / SPP). 

The main features of BlueIMU are:

* Based on inexpensive, easily available hardware components, namely, the 6 degree of freedom GY521 IMU module featuring an InvenSense MPU-6050 sensor, the HC06 Bluetooth module, and an Atmega328P microcontroller. 
* Can run from a single cell LiPo battery (overdischarge protection included in the design). 
* Raw sensor values can be transmitted at sampling rates of 200 Hz and higher.
* Software is based on the Arduino IDE and can be easily modified.
* Fully open source hardware (circuit board / PCB) and software design.
* Sample application with Kalman filter for calculating attitude (pitch and roll) included. 

The following image shows the BlueIMU board:

![BlueIMU](/images/blueimu.jpg)

# Why Yet Another IMU?

Many wireless IMUs today are based on Bluetooth Low Energy (BLE). Although BLE is optimized for very low energy consumption, it lacks support for higher bandwidth and low latency as required by sensors at higher sampling rates. In particular, the minumum connection interval of 7.5 ms limits the sampling rate of wireless sensors to 133 Hz (if your BLE central device supports such a low connection interval).  

Therefore, BlueIMU uses standard Bluetooth supporting much higher data rates (serial connection at 230400 Baud) to allow for higher sensor sampling rates at 200 Hz and higher.
 
# Hardware Design

The design of the BlueIMU circuit board (PCB) can be found in folder `pcb` (Eagle schematics and board design as well as Gerber files for production at a PCB house are included). 
  
BlueIMU is based on easily available low-cost components such as the GY521 IMU featuring a 6 degrees of freedom InvenSense MPU-6050 sensor, the HC06 Bluetooth board, and the Atmega328P microcontroller, which is well-supported by the popular Arduino platform.

The following images shows the schematic of the BlueIMU board.

![Schematic](/images/schematic.png)

The Atmega328P microcontroller (MCU) runs at 7.3728 MHz to allow for a perfect timing of the 230400 Baud serial connection to the HC06 Bluetooth module (230400 = 7.3728 MHz/32). Besides the serial connection, the HC06 module signals the Bluetooth connection state through a pin changing with a period of 750 ms while not connected and being constantly high while connected. 

A 6 pin connector is used to connect an in-system programmer with the following common pinout:

```
MISO <-- 1 2 --> VCC
SCK <--  3 4 --> MOSI
RST <--  5 6 --> GND
```

The GY521 module is connected through I2C to the MCU.

An overdischarge protection is included using the ??? voltage monitor. The MCU, IMU, and Bluetooth module will be disconnected from power at about 3.0 V. At about 3.2 V, the LED (charge indicator) goes off to signal a low battery (the GY521 module has another onboard LED to show that the board is still running down to 3.0 V).  

## Programming the Atmega328P

Program the following fuses (note that "1" means fuse *not* programmed; "0" means fuse programmed):

Low Power Crystal Oscillator 3-8 MHz, 16k +14k (BOD enabled) 
* CKSEL 1101 
* SUT 01

BOD enabled at 2.7 V
* BODLEVEL 101 

Enable serial programming
* SPIEN = 0 

Boot reset vector not enabled
* BOOTRST = 1

External reset not disabled
* RSTDISBL = 1

This results in the following fuse bytes: 

* Low = 0xDD
* High = 0xD9
* Extended = 0x05

The MCU can be programmed using avrdude as follows:

```
$ sudo avrdude -c usbasp -p m328p -U lfuse:w:0xdd:m -U hfuse:w:0xd9:m -U efuse:w:0x05:m
```

## Flashing the Software

In folder `src/transmitter-raw` you will find software (Arduino sketch) for BlueIMU transmitting raw sensor values (3 axes acceleration and 3 axes angular velocity) and a timestamp for each sample at a sampling rate of 200 Hz.

In folder `arduino_board_definition` you will find the correct board definition for the Atmega328P MCU at 7.3728 MHz. Copy this directory into the folder `hardware` in your Arduino sketchbook.

Then, compile the Arduino sketch from folder `src/transmitter-raw`. This generates a hex file for the Atmega328P MCU. This hex file is a little bit hidden in the temporary build directory of the Arduino IDE. If you use Linux  and Arduino IDE 1.6, have a look at the `/tmp`  directory. After hitting the  compile button in the Arduino IDE, search for the latest hex file called  `transmitte-raw.cpp.hex` in a temporary directory named `/tmp/build...`. If you have found the hex file, you can flash it using avrdude:

$ sudo avrdude -p m328p -c usbasp -v -U flash:w:transmitter-raw.cpp.hex

# Connecting a Linux host to BlueIMU over Bluetooth

You can connect a Linux host to the BlueIMU board via Bluetooth as follows:

Step 1: Find the BlueIMU device 

```
$ hcitool scan
Scanning ...
	00:11:12:31:04:12	BlueIMU
```

Step 2: Pairing Linux host and BlueIMU device:

```
$ bluetooth-agent 1234 00:11:12:31:04:12
```

Step 3: Add device profile

```
$ sudo nano /etc/bluetooth/rfcomm.conf
```

Add the following section:

```
rfcomm1 {
  bind no;
  device 00:11:12:31:04:12;
  channel 1;
  comment "Blue IMU serial Port";
}
```

Connect host to BlueIMU device: 

```
$ rfcomm connect 1
```

Now, you should have a serial device called `/dev/rfcomm1` that you can open with your application. For instance, you can use screen to log data as follows:

```
$ screen -L /dev/rfcomm1
```

# Kalman Filter

In folder `src/receiver-attitude` you find a sample application for Linux implementing a Kalman filter to calculate the attitude (pitch and roll) and logging raw sensor values and Kalman filter values (pitch, roll, estimated gyro bias) to a file as comma-separate values. Note that the GY521 IMU does not feature a magnetometer (compass), thus, we cannot easily detect the absolute heading (yaw). 

You can compile this application with the following commands:

```
$ cd src/receiver-attitude
$ make 
```

Then you start it as follows:

```
$ ./receiver-attitude -d /dev/rfcomm1 -o out.csv
```

The Kalman filter uses the following equations:

State vector (phi is the angle; phidot is the angular velocity; bias is the bias of angular velocity (gyro bias)): 

```
    [ phi  ] 
x = [phidot]
    [ bias ] 
```

State equation:

```
x(k) = F(k)*x(k-1) + omega(k)
```

with

```
       [1 t_delta -t_delta]
F(k) = [0    1        0   ]
       [0    0        1   ]
```

This implementation assumes no control input model (B). Instead, a normally distributed random angular acceleration (phidotdot) is introduced modeling the effect of external forces like motors as part of the random process noise (omega):

```
           [t_delta**2/2]
omega(k) = [  t_delta   ] * phidotdot
           [     0      ]
```

Covariance matrix Q of normally distributed random process noise omega ~ N(0,Q):

```
Q = G*Gtrans*sigma_phidotdot**2 

    [t_delta**4/4  t_delta**3/2  0]
  = [t_delta**3/2   t_delta**2   0] * sigma_phidotdot**2 +
    [      0             0       0]

    [ 0 0 0 ]
    [ 0 0 0 ] * sigma_bias**2
    [ 0 0 1 ]
```

Moreover, we assume that we can measure the angle from the IMU acceleration (phi_m) and the angular velocity (phidot_m) from the gyroscope. However, we cannot measure the gyro bias. Measurements are modeled as independent normally distributed random variables to account for measurement noise. The covariance matrix R of normally distributed random measuring noise v ~ N(0,R) is defined as:

```
    [sigma_phi_m**2         0         ]
R = 
    [     0          sigma_phidot_m**2]
```

Sample data of a stationary IMU and an IMU attached to a pendulum can be found in folder `sample-data`. 

The following plot shows the angle of a pendulum in radians over time as calculated by the Kalman filter:

![Angle pendulum Kalman](images/angle_pendulum_kalman.png)

As a comparison, here the angle in radians over time calculated directly from the accelerometer without Kalman filter, i.e., without fusing measurements from accelerometer and gyroscope:

![Angle pendulum accelerometer](images/angle_pendulum_accelerometer.png)

Observer how the angle calculated just from acceleration is only correct while the object is at rest and clearly wrong when the pendulum moves such that gravity cannot be easily distinguished anymore from the acceleration caused by centripetal force. In that case, the Kalman filter can use the angular velocity to predict the angle leading to much more accurate results.