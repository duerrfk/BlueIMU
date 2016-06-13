# System clock / Serial clock

* System clock: 7.3728 MHz
* Serial clock (baud rate): 7.3728 MHz / 32 = 230400 Hz

# Fuses

"1" means fuse *not* programmed; "0" means fuse programmed

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

Low: 	  0xDD
High: 	  0xD9
Extended: 0x05

$ sudo avrdude -c usbasp -p m328p -U lfuse:w:0xdd:m -U hfuse:w:0xd9:m -U efuse:w:0x05:m

# HC06 Module

## LED
  
* if not connected, blinks with period 750 ms 
* if connected, constantly on 

# Bluetooth and Linux

Find device: 

$ hcitool scan
Scanning ...
	00:11:12:31:04:12	BlueIMU

Pair device:

$ bluetooth-agent 1234 00:11:12:31:04:12

Add device profile:

$ sudo nano /etc/bluetooth/rfcomm.conf

rfcomm1 {
  bind no;
  device 00:11:12:31:04:12;
  channel 1;
  comment "Blue IMU serial Port";
}

Connect to device, start interactive session, finally release connection:

$ rfcomm connect 1
$ screen /dev/rfcomm1
$ sudo rfcomm release 1

Log data sent over serial connection:

$ screen -L /dev/rfcomm1
