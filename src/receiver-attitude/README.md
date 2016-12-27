# Setting up a connection between host and BlueIMU

Follow these steps to set up a serial connection between the Linux host and the BlueIMU device.

Step 1 -- Find the BlueIMU device:

$ hcitool scan
Scanning ...
	00:11:12:31:04:12	BlueIMU

So the MAC address of the BlueIMU device here is 00:11:12:31:04:12.

Step 2 -- Add configuration for BlueIMU device:

Add the following block to the file `/etc/bluetooth/rfcomm.conf`:

rfcomm1 {
  bind no;
  device 00:11:12:31:04:12;
  channel 1;
  comment "Blue IMU serial Port";
}

Step 3 -- Pairing host and BlueIMU device:

$ bluez-simple-agent hci0 00:11:12:31:04:12
RequestPinCode (/org/bluez/455/hci0/dev_00_11_12_31_04_12)
Enter PIN Code: 1234
Release
New device (/org/bluez/455/hci0/dev_00_11_12_31_04_12)

Step 4 -- Connecting host to BlueIMU device:

$ sudo rfcomm connect 1
Connected /dev/rfcomm1 to 00:11:12:31:04:12 on channel 1
Press CTRL-C for hangup

Now you should see a serial device `/dev/rfcomm1`, which can be opened by an application.

