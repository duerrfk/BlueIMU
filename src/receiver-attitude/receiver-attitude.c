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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <math.h>
#include "kalmanfilter.h"

// Size of a frame transmitted over the Bluetooth serial connection.
#define FRAMESIZE 20

// Number of tolerated corrupt frames before the data stream is
// re-synchronized.
#define MAX_FRAME_ERR 1

// Gravitational acceleration, used to translate g values of accelerometer to
// acceleration in m/s**2.
const float grav_accel = 9.81f;

// Sampling period in seconds.
// Timestamps are sent together with samples. So manually defining the
// sampling period is not strictly required, but might be useful if
// recorded timestamps are thought to be inaccurate.
const float sampling_period = 0.005f;

// Standard deviations of the Gaussian distributions of angular measurements.
// The angle (phi) is only observed indirectly by measuring the 
// objects acceleration, assuming that acceleration is only due to gravity 
// and thus pointing towards the center of the earth. Obviously, this 
// assumption does not hold if the object is accelerated by other forces in 
// addition to gravity. 
// 0.004 is the standard deviation of an object at rest calculated from sample 
// measurements.
const float sigma_phi = 0.004f;

// Standard deviations of the Gaussian distributions of angular velocity
// measurements. Angular velocity is measured directly by the gyroscope. 
// 0.003 is the standard deviation calculated from sample measurements.
const float sigma_phidot = 0.003f;

// Standard deviation of the Gaussian distribution modeling uncontrolled
// angular acceleration. This noise depends on the external forces (e.g.,
// due to motors, wind, etc.) accelerating the object around its axes.  
const float sigma_angularaccel = 50.0/360.0 * 2.0*M_PI;

// Standard deviation of the Gaussian distribution modeling uncontrolled
// change of gyro bias. The Kalman filter can automatically estimate the gyro
// bias. However, you can set sigma_bias to 0 to switch off automatic bias 
// estimation (bias = 0). Typically, this value should be very small. 
const float sigma_bias = 0.000001f;
//const float sigma_bias = 0.0f;

// The sensitivity and value range of the accelerometer as defined by the
// following table (must match the configuration of the IMU):
//
// AFS_SEL | Full Scale Range | LSB Sensitivity
// --------+------------------+----------------
// 0       | +/- 2g           | 16384 LSB/g
// 1       | +/- 4g           | 8192 LSB/g
// 2       | +/- 8g           | 4096 LSB/g
// 3       | +/- 16g          | 2048 LSB/g
const float lsb_per_g = 8192.0f;

// The sensitivity and value range of the gyroscope as defined by the
// following table (must match the configuration of the IMU):
//
// FS_SEL | Full Scale Range   | LSB Sensitivity
// -------+--------------------+----------------
// 0      | +/- 250 degrees/s  | 131 LSB/deg/s
// 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
// 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
// 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
const float lsb_per_degs = 65.5f;

enum States {synced, unsynced1, unsynced2} state;

struct Sample {
     // Acceleration [m/s**2].
     float accel_x; 
     float accel_y; 
     float accel_z;
     // Angular velocity [rad/s].
     float gyro_x;
     float gyro_y;
     float gyro_z;
     // Timestamp in milliseconds since IMU was booted.
     uint32_t timestamp;
};

// Start of frame pattern.
const uint8_t start_of_frame[] = {0x5A, 0xA5};

int serial = -1;
FILE *out = NULL;

bool is_first_update = true;
uint32_t t_last_update;

/**
 * Gracefully exit the application.
 */
void die(int exitcode)
{
     if (serial != -1)
	  close(serial);

     if (out != NULL)
	  fclose(out);

     exit(exitcode);
}

/**
 * SIGINT handler.
 */
void sig_int(int signo)
{
     die(0);
}

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
 * Print usage information.
 */
void usage(const char *appl)
{
     fprintf(stderr, "%s -d SERIAL_DEVICE -o OUTPUT_FILE_PATH \n", appl);
}

void setup_serial(int fd)
{
     struct termios tty;
     memset(&tty, 0, sizeof tty);
     if (tcgetattr(fd, &tty) != 0) {
          perror("Could not get terminal attributes");
          die(-1);
     }
     
     // BlueIMU uses 230400 baud/s.
     cfsetospeed(&tty, B230400);
     cfsetispeed(&tty, B230400);

     tty.c_cflag &= ~CSIZE;   // 8-bit characters
     tty.c_cflag |= CS8;    
     tty.c_cflag &= ~CSTOPB;  // one stop bit (= no two stop bits)
     tty.c_cflag &= ~PARENB;  // no parity check
     tty.c_cflag |= CLOCAL;   // ignore modem control lines
     tty.c_cflag |= CREAD;    // enable receiver
     tty.c_cflag &= ~CRTSCTS; // disable hardware flow control

     // Turn off software flow control
     tty.c_iflag &= ~(IXON | IXOFF | IXANY);

     tty.c_iflag |= IGNBRK;  // no break processing
     tty.c_iflag &= ~IGNCR;  // don't discard CR characters
     tty.c_iflag &= ~INLCR;  // don't translate NL to CR
     tty.c_iflag &= ~ISTRIP; // don't strip off 8th bit
     tty.c_iflag &= ~ICRNL;  // don't translate CR to NL
     tty.c_oflag = 0;        // Turn off any output processing

     // Set non-canonical input mode (raw; non-line-oriented)
     tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

     tty.c_cc[VMIN] = 1;  // read blocks if there is not at least one byte
     tty.c_cc[VTIME] = 0; // no read timeout

     if (tcsetattr(fd, TCSANOW, &tty) != 0) {
          perror("Could not set terminal attributes");
          die(-1);
     }

     // Clear buffers.
     // Flush only seems to work with some sleep time -- possibly a USB 
     // problem, see:
     // http://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
     sleep(2);  
     tcflush(fd,TCIOFLUSH);
}

/**
 * Parse a sample transmitted in a frame.
 *
 * @param frame the frame containing the sample.
 * @param sample pointer to the allocated sample data structure.
 */
void parse_sample(uint8_t *frame, struct Sample *sample)
{
     // All values are in Big Endian byte order.
     int16_t lsb_accel_x = (((int16_t) frame[2])<<8) | frame[3];
     int16_t lsb_accel_y = (((int16_t) frame[4])<<8) | frame[5];
     int16_t lsb_accel_z = (((int16_t) frame[6])<<8) | frame[7];

     int16_t lsb_gyro_x = (((int16_t) frame[8])<<8) | frame[9];
     int16_t lsb_gyro_y = (((int16_t) frame[10])<<8) | frame[11];
     int16_t lsb_gyro_z = (((int16_t) frame[12])<<8) | frame[13];

     // Translate raw ADC readings to meaningful acceleration and 
     // angular velocity values in m/s**2 and rad/s, respectively.
     sample->accel_x = lsb_accel_x/lsb_per_g*grav_accel;
     sample->accel_y = lsb_accel_y/lsb_per_g*grav_accel;
     sample->accel_z = lsb_accel_z/lsb_per_g*grav_accel;

     sample->gyro_x = lsb_gyro_x/lsb_per_degs/360.0f*2.0f*M_PI;
     sample->gyro_y = lsb_gyro_y/lsb_per_degs/360.0f*2.0f*M_PI;
     sample->gyro_z = lsb_gyro_z/lsb_per_degs/360.0f*2.0f*M_PI;

     sample->timestamp = (((uint32_t) frame[14])<<24) |
	  (((uint32_t) frame[15])<<16) |
	  (((uint32_t) frame[16])<<8) |
	  (((uint32_t) frame[17]));

     // Default mounting orientation of the IMU is as follows:
     // 
     // * roll: around x axis
     // * pitch: around y axis
     // * yaw: around z axis
     // 
     // For roll = pitch = 0, the positive z axis is pointing away from the 
     // center of the earth.
     //
     // If the IMU is mounted differently, we need to rotate the axes.
     // The code below rotates the axes as follows: 
     //
     // * roll: around z axis
     // * pitch: around y axis
     // * yaw: around x axis
     //
     // For roll = pitch = 0, the positive x axis is pointing towards the 
     // center of the earth. 
     float tempX = sample->accel_x;
     float tempY = sample->accel_y;
     float tempZ = sample->accel_z;
     sample->accel_x = tempZ;
     sample->accel_y = tempY;
     sample->accel_z = -tempX;
     tempX = sample->gyro_x;
     tempY = sample->gyro_y;
     tempZ = sample->gyro_z;
     sample->gyro_x = tempZ;
     sample->gyro_y = tempY;
     sample->gyro_z = -tempX;
}

/**
 * Update a Kalman filter given a measurement.
 *
 * @param filter pointer to Kalman filter to be updated. 
 * @param phi measured angle [rad].
 * @param phidot measured angular velocity [rad/s].
 * @param t_last_update time of last update [ms].
 * @param t_sample time of sample [ms].
 */
void kalmanfilter_update(struct kf *filter, float phi, float phidot,
			 uint32_t t_last_update, uint32_t t_sample)
{
     float t_delta;
     if (t_last_update <= t_sample) {
	  t_delta = (float) (t_sample-t_last_update)/1000.0f;
     } else {
	  // wrap around
	  t_delta = (float) ((0xffffffffUL-t_last_update)+t_sample)/1000.0f;
     }

     // Use sampling period calculated from sample timestamps sent by BlueIMU.
     kf_update(filter, phi, phidot, t_delta);
     // Alternatively, we can use a pre-defined fixed sampling period.
     //kf_update(filter, phi, phidot, sampling_period);
}

/**
 * Calculate roll angle (rotation around x axis) from measured acceleration. 
 * We assume that acceleration is just gravity and, therefore, the acceleration 
 * vector points towards the center of the earth. 
 *
 * @param accel_y acceleration along Y axis of the IMU.
 * @param accel_z acceleration along Z axis of the IMU.
 * @return roll angle [rad]
 */
float roll_from_accel(float accel_y, float accel_z)
{
     return atan2f(accel_y, accel_z);
}

/**
 * Calculate pitch angle (rotation around y axis) from measured acceleration. 
 * We assume that acceleration is just gravity and, therefore, the acceleration 
 * vector points towards the center of the earth. 
 *
 * @param accel_x acceleration along X axis of the IMU.
 * @param accel_z acceleration along Z axis of the IMU.
 * @return pitch angle [rad]
 */
float pitch_from_accel(float accel_x, float accel_z)
{
     return -atan2f(accel_x, accel_z);
}

/**
 * Print frame contents as string of bytes on stdout.
 * 
 * @param buffer buffer with frame content.
 * @param s size of buffer.
 */
void printframe(uint8_t *buffer, size_t s)
{
     for (int i = 0; i < s; i++) {
	  printf("%d ", buffer[i]);
     }
     printf("\n");
}

/**
 * Output data as comma-separated values (CSV) in the following order:
 *
 * 1. timestamp [ms]
 * 2. measured acceleration of x axis (accelerometer x measurement) [m/s**2]
 * 3. measured acceleration of y axis (accelerometer y measurement) [m/s**2]
 * 4. measured acceleration of z axis (accelerometer z measurement) [m/s**2]
 * 5. measured angular velocity of x axis (gyro x measurement) [rad/s]
 * 6. measured angular velocity of y axis (gyro y measurement) [rad/s]
 * 7. measured angular velocity of z axis (gyro z measurement) [rad/s]
 * 8. roll calculated from acceleration measurements [rad]
 * 9. pitch calculated from acceleration measurements [rad]
 * 10. roll from Kalman filter [rad]
 * 11. pitch from Kalman filter [rad]
 * 12. angular velocity of roll from Kalman filter [rad/s]
 * 13. angular velocity of pitch from Kalman filter [rad/s]
 * 14. roll bias from Kalman filter
 * 15. pitch bias from Kalman filter
 */
void flog(FILE *f, uint32_t timestamp, 
	  float accel_x, float accel_y, float accel_z,
	  float gyro_x, float gyro_y, float gyro_z,
	  float roll_measure, float pitch_measure, 
	  float kf_roll_phi, float kf_pitch_phi,
	  float kf_roll_phidot, float kf_pitch_phidot, 
	  float kf_roll_bias, float kf_pitch_bias) 
{
     fprintf(f, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
	     timestamp, accel_x, accel_y, accel_z,
	     gyro_x, gyro_y, gyro_z,
	     roll_measure, pitch_measure,
	     kf_roll_phi, kf_pitch_phi, 
	     kf_roll_phidot, kf_pitch_phidot, 
	     kf_roll_bias, kf_pitch_bias);
}

int main(int argc, char *argv[])
{
     // Parse command line arguments.
     char *device_arg = NULL;
     char *outpath_arg = NULL;
     int c;
     while ((c = getopt(argc, argv, "d:b:o:")) != -1) {
          switch (c) {
          case 'd' :
               device_arg = malloc(strlen(optarg)+1);
               strcpy(device_arg, optarg);
               break;
	  case 'o' :
	       outpath_arg = malloc(strlen(optarg)+1);
	       strcpy(outpath_arg, optarg);
	       break;
	  }
     }

     if (device_arg == NULL) {
	  usage(argv[0]);
	  die(-1);
     }

     if (outpath_arg == NULL) {
	  usage(argv[0]);
	  die(-1);
     }

     // Kalman filters are later initialized after first sample.
     struct kf kf_pitch, kf_roll;

     // Open output file.
     if ((out = fopen(outpath_arg, "w")) == NULL) {
	  perror("Could not open output file");
	  die(-1);
     }

     // Open and configure serial device.
     if ((serial = open(device_arg, O_RDWR | O_NOCTTY)) == -1) {
          perror("Could not open serial device");
          die(-1);
     }
     setup_serial(serial);

     // Install signal handler for SIGINT for graceful termination.
     if (signal(SIGINT, sig_int) == SIG_ERR) {
          perror("Could not set signal handler: SIGINT");
          die(-1);
     }

     state = unsynced1;
     uint8_t buffer[FRAMESIZE];
     size_t nread, n;
     unsigned int nframeerr;
     struct Sample sample;
     while (1) {
	  switch (state) {
	  case unsynced1:
	       if ((n = read(serial, &buffer[0], 1)) == -1) {
		   perror("Error reading from serial port");
		   die(-1);
	       }
	       if (n == 1 && buffer[0] == start_of_frame[0])
		    state = unsynced2;
	       break;
	  case unsynced2:
	       if ((n = read(serial, &buffer[1], 1)) == -1) {
		    perror("Error reading from serial port");
		    die(-1);
	       }	    
	       if (n == 1 && buffer[1] == start_of_frame[1]) {
		    state = synced;
		    nread = 2;
		    nframeerr = 0;
#ifdef DEBUG
		    printf("synced\n");
#endif
	       }
	       break;
	  case synced:
	       if ((n = read(serial, &buffer[nread], FRAMESIZE-nread)) == -1) {
		    perror("Error reading from serial port");
		    die(-1);
	       }
	       nread += n;
	       if (nread == FRAMESIZE) {
#ifdef DEBUG
		    printframe(buffer, FRAMESIZE);
#endif
		    // Complete frame read.
		    if (checksum(buffer, FRAMESIZE) != 0) {
			 // Bad frame.
#ifdef DEBUG
			 printf("CORRUPT FRAME\n");
#endif
			 nframeerr++;
			 if (nframeerr > MAX_FRAME_ERR) {
			      // Too many corrupt frames -> resync.
			      state = unsynced1;
#ifdef DEBUG
			      printf("UNSYNCED\n");
#endif
			 }
		    } else {
			 // Frame OK.
			 parse_sample(buffer, &sample);
			 float roll = roll_from_accel(sample.accel_y,
			      sample.accel_z);
			 float pitch = pitch_from_accel(sample.accel_x,
			      sample.accel_z);
			 if (is_first_update) {
			      kf_init(&kf_pitch, pitch, sample.gyro_y, 0.0f,
				      sigma_phi, sigma_phidot, 
				      sigma_angularaccel, sigma_bias);
			      kf_init(&kf_roll, roll, sample.gyro_x, 0.0f, 
				      sigma_phi, sigma_phidot, 
				      sigma_angularaccel, sigma_bias);
			      is_first_update = false;
			 } else {
			     kalmanfilter_update(&kf_pitch, pitch, 
			         sample.gyro_y, t_last_update, 
			         sample.timestamp);
			     kalmanfilter_update(&kf_roll, roll, 
			         sample.gyro_x, t_last_update, 
			         sample.timestamp);
			 }
			 t_last_update = sample.timestamp;
			 nframeerr = 0;
			 flog(out, sample.timestamp, 
			      sample.accel_x, sample.accel_y, sample.accel_z,
			      sample.gyro_x, sample.gyro_y, sample.gyro_z,
			      roll, pitch, 
			      kf_roll.x[0], kf_pitch.x[0],
			      kf_roll.x[1], kf_pitch.x[1], 
			      kf_roll.x[2], kf_pitch.x[2]);
#ifdef DEBUG
			 float rollkf = kf_roll.x[0]/(2.0f*M_PI)*360.0f;
			 float pitchkf = kf_pitch.x[0]/(2.0f*M_PI)*360.0f;
			 printf("pitch = %f\troll = %f\n", pitchkf, rollkf);
#endif
		    }
		    nread = 0;
	       }
	       break;
	  }
     }
}
