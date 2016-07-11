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

// Standard deviations of the Gaussian distributions of measurements.
// The angle (phi) is only observed indirectly by measuring the 
// objects acceleration by the accelerometer and assuming that acceleration 
// points to the center of the earth due to gravity. Obviously, the
// acceleration vector might not point towards the center of the earth
// if the object is accelerated by other forces than gravity.
// Therefore, we assume that in general angular measurements have a quite 
// broad distribution.
const float sigma_phi = .1/360.0 * 2.0*M_PI;
// Angular velocity is measured directly by the gyroscope and should be 
// quite accurate.
const float sigma_phidot = 0.1/360.0 * 2.0*M_PI;

// Standard deviation of the Gaussian distribution modelling the uncontrolled
// angular acceleration. This noise depends on the external forces (e.g.,
// due to motors, wind, etc.) that accelerate the object around its axes.  
const float sigma_angularaccel = 1.0f/360.0 * 2.0*M_PI;

// The sensitivity and value range of the accelerometer as defined by the
// following table:
//
// AFS_SEL | Full Scale Range | LSB Sensitivity
// --------+------------------+----------------
// 0       | +/- 2g           | 16384 LSB/g
// 1       | +/- 4g           | 8192 LSB/g
// 2       | +/- 8g           | 4096 LSB/g
// 3       | +/- 16g          | 2048 LSB/g
const float lsb_per_g = 16384.0f;

// The sensitivity and value range of the gyroscope as defined by the
// following table:
//
// FS_SEL | Full Scale Range   | LSB Sensitivity
// -------+--------------------+----------------
// 0      | +/- 250 degrees/s  | 131 LSB/deg/s
// 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
// 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
// 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
const float lsb_per_degs = 131.0f;

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

// Start of frame pattern
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
}

/**
 * Parse a sample transported in a frame.
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
     sample->accel_x = lsb_accel_x/lsb_per_g*9.81f;
     sample->accel_y = lsb_accel_y/lsb_per_g*9.81f;
     sample->accel_z = lsb_accel_z/lsb_per_g*9.81f;

     sample->gyro_x = lsb_gyro_x/lsb_per_degs/360.0f*2.0f*M_PI;
     sample->gyro_y = lsb_gyro_y/lsb_per_degs/360.0f*2.0f*M_PI;
     sample->gyro_z = lsb_gyro_z/lsb_per_degs/360.0f*2.0f*M_PI;

     sample->timestamp = (((uint32_t) frame[14])<<24) |
	  (((uint32_t) frame[15])<<16) |
	  (((uint32_t) frame[16])<<8) |
	  (((uint32_t) frame[17]));
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

     kf_update(filter, phi, phidot, t_delta);
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

#ifdef DEBUG
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
#endif

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
#ifdef DEBUG
			 printf("correct frame\n");
			 printf("accel_x = %f g\taccel_y = %f g\t"
				"accel_z = %f g\tgyro_x= %f rad/s\t"
				"gyro_y = %f rad/s\tgyro_z = %f rad/s\t"
				"t = %u\n", sample.accel_x,
				sample.accel_y, sample.accel_z,
				sample.gyro_x, sample.gyro_y, sample.gyro_z,
				sample.timestamp);
#endif

			 float roll = roll_from_accel(sample.accel_y,
			      sample.accel_z);
			 float pitch = pitch_from_accel(sample.accel_x,
			      sample.accel_z);
			 if (is_first_update) {
			      kf_init(&kf_pitch, pitch, sample.gyro_y, 
				      sigma_phi, sigma_phidot, 0.0f, 
				      sigma_angularaccel);
			      kf_init(&kf_roll, roll, sample.gyro_x, 
				      sigma_phi, sigma_phidot, 0.0f, 
				      sigma_angularaccel);
			      is_first_update = false;
			 } else {
			     kalmanfilter_update(&kf_pitch, pitch, 
			         sample.gyro_y, t_last_update, 
			         sample.timestamp);
			     kalmanfilter_update(&kf_roll, roll, 
			         sample.gyro_x, t_last_update, 
			         sample.timestamp);
			 }
#ifdef DEBUG
			 float roll_deg = roll/(2.0f*M_PI)*360.0f;
			 float pitch_deg = pitch/(2.0f*M_PI)*360.0f;
			 printf("Measure: roll = %f deg\n",
				roll_deg);
			 printf("Measure: pitch = %f deg\n",
				pitch_deg);
			 roll_deg = kf_roll.x[0]/(2.0f*M_PI)*360.0f;
			 pitch_deg = kf_pitch.x[0]/(2.0f*M_PI)*360.0f;
			 printf("Kalman: roll = %f deg "
				"v = %f [rad/s] "
				"bias = %f [rad/s]\n",
				roll_deg, kf_roll.x[1], kf_roll.x[2]);
			 printf("Kalman: pitch = %f deg "
				"v = %f [rad/s] "
				"bias = %f [rad/s]\n",
				roll_deg, kf_pitch.x[1], kf_pitch.x[2]);
#endif
			 t_last_update = sample.timestamp;
			 nframeerr = 0;
		    }
		    nread = 0;
	       }
	       break;
	  }
     }
}
