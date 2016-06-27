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

// Sigma values of the Gaussian distributions of measurements.
// The angle (phi) is only observed indirectly by measuring the 
// objects acceleration with the accelerometer and assuming that acceleration 
// points to the center of the earth due to gravity. Obviously, when the 
// object accelerates, this is not valid. Therefore, we assume that
// in general angular measurements have a quite broad distribution.
const float sigma_phi = 10.0/360.0 * 2.0*M_PI;
// Angular velocity is measured directly by the gyroscope and should be 
// quite accurate.
const float sigma_phidot = 0.1/360.0 * 2.0*M_PI;

// Sigma values of the Gaussian distribution of the uncontrolled
// angular acceleration. This noise depends on the external forces
// that accelerate the object around its axis (e.g., motors).  
const float sigma_angularaccel = 1.0/360.0 * 2.0*M_PI;

// The sensitivity and value range of the accelerometer as defined by the
// following table:
//
// AFS_SEL | Full Scale Range | LSB Sensitivity
// --------+------------------+----------------
// 0       | +/- 2g           | 8192 LSB/mg
// 1       | +/- 4g           | 4096 LSB/mg
// 2       | +/- 8g           | 2048 LSB/mg
// 3       | +/- 16g          | 1024 LSB/mg
const float lsb_to_mg = 8192.0f;

// The sensitivity and value range of the gyroscope as defined by the
// following table:
//
// FS_SEL | Full Scale Range   | LSB Sensitivity
// -------+--------------------+----------------
// 0      | +/- 250 degrees/s  | 131 LSB/deg/s
// 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
// 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
// 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
const float lsb_to_degs = 131.0f;

enum States {synced, unsynced1, unsynced2} state;

struct Sample {
     // Acceleration in g.
     float accelX; 
     float accelY; 
     float accelZ;
     // Angular velocity in deg/s.
     float gyroX;
     float gyroY;
     float gyroZ;
     // Timestamp in milliseconds since device reboot.
     uint32_t timestamp;
};

// Start of frame pattern
const uint8_t start_of_frame[] = {0x5A, 0xA5};

int serial = -1;
FILE *out = NULL;

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
     // All values are in Big Endiand byte order.
     int16_t lsb_accelX = (((uint16_t) frame[2])<<8) | frame[3];
     int16_t lsb_accelY = (((uint16_t) frame[4])<<8) | frame[5];
     int16_t lsb_accelZ = (((uint16_t) frame[6])<<8) | frame[7];

     int16_t lsb_gyroX = (((uint16_t) frame[8])<<8) | frame[9];
     int16_t lsb_gyroY = (((uint16_t) frame[10])<<8) | frame[11];
     int16_t lsb_gyroZ = (((uint16_t) frame[12])<<8) | frame[13];

     // Translate raw ADC readings to meaningful acceleration and 
     // angular velocity values.
     sample->accelX = lsb_to_mg/1000.0f * lsb_accelX;
     sample->accelY = lsb_to_mg/1000.0f * lsb_accelY;
     sample->accelZ = lsb_to_mg/1000.0f * lsb_accelZ;

     sample->gyroX = lsb_to_degs * lsb_gyroX;
     sample->gyroY = lsb_to_degs * lsb_gyroY;
     sample->gyroZ = lsb_to_degs * lsb_gyroZ;

     sample->timestamp = (((uint32_t) frame[14])<<24) |
	  (((uint32_t) frame[15])<<16) |
	  (((uint32_t) frame[16])<<8) |
	  (((uint32_t) frame[17]));
}

void kalmanfilter_update(const struct Sample *sample)
{
     
}

#ifdef DEBUG
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

     // Initialize Kalman filters.
     // Initial angle: 0 rad
     // Initial angular velocity: 0 rad/s
     struct kf kf_pitch, kf_roll;
     kf_init(&kf_pitch, 0.0f, 0.0f, 
	     sigma_phi, sigma_phidot, sigma_angularaccel);
     kf_init(&kf_roll, 0.0f, 0.0f, 
	     sigma_phi, sigma_phidot, sigma_angularaccel);

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
			 kalmanfilter_update(&sample);
#ifdef DEBUG
			 printf("correct frame\n");
#endif
			 nframeerr = 0;
		    }
		    nread = 0;
	       }
	       break;
	  }
     }
}
