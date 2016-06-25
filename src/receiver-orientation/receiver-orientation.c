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
#include "kalmanfilter.h"

#define FRAMESIZE 20
#define MAX_FRAME_ERR 1

enum States {synced, unsynced1, unsynced2} state;

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
			      // Too many corrupt frames. Possibly out of sync.
			      state = unsynced1;
#ifdef DEBUG
			      printf("UNSYNCED\n");
#endif
			 }
		    } else {
			 // Frame OK.
			 // TODO: handle frame
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
