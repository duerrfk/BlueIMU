CC=gcc

CFLAGS=-c -Wall -std=gnu99 -D_XOPEN_SOURCE=500 -D_GNU_SOURCE -DDEBUG
LDFLAGS=-lm

receiver-attitude: kalmanfilter.o receiver-attitude.o
	$(CC) kalmanfilter.o receiver-orientation.o $(LDFLAGS) -o $@

.PHONY: clean
clean:
	rm -rf receiver-attitude.o kalmanfilter receiver-attitude
