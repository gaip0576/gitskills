#CC=/home/liwei/work/arm-jushri-linux-gnueabi-gcc
CC=/opt/buildroot-gcc342/bin/mipsel-linux-gcc
CC=/opt/buildroot-gcc463/usr/bin/mipsel-linux-gcc

OBJECT := timestamp_stat_app.o sgitg_gps.o air_signal.o

all: timestamp_stat

CFLAGS += -Os -Wall
LDLIBS += -lpthread

%.o: %.c
	$(CC) -I. $(EXTRA_CFLAGS) -c -o $@ $^

timestamp_stat: $(OBJECT)
	$(CC) $(CFLAGS) $(LDLIBS) -o $@ $^

clean:
	rm -f *.o timestamp_stat