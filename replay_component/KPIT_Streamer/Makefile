TARGET1=KPIT_Streamer
TARGET2=KPIT_PipeStreamer
TARGET3=KPIT_VideoReader
LIBS=lusb-1.0
all:
	g++ -fPIC -o lib/libcyusb.o -c lib/libcyusb.cpp
	g++ -shared -Wl,-soname,libcyusb.so -o lib/libcyusb.so.1 lib/libcyusb.o -l usb-1.0 -l rt
	gcc -o $(TARGET1) lib/libcyusb.cpp src/KPIT_streamer_ReadFile.cpp src/ADAS_Support.c -lusb-1.0 -pthread
	gcc -o $(TARGET2) lib/libcyusb.cpp src/KPIT_streamer_ReadPipe.cpp src/ADAS_Support.c -lusb-1.0 -pthread
	gcc -o $(TARGET3) src/KPIT_VidReader.cpp src/ADAS_Support.c -lusb-1.0 -pthread
	cd lib; ln -sf libcyusb.so.1 libcyusb.so
	rm -f lib/libcyusb.o
clean:
	rm -f lib/libcyusb.so lib/libcyusb.so.1
help:
	@echo	'make		would compile and create the library and create a link'
	@echo	'make clean	would remove the library and the soft link to the library (soname)'
