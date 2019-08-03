ALL_PROGS = v4l2copy v4l2convert_yuv v4l2source_yuv v4l2dump
CFLAGS = -W -Wall -pthread -g -pipe $(CFLAGS_EXTRA) -I include -ljpeg
RM = rm -rf
CC = $(CROSS)gcc
CXX = $(CROSS)g++
PREFIX?=/usr

# log4cpp
ifneq ($(wildcard $(SYSROOT)$(PREFIX)/include/log4cpp/Category.hh),)
$(info with log4cpp)
CFLAGS += -DHAVE_LOG4CPP -I $(SYSROOT)$(PREFIX)/include
LDFLAGS += -llog4cpp 
endif

# v4l2wrapper
CFLAGS += -I v4l2wrapper/inc

.DEFAULT_GOAL := all

# raspberry tools using ilclient

ifneq ($(HAVE_RPI),)
CFLAGS  += -DOMX_SKIP64BIT
CFLAGS  += -lpthread -lopenmaxil -lbcm_host -lvcos -lvchostif -lvchiq_arm -lilclient

v4l2compress_omx: src/encode_omx.cpp src/v4l2compress_omx.cpp libv4l2wrapper.a
	$(CXX) -o $@ $^ -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -Wno-psabi $(CFLAGS) $(LDFLAGS) 

v4l2grab_h264: src/encode_omx.cpp src/v4l2grab_h264.cpp libv4l2wrapper.a
	$(CXX) -o $@ $^ -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -Wno-psabi $(CFLAGS) $(LDFLAGS) 

v4l2display_h264: src/v4l2display_h264.cpp libv4l2wrapper.a
	$(CXX) -o $@ $^ -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -Wno-psabi $(CFLAGS) $(LDFLAGS) 
	
ALL_PROGS+=v4l2grab_h264
ALL_PROGS+=v4l2display_h264
ALL_PROGS+=v4l2compress_omx
endif

# opencv
#ifneq ($(wildcard /usr/include/opencv),)
#ALL_PROGS+=v4l2detect_yuv
#endif

# libx264
ifneq ($(HAVE_X264),)
ALL_PROGS+=v4l2compress_h264
endif

# libx265
ifneq ($(HAVE_X265),)
ALL_PROGS+=v4l2compress_x265
endif

# libvpx
ifneq ($(HAVE_LIBVPX),)
ALL_PROGS+=v4l2compress_vpx
endif

# libjpeg
ifneq ($(HAVE_LIBJPEG),)
ALL_PROGS+=v4l2compress_jpeg v4l2uncompress_jpeg
CFLAGS += -DHAVE_JPEG
endif

# libfuse
ifneq ($(HAVE_LIBFUSE),)
ALL_PROGS+=v4l2fuse
endif

all: $(ALL_PROGS)

libyuv.a:
	cp libyuv/libyuv.a .

libv4l2wrapper.a: 
	make CC="$(CC)" CFLAGS_EXTRA="$(CFLAGS_EXTRA)" LDFLAGS="$(LDFLAGS)" -C v4l2wrapper all
	cp v4l2wrapper/libv4l2wrapper.a .

# read V4L2 capture -> write V4L2 output
v4l2copy: src/v4l2copy.cpp  libv4l2wrapper.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS)

# read V4L2 capture -> convert YUV format -> write V4L2 output
v4l2convert_yuv: src/v4l2convert_yuv.cpp  libyuv.a libv4l2wrapper.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS) -I libyuv/include

# -> write V4L2 output
v4l2source_yuv: src/v4l2source_yuv.cpp  libv4l2wrapper.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS)

# read V4L2 capture -> compress using libvpx -> write V4L2 output
v4l2compress_vpx: src/v4l2compress_vpx.cpp libyuv.a  libv4l2wrapper.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS) -lvpx -I libyuv/include

# read V4L2 capture -> compress using x264 -> write V4L2 output
v4l2compress_h264: src/v4l2compress_h264.cpp libyuv.a  libv4l2wrapper.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS) -lx264 -I libyuv/include

# read V4L2 capture -> compress using x265 -> write V4L2 output
v4l2compress_x265: src/v4l2compress_x265.cpp libyuv.a  libv4l2wrapper.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS) -lx265 -I libyuv/include

# read V4L2 capture -> compress using libjpeg -> write V4L2 output
v4l2compress_jpeg: src/v4l2compress_jpeg.cpp libyuv.a  libv4l2wrapper.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS) -ljpeg -I libyuv/include

# read V4L2 capture -> uncompress using libjpeg -> write V4L2 output
v4l2uncompress_jpeg: src/v4l2uncompress_jpeg.cpp libyuv.a  libv4l2wrapper.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS) -ljpeg -I libyuv/include
	
# try with opencv
v4l2detect_yuv: src/v4l2detect_yuv.cpp libyuv.a  libv4l2wrapper.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS) -lopencv_core -lopencv_objdetect -lopencv_imgproc -I libyuv/include

h264bitstream/Makefile:
	cd h264bitstream && autoreconf -i -f && ./configure --host $(shell $(CC) -dumpmachine)

h264bitstream/.libs/libh264bitstream.so: h264bitstream/Makefile
	make CC="$(CC)" CFLAGS="$(CFLAGS)" LDFLAGS="$(LDFLAGS)" -C h264bitstream

hevcbitstream/Makefile:
	cd hevcbitstream && autoreconf -i -f && LDFLAGS=-lm ./configure --host $(shell $(CC) -dumpmachine)

hevcbitstream/.libs/libhevcbitstream.so: hevcbitstream/Makefile
	make CC="$(CC)" CFLAGS="$(CFLAGS)" LDFLAGS="$(LDFLAGS)" -C hevcbitstream

v4l2dump: src/v4l2dump.cpp libv4l2wrapper.a h264bitstream/.libs/libh264bitstream.so  hevcbitstream/.libs/libhevcbitstream.so libyuv.a
	$(CXX) -o $@ $(CFLAGS) $^ $(LDFLAGS) -Ih264bitstream  -Ihevcbitstream -Wl,-rpath=./h264bitstream/.libs,-rpath=./hevcbitstream/.libs -I libyuv/include 

v4l2fuse: src/v4l2fuse.c 
	$(CC) -o $@ $(CFLAGS) $^ $(LDFLAGS) -D_FILE_OFFSET_BITS=64 -lfuse
	
install: all
	mkdir -p $(PREFIX)/bin
	install -D -m 0755 $(ALL_PROGS) $(PREFIX)/bin

clean:
	-@$(RM) $(ALL_PROGS) .*o *.a
