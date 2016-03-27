
CPLUSPLUS = g++
CC = gcc
AR = ar -rs 
RM = rm -f
MAKEDIR = mkdir -p
RMDIR = rm -rf

C++FLAGS = -O2 -Wall -g2 -I/usr/X11/include -Wno-deprecated
CFLAGS = -O2 -Wall -g2 -fPIC -I/usr/X11/include

LIBS = -lm -lrt
INCLUDES = -I. -I./include

BASENAME=rtlsdr
VERSION_MAJOR=0

CSRC = $(wildcard *.c convenience/*.c src/*.c)
CPLUSPLUSSRC = $(wildcard *.cpp)
OBJS = $(CPLUSPLUSSRC:%.cpp=%.o) $(CSRC:%.c=%.o)

SHARED_LIB=lib/lib$(BASENAME).so.$(VERSION_MAJOR)
STATIC_LIB=lib/lib$(BASENAME).a

###################################################

all: $(STATIC_LIB) $(SHARED_LIB)

$(SHARED_LIB):	$(OBJS)
	$(MAKEDIR) lib
	$(RM) $@ lib/lib$(BASENAME).so
	$(CC) -shared -Wl,-soname,lib$(BASENAME).so.$(VERSION_MAJOR) -o $@ $(OBJS) $(LIBS)
	ln -s lib$(BASENAME).so.$(VERSION_MAJOR) lib/lib$(BASENAME).so

$(STATIC_LIB):	$(OBJS)
	$(MAKEDIR) lib
	$(RM) $@
	$(AR) $@ $(OBJS)

clean:
	$(RMDIR) lib
	$(RM) *.o *.a $(OBJS) $(SHARED_LIB) $(STATIC_LIB) lib/lib$(BASENAME).so

#########################

.SUFFIXES : .c .cpp .o

.cpp.o:
	$(CPLUSPLUS) -c $(C++FLAGS) $(INCLUDES) -o $@ $<

.c.o:
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<
