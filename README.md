# LibRtlSdrOverV4L2

This is meant as a drop-in replacement for LibRtlSdr.
It uses the v4l-sdr driver, which is part of newer Linux-Kernels.
The main advantage of using this lib:
- you do not have to unload kernel modules (rtl2832).
- Other Hardware (other than rtl2832) may also work,
  if a suiting Kernel-Module is provided 
The disadvantage:
- some functions are dummies - especially the setting of gains,
  which are not supported by the kernel-module yet (at least
  for my dongle with FC0013 tuner).

Though the io-methods READ, MMAP and USERPTR
are implemented, you should normally use READ (the default
and most inefficient), because most Software which uses
LibRtlSdr can not cope with the buffer-sizes which are
supported by the kernel-module.

I copied the original header-files of LibRtlSdr to
the include-dir. So compiling should be pretty simple .
To build just type:
make

The simplest way to use this lib, is to set the LD_LIBRARY_PATH to the
location of the new librtlsdr.so file.
Example (suppose librtlsdr.so is in /opt/LibRtlSdrOverV4L2/lib):

LD_LIBRARY_PATH=/opt/LibRtlSdrOverV4L2/lib rtl_fm -f "91.2e6" -s 192000 -M wbfm -r 48000 -g 16 - | aplay -r 48k -f S16_LE -t raw


Have fun
