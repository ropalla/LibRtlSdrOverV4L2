/*
* author:        Rudolf Opalla
* Last-Change:   2015-07-26
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include <rtl-sdr.h>

#define MAX_STRING 80
#define SWRADIO_NAME "swradio"
#define DEVICES_DIR "/dev"

#define DEFAULT_BUF_NUMBER      10
#define DEFAULT_BUF_LENGTH      (128 * 512)

typedef enum {
  RTLSDR_INACTIVE = 0,
  RTLSDR_CANCELING,
  RTLSDR_RUNNING
} rtlsdr_async_status;

typedef enum {
  IO_METHOD_READ,
  IO_METHOD_MMAP,
  IO_METHOD_USERPTR,
} rtlsdr_io_method;

struct rtlsdr_buf {
  unsigned char  *start;
  int             length;
};

struct rtlsdr_dev {
  int             fd;
  FILE           *fp;
  rtlsdr_io_method io_method;

  char            path[MAX_STRING];
  struct v4l2_capability vCaps;

  uint32_t        xfer_buf_num;
  uint32_t        xfer_requested_buf_len;
  struct rtlsdr_buf *xfer_buf;

  rtlsdr_read_async_cb_t cb;
  void           *cb_ctx;
  rtlsdr_async_status async_status;
};

static struct rtlsdr_dev *devices = NULL;
static int      devicesUsed = 0;
static int      devicesAllocated = 0;

/* static rtlsdr_io_method default_io_method = IO_METHOD_MMAP; */
static rtlsdr_io_method default_io_method = IO_METHOD_READ;

/*  Static helper-functions */

static int
xioctl(int fh, int request, void *arg)
{
  int             r;

  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);
  return r;
}

static int
_rtlsdr_get_v4l2_control(rtlsdr_dev_t * dev, int control_id)
{
  struct v4l2_control control;

  memset(&control, 0, sizeof(struct v4l2_control));
  control.id = control_id;
  if (-1 == ioctl(dev->fd, VIDIOC_G_CTRL, &control)) {
    perror("swradio_get_v4l2_control VIDIOC_G_CTRL");
    return -1;
  }
  return control.value;
}

static int
_rtlsdr_set_v4l2_control(rtlsdr_dev_t * dev, int control_id, uint32_t value)
{
  struct v4l2_control control;

  memset(&control, 0, sizeof(struct v4l2_control));
  control.id = control_id;
  control.value = value;
  if (xioctl(dev->fd, VIDIOC_S_CTRL, &control) < 0) {
    perror("VIDIOC_S_CTRL");
    return -1;
  }
  return 0;
}

static int
_rtlsdr_close(struct rtlsdr_dev *dev)
{
  char            errMessage[MAX_STRING];
  int             ret;

  if (dev->fd >= 0) {
    /*
     * free all attached resources 
     */
    if (dev->fp != NULL) {
      ret = fclose(dev->fp);
      dev->fp = NULL;
    } else {
      ret = close(dev->fd);
    }
    if (ret != 0) {
      snprintf(errMessage, MAX_STRING, "Can not close 'SWRADIO DEVICE': %s", dev->path);
      perror(errMessage);
    }
    dev->fd = -1;
    return ret;
  }
  return -1;
}

static int
_rtlsdr_open(struct rtlsdr_dev *dev, char *device_name)
{
  char            errMessage[MAX_STRING];
  struct v4l2_frequency vFreq;
  struct v4l2_format vFormat;

  if (dev == NULL) {
    fprintf(stderr, "Could not open NULL SWRadio Device!\n");
    return -1;
  }

  if (dev->fd >= 0) {
    fprintf(stderr, "SWRadio Device is already open!\n");
    return -1;
  }

  strncpy(dev->path, device_name, MAX_STRING);

  if (default_io_method == IO_METHOD_READ) {
    dev->fd = open(dev->path, O_RDWR);
  } else {
    dev->fd = open(dev->path, O_RDWR | O_NONBLOCK);
  }
  if (dev->fd < 0) {
    snprintf(errMessage, MAX_STRING, "Can not open 'SWRADIO DEVICE': %s", dev->path);
    perror(errMessage);
    return -1;
  }
  dev->io_method = default_io_method;
  if (xioctl(dev->fd, VIDIOC_QUERYCAP, &(dev->vCaps)) < 0) {
    snprintf(errMessage, MAX_STRING, "Can not get caps from %s", dev->path);
    perror(errMessage);
    _rtlsdr_close(dev);
    return -2;
  }
  if (!(dev->vCaps.capabilities & V4L2_CAP_SDR_CAPTURE)) {
    fprintf(stderr, "V4L2_CAP_SDR_CAPTURE is not in %s\n", dev->path);
    _rtlsdr_close(dev);
    return -2;
  }

  if ((!(dev->vCaps.capabilities & V4L2_CAP_STREAMING)) && (!(dev->vCaps.capabilities & V4L2_CAP_READWRITE))) {
    fprintf(stderr, "%s does not support read or streaming i/o\n", dev->path);
    _rtlsdr_close(dev);
    return (-2);
  }
  if ((dev->io_method == IO_METHOD_READ) && (!(dev->vCaps.capabilities & V4L2_CAP_READWRITE))) {
    fprintf(stderr, "V4L2_CAP_READWRITE is not set -> falling back to IO_METHOD_MMAP\n");
    dev->io_method = IO_METHOD_MMAP;
  }
  if (!(dev->vCaps.capabilities & V4L2_CAP_STREAMING)) {
    fprintf(stderr, "V4L2_CAP_STREAMING is not set -> falling back to IO_METHOD_READ\n");
    dev->io_method = IO_METHOD_READ;
  }
  memset(&vFormat, 0, sizeof(struct v4l2_format));
  vFreq.tuner = 1;
  if (xioctl(dev->fd, VIDIOC_G_FREQUENCY, &vFreq) < 0) {
    snprintf(errMessage, MAX_STRING, "Can not get frequency from %s", dev->path);
    perror(errMessage);
    _rtlsdr_close(dev);
    return -3;
  }
  memset(&vFormat, 0, sizeof(struct v4l2_format));
  vFormat.type = V4L2_BUF_TYPE_SDR_CAPTURE;
  if (xioctl(dev->fd, VIDIOC_G_FMT, &vFormat) < 0) {
    snprintf(errMessage, MAX_STRING, "Query format failed %s", dev->path);
    perror(errMessage);
    _rtlsdr_close(dev);
    return -4;
  }
  if (dev->io_method == IO_METHOD_READ) {
    dev->fp = fdopen(dev->fd, "rw");
    if (dev->fp == NULL) {
      /*
       * it is not fatal to fail here - just means, that read has to be used instead of fread 
       */
      perror("fdopen failed");
    }
  }

  return 0;
}


static int
_rtlsdr_enqueue_v4l2_buffer(struct v4l2_buffer *buf, rtlsdr_dev_t * dev, int i)
{
  memset(buf, 0, sizeof(struct v4l2_buffer));
  buf->type = V4L2_BUF_TYPE_SDR_CAPTURE;
  buf->index = i;
  if (dev->io_method == IO_METHOD_MMAP) {
    buf->memory = V4L2_MEMORY_MMAP;
  } else if (dev->io_method == IO_METHOD_USERPTR) {
    buf->memory = V4L2_MEMORY_USERPTR;
    buf->m.userptr = (unsigned long) dev->xfer_buf[i].start;
    buf->length = dev->xfer_buf[i].length;
  }
  if (xioctl(dev->fd, VIDIOC_QBUF, buf) < 0) {
    perror("VIDIOC_QBUF");
    return -3;
  }
  return 0;
}

static int
_rtlsdr_dequeue_v4l2_buffer(struct v4l2_buffer *buf, rtlsdr_dev_t *dev)
{
  memset(buf, 0, sizeof(struct v4l2_buffer));
  buf->type = V4L2_BUF_TYPE_SDR_CAPTURE;
  if (dev->io_method == IO_METHOD_MMAP) {
    buf->memory = V4L2_MEMORY_MMAP;
  } else if (dev->io_method == IO_METHOD_USERPTR) {
    buf->memory = V4L2_MEMORY_USERPTR;
  }
  if (xioctl(dev->fd, VIDIOC_DQBUF, buf) < 0) {
    perror("VIDIOC_DQBUF");
    return -4;
  }
  return 0;
}



static int
_rtlsdr_free_async_buffers(rtlsdr_dev_t * dev)
{
  int             i,
                  ret;

  ret = 0;
  if (dev->xfer_buf != NULL) {
    for (i = 0; i < dev->xfer_buf_num; i++) {
      if (dev->xfer_buf[i].start != NULL) {
	if (dev->io_method == IO_METHOD_MMAP) {
	  if (munmap(dev->xfer_buf[i].start, dev->xfer_buf[i].length) < 0) {
	    perror("Failed UNMAPPING ");
	    ret = -2;
	  }
	} else {		/* READ and USERPTR */
	  free(dev->xfer_buf[i].start);
	}
	dev->xfer_buf[i].start = NULL;
      }
    }
    free(dev->xfer_buf);
    dev->xfer_buf = NULL;
  } else {
    ret -= 1;
  }
  return ret;
}

static int
_rtlsdr_alloc_async_buffers(rtlsdr_dev_t * dev, int buf_num, uint32_t buf_len)
{
  int             i;
  struct v4l2_requestbuffers req;
  struct v4l2_buffer buf;

  if (buf_num > 0) {
    dev->xfer_buf_num = buf_num;
  } else {
    dev->xfer_buf_num = DEFAULT_BUF_NUMBER;
  }
  if (buf_len > 0 && buf_len % sysconf(_SC_PAGESIZE) == 0) {	/* len must be multiple of page_size (512) */
    dev->xfer_requested_buf_len = buf_len;
  } else {
    dev->xfer_requested_buf_len = DEFAULT_BUF_LENGTH;
  }

  if ((dev->io_method == IO_METHOD_MMAP) || (dev->io_method == IO_METHOD_USERPTR)) {
    memset(&req, 0, sizeof(struct v4l2_requestbuffers));
    req.count = dev->xfer_buf_num;
    req.type = V4L2_BUF_TYPE_SDR_CAPTURE;
    if (dev->io_method == IO_METHOD_MMAP) {
      req.memory = V4L2_MEMORY_MMAP;
    } else if (dev->io_method == IO_METHOD_USERPTR) {
      req.memory = V4L2_MEMORY_USERPTR;
    }
    if (xioctl(dev->fd, VIDIOC_REQBUFS, &req) < 0) {
      perror("VIDIOC_REQBUFS failed");
      return -1;
    }
  }

  dev->xfer_buf = (void *) calloc(dev->xfer_buf_num, sizeof(struct rtlsdr_buf));
  if (dev->xfer_buf == NULL) {
    perror("Allocating xfer_buf");
    return -2;
  }
  for (i = 0; i < dev->xfer_buf_num; i++) {
    if (dev->io_method == IO_METHOD_MMAP) {
      
      if (_rtlsdr_enqueue_v4l2_buffer(&buf, dev, i) < 0) {
	return -3;
      }
      dev->xfer_buf[i].start = (unsigned char *)
	mmap(NULL /* start anywhere */ ,
	     buf.length, PROT_READ | PROT_WRITE /* required */ ,
	     MAP_SHARED /* recommended */ ,
	     dev->fd, buf.m.offset);
      if (MAP_FAILED == dev->xfer_buf[i].start) {
	perror("mmap failed");
	return -4;
      }
      dev->xfer_buf[i].length = buf.length;
    } else {
      dev->xfer_buf[i].start = (unsigned char *) malloc(dev->xfer_requested_buf_len);
      if (dev->xfer_buf[i].start == NULL) {
	perror("Allocating xfer_buf part");
	_rtlsdr_free_async_buffers(dev);
	return (-5);
      }
      dev->xfer_buf[i].length = dev->xfer_requested_buf_len;
      if (dev->io_method == IO_METHOD_USERPTR) {
        if (_rtlsdr_enqueue_v4l2_buffer(&buf, dev, i) < 0) {
	  return -6;
        }
      }
    }
  }

  return dev->xfer_buf_num;
}


static int
_rtlsdr_start_reading(rtlsdr_dev_t * dev)
{
  enum v4l2_buf_type h_type;
  long            fctlret;

  if ((dev == NULL) || (dev->fd < 0)) {
    return -1;
  }
  if (dev->async_status == RTLSDR_INACTIVE) {
    return (-2);
  }

  if ((dev->io_method == IO_METHOD_MMAP) || (dev->io_method == IO_METHOD_USERPTR)) {
    fctlret = fcntl(dev->fd, F_GETFL);
    if ((fctlret & O_NONBLOCK) == 0) {
      if (fcntl(dev->fd, F_SETFL, fctlret | O_NONBLOCK) < 0) {
	perror("_rtlsdr_start_reading setting O_NONBLOCK failed! ");
	return -3;
      }
    }

    h_type = V4L2_BUF_TYPE_SDR_CAPTURE;
    if (xioctl(dev->fd, VIDIOC_STREAMON, &h_type) < 0) {
      perror("VIDIOC_STREAMON");
      return -1;
    }
  }

  return 0;
}

static int
_rtlsdr_stop_reading(rtlsdr_dev_t * dev)
{
  enum v4l2_buf_type h_type;

  if ((dev == NULL) || (dev->fd < 0)) {
    return -1;
  }
  if (dev->async_status == RTLSDR_INACTIVE) {
    return (-2);
  }

  if ((dev->io_method == IO_METHOD_MMAP) || (dev->io_method == IO_METHOD_USERPTR)) {
    h_type = V4L2_BUF_TYPE_SDR_CAPTURE;
    if (xioctl(dev->fd, VIDIOC_STREAMOFF, &h_type) < 0) {
      perror("VIDIOC_STREAMOFF");
      return -1;
    }
  }

  return 0;
}

/* returns the used buffer-index */
static int
_rtlsdr_get_next_package(rtlsdr_dev_t * dev,
			 int last_buffer_index, unsigned char **ret_package_start, uint32_t * ret_package_len)
{
  int             i,
                  n_read,
                  ret,
                  next_buffer_index;
  struct v4l2_buffer buf;
  fd_set          fds;
  struct timeval  tv;

  if ((dev == NULL) || (dev->fd < 0)) {
    return -1;
  }
  if (dev->async_status != RTLSDR_RUNNING) {
    return (-2);
  }

  next_buffer_index = -1;

  if (dev->io_method == IO_METHOD_READ) {
    next_buffer_index = last_buffer_index + 1;
    if ((next_buffer_index > dev->xfer_buf_num) || (next_buffer_index < 0)) {
      next_buffer_index = 0;
    }
    ret = rtlsdr_read_sync(dev, dev->xfer_buf[next_buffer_index].start, dev->xfer_buf[next_buffer_index].length, &n_read);
    *ret_package_start = dev->xfer_buf[next_buffer_index].start;
    *ret_package_len = n_read;
    if (ret < 0) {
      return ret;
    }
  } else if ((dev->io_method == IO_METHOD_MMAP) || (dev->io_method == IO_METHOD_USERPTR)) {
    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    ret = select(dev->fd + 1, &fds, NULL, NULL, &tv);
    if (ret < 0) {
      if (errno != EINTR) {
	perror("select");
      }
    } else if (ret == 0) {
      fprintf(stderr, "No data in time (%ld.%ld Seconds)\n", tv.tv_sec, tv.tv_usec);
      /*
       * No data in time (two Seconds) 
       */
    } else {
      if ((last_buffer_index >= 0) && (last_buffer_index < dev->xfer_buf_num)) {
	if (_rtlsdr_enqueue_v4l2_buffer(&buf, dev, last_buffer_index) < 0) {
	  /* continue without this buffer */
	}
      }
      if (_rtlsdr_dequeue_v4l2_buffer(&buf, dev) < 0) {
	return -3;
      }
      if (dev->io_method == IO_METHOD_MMAP) {
	if (buf.index >= dev->xfer_buf_num) {
	  fprintf(stderr, "MMAP buffer-index out of range %d\n", next_buffer_index);
	  return (-4);
	}
	next_buffer_index = buf.index;
	*ret_package_start = dev->xfer_buf[next_buffer_index].start;
	*ret_package_len = buf.bytesused;
      } else if (dev->io_method == IO_METHOD_USERPTR) {
	for (i = 0; (i < dev->xfer_buf_num) && (next_buffer_index < 0); ++i) {
	  if ((buf.m.userptr == (unsigned long) dev->xfer_buf[i].start)
	      && (buf.length == dev->xfer_buf[i].length)) {
	    next_buffer_index = i;
	  }
	}
	if (next_buffer_index >= dev->xfer_buf_num) {
	  fprintf(stderr, "USERPTR buffer-index out of range %d\n", next_buffer_index);
	  return -6;
	}
	*ret_package_start = (unsigned char *) buf.m.userptr;
	*ret_package_len = buf.bytesused;
      }
    }
  }
  return next_buffer_index;
}

/* functions for the interface */

RTLSDR_API      uint32_t
rtlsdr_get_device_count(void)
{
  DIR            *dp;
  struct dirent  *current_entry;
  char            device_name[MAX_STRING];

  if (devices == NULL) {
    devices = (struct rtlsdr_dev *) malloc(2 * sizeof(struct rtlsdr_dev));
    devicesAllocated = 2;

    dp = opendir(DEVICES_DIR);
    if (dp == NULL) {
      return 0;
    }

    current_entry = readdir(dp);
    while (current_entry != NULL) {
      if (strncmp(SWRADIO_NAME, current_entry->d_name, strlen(SWRADIO_NAME)) == 0) {
	if (devicesUsed >= devicesAllocated) {
	  devicesAllocated += 2;
	  devices = (struct rtlsdr_dev *) realloc(devices, devicesAllocated * sizeof(struct rtlsdr_dev));
	}
	memset((devices + devicesUsed), 0, sizeof(struct rtlsdr_dev));
	devices[devicesUsed].fd = -1;
	snprintf(device_name, MAX_STRING, "%s/%s", DEVICES_DIR, current_entry->d_name);
	if (_rtlsdr_open(devices + devicesUsed, device_name) >= 0) {
	  _rtlsdr_close(devices + devicesUsed);
	  devicesUsed++;
	}
      }
      current_entry = readdir(dp);
    }
    closedir(dp);
  }

  return (devicesUsed);
}

RTLSDR_API const char *
rtlsdr_get_device_name(uint32_t index)
{
  if (index >= devicesUsed) {
    return NULL;
  }

  return ((char *) devices[index].vCaps.card);
}


RTLSDR_API int
rtlsdr_get_device_usb_strings(uint32_t index, char *manufact, char *product, char *serial)
{
  if (index >= devicesUsed) {
    return -2;
  }

  return (rtlsdr_get_usb_strings(&(devices[index]), manufact, product, serial));
}

RTLSDR_API int
rtlsdr_get_index_by_serial(const char *serial)
{
  int             i;

  if (serial == NULL) {
    return -1;
  }
  if (devicesUsed == 0) {
    return -2;
  }
  for (i = 0; i < devicesUsed; i++) {
    if (strcmp(serial, (char *) devices[i].vCaps.bus_info) == 0) {
      return (i);
    }
  }

  return -3;
}

RTLSDR_API int
rtlsdr_open(rtlsdr_dev_t ** retdev, uint32_t index)
{
  char            errMessage[MAX_STRING];
  struct v4l2_format vFormat;
  rtlsdr_dev_t   *dev;
  int             ret;

  if (index >= devicesUsed) {
    return -1;
  }
  dev = (devices + index);
  ret = _rtlsdr_open(dev, dev->path);
  if (ret < 0) {
    return ret;
  }

  memset(&vFormat, 0, sizeof(struct v4l2_format));
  vFormat.type = V4L2_BUF_TYPE_SDR_CAPTURE;
  vFormat.fmt.sdr.pixelformat = V4L2_SDR_FMT_CU8;
  if (xioctl(dev->fd, VIDIOC_S_FMT, &vFormat) < 0) {
    snprintf(errMessage, MAX_STRING, "Setting format CU8 failed %s", dev->path);
    perror(errMessage);
    _rtlsdr_close(dev);
    return -5;
  }

  *retdev = dev;
  return (ret);
}

RTLSDR_API int
rtlsdr_close(rtlsdr_dev_t * dev)
{
  return _rtlsdr_close(dev);
}


/* configuration functions */

RTLSDR_API int
rtlsdr_set_xtal_freq(rtlsdr_dev_t * dev, uint32_t rtl_freq, uint32_t tuner_freq)
{
  fprintf(stderr, "set_xtal_freq is not implemented\n");

  return -1;
}

RTLSDR_API int
rtlsdr_get_xtal_freq(rtlsdr_dev_t * dev, uint32_t * rtl_freq, uint32_t * tuner_freq)
{
  fprintf(stderr, "get_xtal_freq is not implemented\n");

  return -1;
}


RTLSDR_API int
rtlsdr_get_usb_strings(rtlsdr_dev_t * dev, char *manufact, char *product, char *serial)
{
  if (dev == NULL) {
    return -1;
  }

  strncpy(manufact, (char *) dev->vCaps.driver, 256);
  strncpy(product, (char *) dev->vCaps.card, 256);
  strncpy(serial, (char *) dev->vCaps.bus_info, 256);
  return (0);
}

RTLSDR_API int
rtlsdr_write_eeprom(rtlsdr_dev_t * dev, uint8_t * data, uint8_t offset, uint16_t len)
{
  fprintf(stderr, "rtlsdr_write_eeprom is not implemented\n");

  return -3;
}

RTLSDR_API int
rtlsdr_read_eeprom(rtlsdr_dev_t * dev, uint8_t * data, uint8_t offset, uint16_t len)
{
  fprintf(stderr, "rtlsdr_write_eeprom is not implemented\n");

  return -3;
}

RTLSDR_API int
rtlsdr_set_center_freq(rtlsdr_dev_t * dev, uint32_t freq)
{
  struct v4l2_frequency vFreq;

  if ((dev == NULL) || (dev->fd < 0)) {
    fprintf(stderr, "rtlsdr_set_center_freq device is NULL or closed\n");
    return -1;
  }

  memset(&vFreq, 0, sizeof(struct v4l2_frequency));
  vFreq.tuner = 1;
  vFreq.type = V4L2_TUNER_RF;
  vFreq.frequency = freq;
  if (xioctl(dev->fd, VIDIOC_S_FREQUENCY, &vFreq) < 0) {
    perror("rtlsdr_set_center_freq set freq failed");
    return -3;
  }

  return 0;
}


RTLSDR_API      uint32_t
rtlsdr_get_center_freq(rtlsdr_dev_t * dev)
{
  struct v4l2_frequency vFreq;

  if ((dev == NULL) || (dev->fd < 0)) {
    fprintf(stderr, "rtlsdr_get_center_freq device is NULL or closed\n");
    return 0;
  }

  memset(&vFreq, 0, sizeof(struct v4l2_frequency));
  vFreq.tuner = 1;
  vFreq.type = V4L2_TUNER_RF;
  if (xioctl(dev->fd, VIDIOC_G_FREQUENCY, &vFreq) < 0) {
    perror("rtlsdr_set_center_freq get freq failed");
    return 0;
  }
  return (vFreq.frequency);
}


RTLSDR_API int
rtlsdr_set_freq_correction(rtlsdr_dev_t * dev, int ppm)
{
  fprintf(stderr, "rtlsdr_set_freq_correction not implemented yet\n");
  return -1;
}


RTLSDR_API int
rtlsdr_get_freq_correction(rtlsdr_dev_t * dev)
{
  fprintf(stderr, "rtlsdr_get_freq_correction not implemented yet\n");
  return -1;
}

RTLSDR_API enum rtlsdr_tuner
rtlsdr_get_tuner_type(rtlsdr_dev_t * dev)
{
  return RTLSDR_TUNER_UNKNOWN;
}


RTLSDR_API int
rtlsdr_get_tuner_gains(rtlsdr_dev_t * dev, int *gains)
{
  fprintf(stderr, "rtlsdr_get_tuner_gains not implemented yet\n");
  return 0;  /* -1 leads to exception with sdr-j-fm */
}


RTLSDR_API int
rtlsdr_set_tuner_gain(rtlsdr_dev_t * dev, int gain)
{
  fprintf(stderr, "rtlsdr_set_tuner_gain not implemented yet\n");
  return -1;
}

RTLSDR_API int
rtlsdr_get_tuner_gain(rtlsdr_dev_t * dev)
{
  fprintf(stderr, "rtlsdr_get_tuner_gain not implemented yet\n");
  return -1;
}

RTLSDR_API int
rtlsdr_set_tuner_if_gain(rtlsdr_dev_t * dev, int stage, int gain)
{
  fprintf(stderr, "rtlsdr_set_tuner_if_gain not implemented yet\n");
  return -1;
}

RTLSDR_API int
rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t * dev, int manual)
{
  fprintf(stderr, "rtlsdr_set_tuner_gain_mode not implemented yet\n");
  return -1;
}

RTLSDR_API int
rtlsdr_set_sample_rate(rtlsdr_dev_t * dev, uint32_t rate)
{
  struct v4l2_frequency vFreq;

  if ((dev == NULL) || (dev->fd < 0)) {
    fprintf(stderr, "rtlsdr_set_center_freq device is NULL or closed\n");
    return -1;
  }

  memset(&vFreq, 0, sizeof(struct v4l2_frequency));
  vFreq.tuner = 0;
  vFreq.type = V4L2_TUNER_ADC;
  vFreq.frequency = rate;
  if (xioctl(dev->fd, VIDIOC_S_FREQUENCY, &vFreq) < 0) {
    perror("rtlsdr_set_center_freq set freq failed");
    return -2;
  }

  return 0;
}

RTLSDR_API      uint32_t
rtlsdr_get_sample_rate(rtlsdr_dev_t * dev)
{
  struct v4l2_frequency vFreq;

  if ((dev == NULL) || (dev->fd < 0)) {
    fprintf(stderr, "rtlsdr_get_sample_rate device is NULL or closed\n");
    return 0;
  }

  memset(&vFreq, 0, sizeof(struct v4l2_frequency));
  vFreq.tuner = 0;
  vFreq.type = V4L2_TUNER_ADC;
  if (xioctl(dev->fd, VIDIOC_G_FREQUENCY, &vFreq) < 0) {
    perror("rtlsdr_set_center_freq get freq failed");
    return 0;
  }
  return (vFreq.frequency);
}

RTLSDR_API int
rtlsdr_set_testmode(rtlsdr_dev_t * dev, int on)
{
  fprintf(stderr, "rtlsdr_set_testmode is not implemented\n");

  return -1;
}


RTLSDR_API int
rtlsdr_set_agc_mode(rtlsdr_dev_t * dev, int on)
{
  fprintf(stderr, "rtlsdr_set_agc_mode is not implemented\n");

  return -1;
}

RTLSDR_API int
rtlsdr_set_direct_sampling(rtlsdr_dev_t * dev, int on)
{
  fprintf(stderr, "rtlsdr_set_direct_sampling is not implemented\n");

  return -1;
}

RTLSDR_API int
rtlsdr_get_direct_sampling(rtlsdr_dev_t * dev)
{
  fprintf(stderr, "rtlsdr_get_direct_sampling is not implemented\n");

  return -1;
}

RTLSDR_API int
rtlsdr_set_offset_tuning(rtlsdr_dev_t * dev, int on)
{
  fprintf(stderr, "rtlsdr_set_offset_tuning is not implemented\n");

  return -1;
}

RTLSDR_API int
rtlsdr_get_offset_tuning(rtlsdr_dev_t * dev)
{
  fprintf(stderr, "rtlsdr_get_offset_tuning is not implemented\n");

  return -1;
}

/* streaming functions */

RTLSDR_API int
rtlsdr_reset_buffer(rtlsdr_dev_t * dev)
{
  _rtlsdr_free_async_buffers(dev);

  return 0;
}


RTLSDR_API int
rtlsdr_read_sync(rtlsdr_dev_t * dev, void *buf, int len, int *n_read)
{
  int             retlen;
  long            fctlret;

  if ((dev == NULL) || (dev->fd < 0)) {
    *n_read = 0;
    return -1;
  }
  /* force READ always in blocking mode */
  if (dev->io_method != IO_METHOD_READ) {
    fctlret = fcntl(dev->fd, F_GETFL);
    if ((fctlret & O_NONBLOCK) != 0) {
      if (fcntl(dev->fd, F_SETFL, fctlret ^ O_NONBLOCK) < 0) {
	perror("rtlsdr_read_sync reverting O_NONBLOCK failed! ");
	*n_read = 0;
	return -2;
      }
    }
  }

  if (dev->fp != NULL) {
    retlen = fread(buf, 1, len, dev->fp);
  } else {
    retlen = read(dev->fd, buf, len);
  }
  if (retlen < 0) {
    perror("rtlsdr_read_sync failed: ");
    *n_read = 0;
    return retlen;
  } else {
    *n_read = retlen;
  }

  return 0;
}

RTLSDR_API int
rtlsdr_wait_async(rtlsdr_dev_t * dev, rtlsdr_read_async_cb_t cb, void *ctx)
{
  return rtlsdr_read_async(dev, cb, ctx, 0, 0);
}

RTLSDR_API int
rtlsdr_read_async(rtlsdr_dev_t * dev, rtlsdr_read_async_cb_t cb, void *ctx, uint32_t buf_num, uint32_t buf_len)
{
  uint32_t        package_len;
  int             last_buffer_index,
                  res;
  unsigned char  *package_start;

  if (dev == NULL) {
    return -1;
  }

  if (RTLSDR_INACTIVE != dev->async_status) {
    return -2;
  }
  if (cb == NULL) {
    return -3;
  }
  dev->async_status = RTLSDR_RUNNING;
  dev->cb = cb;
  dev->cb_ctx = ctx;

  res = _rtlsdr_alloc_async_buffers(dev, buf_num, buf_len);
  if (res < 0) {
    return (res);
  }
  res = _rtlsdr_start_reading(dev);
  if (res < 0) {
    _rtlsdr_free_async_buffers(dev);
    return (res);
  }
  last_buffer_index = -1;
  while (RTLSDR_INACTIVE != dev->async_status) {
    if (RTLSDR_CANCELING == dev->async_status) {
      if (_rtlsdr_stop_reading(dev) < 0) {
	fprintf(stderr, "stop asynchron reading failed\n");
      }
      dev->async_status = RTLSDR_INACTIVE;
    } else {
      last_buffer_index = _rtlsdr_get_next_package(dev, last_buffer_index, &package_start, &package_len);
      if ((last_buffer_index >= 0) && (package_len > 0)) {
	dev->cb(package_start, package_len, dev->cb_ctx);
      }
    }
  }
  if (_rtlsdr_free_async_buffers(dev) < 0) {
    fprintf(stderr, "free async buffers failed\n");
  }

  dev->async_status = RTLSDR_INACTIVE;

  return 0;
}


/*!
 * Cancel all pending asynchronous operations on the device.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return 0 on success
 */
RTLSDR_API int
rtlsdr_cancel_async(rtlsdr_dev_t * dev)
{
  if (dev == NULL) {
    return -1;
  }

  /*
   * if streaming, try to cancel gracefully 
   */
  if (RTLSDR_RUNNING == dev->async_status) {
    dev->async_status = RTLSDR_CANCELING;
    return 0;
  }

  return -2;
}



/* *** extensions *** */

RTLSDR_API void
rtlsdr_set_default_io_method_read()
{
  default_io_method = IO_METHOD_READ;
}

RTLSDR_API void
rtlsdr_set_default_io_method_mmap()
{
  default_io_method = IO_METHOD_MMAP;
}

RTLSDR_API void
rtlsdr_set_default_io_method_userptr()
{
  default_io_method = IO_METHOD_USERPTR;
}

RTLSDR_API int
rtlsdr_set_bandwidth_auto(rtlsdr_dev_t * dev, int val)
{
 return  _rtlsdr_set_v4l2_control(dev, V4L2_CID_RF_TUNER_BANDWIDTH_AUTO, val);
}

RTLSDR_API int
rtlsdr_get_bandwidth_auto(rtlsdr_dev_t * dev)
{
  return _rtlsdr_get_v4l2_control(dev, V4L2_CID_RF_TUNER_BANDWIDTH_AUTO);
}


RTLSDR_API int
rtlsdr_set_tuner_bandwidth(rtlsdr_dev_t *dev, uint32_t bw)
{
 return  _rtlsdr_set_v4l2_control(dev, V4L2_CID_RF_TUNER_BANDWIDTH, bw);
}

RTLSDR_API int
rtlsdr_get_tuner_bandwidth(rtlsdr_dev_t * dev)
{
  return _rtlsdr_get_v4l2_control(dev, V4L2_CID_RF_TUNER_BANDWIDTH);
}

RTLSDR_API void
rtlsdr_print_status(rtlsdr_dev_t * dev)
{
  int             i,
                  j,
                  hi;
  struct v4l2_frequency vFreq;
  struct v4l2_format vFormat;
  struct v4l2_ext_controls ctrls;
  struct v4l2_output vout;
  struct v4l2_queryctrl queryctrl_result;
  struct v4l2_query_ext_ctrl query_ext_ctrl_result;
  struct v4l2_querymenu v4l2_querymenu_result;
  struct {
    int             id;
    char           *name;
  } caps[] = {
    {
    V4L2_CAP_VIDEO_CAPTURE, "V4L2_CAP_VIDEO_CAPTURE"}, {
    V4L2_CAP_VIDEO_OUTPUT, "V4L2_CAP_VIDEO_OUTPUT"}, {
    V4L2_CAP_VIDEO_OVERLAY, "V4L2_CAP_VIDEO_OVERLAY"}, {
    V4L2_CAP_VBI_CAPTURE, "V4L2_CAP_VBI_CAPTURE"}, {
    V4L2_CAP_VBI_OUTPUT, "V4L2_CAP_VBI_OUTPUT"}, {
    V4L2_CAP_SLICED_VBI_CAPTURE, "V4L2_CAP_SLICED_VBI_CAPTURE"}, {
    V4L2_CAP_SLICED_VBI_OUTPUT, "V4L2_CAP_SLICED_VBI_OUTPUT"}, {
    V4L2_CAP_RDS_CAPTURE, "V4L2_CAP_RDS_CAPTURE"}, {
    V4L2_CAP_VIDEO_OUTPUT_OVERLAY, "V4L2_CAP_VIDEO_OUTPUT_OVERLAY"}, {
    V4L2_CAP_HW_FREQ_SEEK, "V4L2_CAP_HW_FREQ_SEEK"}, {
    V4L2_CAP_RDS_OUTPUT, "V4L2_CAP_RDS_OUTPUT"}, {
    V4L2_CAP_VIDEO_CAPTURE_MPLANE, "V4L2_CAP_VIDEO_CAPTURE_MPLANE"}, {
    V4L2_CAP_VIDEO_OUTPUT_MPLANE, "V4L2_CAP_VIDEO_OUTPUT_MPLANE"}, {
    V4L2_CAP_VIDEO_M2M_MPLANE, "V4L2_CAP_VIDEO_M2M_MPLANE"}, {
    V4L2_CAP_VIDEO_M2M, "V4L2_CAP_VIDEO_M2M"}, {
    V4L2_CAP_TUNER, "V4L2_CAP_TUNER"}, {
    V4L2_CAP_AUDIO, "V4L2_CAP_AUDIO"}, {
    V4L2_CAP_RADIO, "V4L2_CAP_RADIO"}, {
    V4L2_CAP_MODULATOR, "V4L2_CAP_MODULATOR"}, {
    V4L2_CAP_SDR_CAPTURE, "V4L2_CAP_SDR_CAPTURE"}, {
    V4L2_CAP_EXT_PIX_FORMAT, "V4L2_CAP_EXT_PIX_FORMAT"}, {
    V4L2_CAP_READWRITE, "V4L2_CAP_READWRITE"}, {
    V4L2_CAP_ASYNCIO, "V4L2_CAP_ASYNCIO"}, {
    V4L2_CAP_STREAMING, "V4L2_CAP_STREAMING"}, {
    V4L2_CAP_DEVICE_CAPS, "V4L2_CAP_DEVICE_CAPS"}, {
    0, NULL}
  };
  struct {
    int             id;
    char           *name;
  } tunertypes[] = {
    {
    V4L2_TUNER_RADIO, "V4L2_TUNER_RADIO"}, {
    V4L2_TUNER_ANALOG_TV, "V4L2_TUNER_ANALOG_TV"}, {
    V4L2_TUNER_DIGITAL_TV, "V4L2_TUNER_DIGITAL_TV"}, {
    V4L2_TUNER_ADC, "V4L2_TUNER_ADC"}, {
    V4L2_TUNER_RF, "V4L2_TUNER_RF"}, {
    0, NULL}
  };
  struct {
    int             id;
    char           *name;
  } buftypes[] = {
    {
    V4L2_BUF_TYPE_VIDEO_CAPTURE, "V4L2_BUF_TYPE_VIDEO_CAPTURE"}, {
    V4L2_BUF_TYPE_VIDEO_OUTPUT, "V4L2_BUF_TYPE_VIDEO_OUTPUT"}, {
    V4L2_BUF_TYPE_VIDEO_OVERLAY, "V4L2_BUF_TYPE_VIDEO_OVERLAY"}, {
    V4L2_BUF_TYPE_VBI_CAPTURE, "V4L2_BUF_TYPE_VBI_CAPTURE"}, {
    V4L2_BUF_TYPE_VBI_OUTPUT, "V4L2_BUF_TYPE_VBI_OUTPUT"}, {
    V4L2_BUF_TYPE_SLICED_VBI_CAPTURE, "V4L2_BUF_TYPE_SLICED_VBI_CAPTURE"}, {
    V4L2_BUF_TYPE_SLICED_VBI_OUTPUT, "V4L2_BUF_TYPE_SLICED_VBI_OUTPUT"}, {
    V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY, "V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY"}, {
    V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, "V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE"}, {
    V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, "V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE"}, {
    V4L2_BUF_TYPE_SDR_CAPTURE, "V4L2_BUF_TYPE_SDR_CAPTURE"}, {
    V4L2_BUF_TYPE_PRIVATE, "V4L2_BUF_TYPE_PRIVATE"}, {
    0, NULL}
  };


  if ((dev == NULL) || (dev->fd < 0)) {
    fprintf(stderr, "Device is not Open\n");
  }
  printf("Device-Path: %s\n", dev->path);
  printf("Device-Card: %s\n", dev->vCaps.card);
  printf("Device-Driver: %s\n", dev->vCaps.driver);
  printf("Device-bus_info: %s\n", dev->vCaps.bus_info);
  printf("Device-CAPS: \n");
  i = 0;
  while (caps[i].name != NULL) {
    if ((caps[i].id & dev->vCaps.capabilities) != 0) {
      printf("  %s\n", caps[i].name);
    }
    i++;
  }
  for (j = 0; j < 2; j++) {
    memset(&vFreq, 0, sizeof(struct v4l2_frequency));
    vFreq.tuner = j;
    if (xioctl(dev->fd, VIDIOC_G_FREQUENCY, &vFreq) < 0) {
      perror("get freq failed");
    } else {
      printf("Tuner %d: frequency: %d", j, vFreq.frequency);
      i = 0;
      while (tunertypes[i].name != NULL) {
	if (tunertypes[i].id == vFreq.type) {
	  printf("  type: %s", tunertypes[i].name);
	}
	i++;
      }
      printf("\n");
    }
  }

  j = 0;
  while (buftypes[j].name != NULL) {
    memset(&vFormat, 0, sizeof(struct v4l2_format));
    vFormat.type = buftypes[j].id;
    if (xioctl(dev->fd, VIDIOC_G_FMT, &vFormat) < 0) {
      if (V4L2_BUF_TYPE_SDR_CAPTURE == buftypes[j].id) {
	perror("Query format failed");
      }
    } else {
      printf("Format: ");
      i = 0;
      while (buftypes[i].name != NULL) {
	if (buftypes[i].id == vFormat.type) {
	  printf("  buftype: %s", buftypes[i].name);
	}
	i++;
      }
      if (V4L2_BUF_TYPE_SDR_CAPTURE == buftypes[j].id) {
	printf("  sdr-pixformat: ");
	for (i = 0; i < 4; i++) {
	  hi = vFormat.fmt.sdr.pixelformat;
	  hi = hi >> (i * 8);
	  hi = hi & 255;
	  printf("%c", hi);
	}
	printf(" buffer-size: %d", vFormat.fmt.sdr.buffersize);
      }
      printf("\n");
    }
    j++;
  }

  memset(&queryctrl_result, 0, sizeof(struct v4l2_queryctrl));
  queryctrl_result.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  while (0 == ioctl(dev->fd, VIDIOC_QUERYCTRL, &queryctrl_result)) {
    /*
     * ... 
     */
    printf("ctrls.id=%d - %s\n", queryctrl_result.id, queryctrl_result.name);
    queryctrl_result.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }

  memset(&query_ext_ctrl_result, 0, sizeof(struct v4l2_query_ext_ctrl));
  query_ext_ctrl_result.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  while (0 == ioctl(dev->fd, VIDIOC_QUERY_EXT_CTRL, &query_ext_ctrl_result)) {
    /*
     * ... 
     */
    printf("ext-ctrls.id=%d - %s %d\n", query_ext_ctrl_result.id, query_ext_ctrl_result.name,
	   query_ext_ctrl_result.elems);
    query_ext_ctrl_result.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }

  memset(&v4l2_querymenu_result, 0, sizeof(struct v4l2_querymenu));
  v4l2_querymenu_result.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  while (0 == ioctl(dev->fd, VIDIOC_QUERYMENU, &v4l2_querymenu_result)) {
    /*
     * ... 
     */
    printf("menu.id=%d - %s\n", v4l2_querymenu_result.id, v4l2_querymenu_result.name);
    v4l2_querymenu_result.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }


  /*
   * check for device hardware MPEG decoding capability 
   */
  memset(&ctrls, 0, sizeof(struct v4l2_ext_controls));
  ctrls.ctrl_class = V4L2_CTRL_CLASS_RF_TUNER;
  ctrls.count = 0;
  ctrls.controls = NULL;
  if (xioctl(dev->fd, VIDIOC_G_EXT_CTRLS, &ctrls) < 0) {
    perror("VIDIOC_G_EXT_CTRLS failed: ");
  }
  printf("V4L2_CTRL_CLASS_RF_TUNER:ctrls: %d\n", ctrls.count);

  memset(&ctrls, 0, sizeof(struct v4l2_ext_controls));
  ctrls.ctrl_class = V4L2_CID_RF_TUNER_CLASS_BASE;
  ctrls.count = 0;
  ctrls.controls = NULL;
  if (xioctl(dev->fd, VIDIOC_G_EXT_CTRLS, &ctrls) < 0) {
    perror("V4L2_CID_RF_TUNER_CLASS_BASE failed: ");
  }
  printf("V4L2_CID_RF_TUNER_CLASS_BASE:ctrls: %d\n", ctrls.count);

  memset(&ctrls, 0, sizeof(struct v4l2_ext_controls));
  ctrls.ctrl_class = V4L2_CID_RF_TUNER_CLASS;
  ctrls.count = 0;
  ctrls.controls = NULL;
  if (xioctl(dev->fd, VIDIOC_G_EXT_CTRLS, &ctrls) < 0) {
    perror("V4L2_CID_RF_TUNER_CLASS failed: ");
  }
  printf("V4L2_CID_RF_TUNER_CLASS:ctrls: %d\n", ctrls.count);


  /*
   * list available outputs 
   */
  memset(&vout, 0, sizeof(struct v4l2_output));
  vout.index = 0;
  printf("Available video outputs: ");
  while (ioctl(dev->fd, VIDIOC_ENUMOUTPUT, &vout) >= 0) {
    printf("'#%d, %s' ", vout.index, vout.name);
    vout.index++;
  }
  printf("\n");

  struct {
    int             id;
    char           *name;
  } ll[] = {
    {
    V4L2_CID_RF_TUNER_BANDWIDTH_AUTO, "V4L2_CID_RF_TUNER_BANDWIDTH_AUTO"}, {
    V4L2_CID_RF_TUNER_BANDWIDTH, "V4L2_CID_RF_TUNER_BANDWIDTH"}, {
    V4L2_CID_RF_TUNER_LNA_GAIN_AUTO, "V4L2_CID_RF_TUNER_LNA_GAIN_AUTO,"}, {
    V4L2_CID_RF_TUNER_MIXER_GAIN_AUTO, "V4L2_CID_RF_TUNER_MIXER_GAIN_AUTO"}, {
    V4L2_CID_RF_TUNER_IF_GAIN_AUTO, "V4L2_CID_RF_TUNER_IF_GAIN_AUTO"}, {
    V4L2_CID_RF_TUNER_LNA_GAIN, "V4L2_CID_RF_TUNER_LNA_GAIN"}, {
    V4L2_CID_RF_TUNER_MIXER_GAIN, "V4L2_CID_RF_TUNER_MIXER_GAIN"}, {
    V4L2_CID_RF_TUNER_IF_GAIN, "V4L2_CID_RF_TUNER_IF_GAIN"}, {
  V4L2_CID_RF_TUNER_PLL_LOCK, "V4L2_CID_RF_TUNER_PLL_LOCK"},};
  i = 0;
  while (ll[i].name != NULL) {
    printf("%s:  %d\n", ll[i].name, _rtlsdr_get_v4l2_control(dev, ll[i].id));
    i++;
  }
}
