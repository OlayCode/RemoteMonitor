#ifndef _CAMERA_FUN_H
#define _CAMERA_FUN_H

#include <linux/videodev2.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <pthread.h>
#include <jpeglib.h>
#include <jerror.h>
#include <semaphore.h>

#define		OUTPUT_BUF_SIZE	4096
#define		VIDEO_WIDTH 	640//640
#define 	VIDEO_HEIGHT	480//480
#define 	BUFFER_COUNT	4
#define 	FILE_VIDEO	"/dev/video0"
#define		SUCCESS		0
#define		FAILURE		-1

typedef int status;

extern sem_t sem_ca, sem_lcd;
extern pthread_mutex_t mutex_web;


typedef struct jpeg_buffer{
    unsigned char  start[VIDEO_WIDTH * VIDEO_HEIGHT * 2];
    unsigned int length;
} ImageBuffer;

typedef struct rgb_buffer{
    unsigned char  start[VIDEO_WIDTH * VIDEO_HEIGHT * 3];
    unsigned int length;
} LcdBuffer;

typedef struct {
	struct jpeg_destination_mgr pub;
	JOCTET * buffer; 
	unsigned char *outbuffer;
	int outbuffer_size;
	unsigned char *outbuffer_cursor;
	int *written; 
} mjpg_destination_mgr;
 
typedef mjpg_destination_mgr *mjpg_dest_ptr;
 
void dest_buffer(j_compress_ptr cinfo, unsigned char *buffer, int size, int *written);

status camera_init(void);
void camera_close(void);

status get_camera_jpg(int fd, ImageBuffer *jpg_buf,LcdBuffer *rgb_buf);

int do_camera(ImageBuffer *imgbuf, LcdBuffer *lcdbuf);

void yuyv_2_rgb888(unsigned char *yuyv,LcdBuffer *rgb_buf);

#endif
