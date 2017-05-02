/*************************************************************************
  > File Name: camera.c
	> Author: 
	> Mail: 
	> Created Time: 2016年04月25日 星期一 10时48分27秒
 ************************************************************************/

#include "camera_fun.h"

int do_camera(ImageBuffer *imgbuf, LcdBuffer *lcdbuf)
{
	int camera_fd;

	camera_fd = camera_init();
	if (camera_fd < 0) {
		printf("init failed.\n");
		return -1;
	}else{
		printf("init success!\n");
	}

	while (1)
	{
		get_camera_jpg(camera_fd, imgbuf, lcdbuf);
	}
	return 0;
}
