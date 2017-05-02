#include "camera_fun.h"

sem_t sem_ca, sem_lcd;

pthread_mutex_t mutex_web;

struct buffer {
    void * start;
    unsigned int length;
} *buffers;

struct v4l2_capability cap;			//视频设备信息
struct v4l2_format fmt;		//视频捕获格式
struct v4l2_requestbuffers reqbuf;//视频捕获分配内存
struct v4l2_fmtdesc fmtdesc;		//视频采集的参数
struct v4l2_buffer buf;
struct v4l2_streamparm setfps;  	//视频帧数

static int fd = -1;
//unsigned char frame_buffer[640*480*3];
//ImageBuffer imgbuf;

//1.open dev
int open_camera()
{
	fd = open(FILE_VIDEO, O_RDWR);
	if(fd < 0){
		printf("open failed!\n");
		return -1;
	}	
	printf("open device ok\n");
	return 0;
}
//2.query cap
void get_capability()
{
	int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
	if (ret < 0) {
		printf("VIDIOC_QUERYCAP failed (%d)\n", ret);
		return;
	}
	printf("driver:\t\t%s\n", cap.driver);
	printf("card:\t\t%s\n", cap.card);
	printf("bus_info:\t%s\n", cap.bus_info);
	printf("version:\t%d\n", cap.version);
	printf("capabilities:\t%x\n", cap.capabilities);
     	if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE) 
     	{
		printf("Device %s: supports capture.\n",FILE_VIDEO);
	}
	if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING) 
	{
		printf("Device %s: supports streaming.\n",FILE_VIDEO);
	}
}
//3.emu all support fmt
// 获取视频采集的参数
// VIDIOC_ENUM_FMT：获取当前驱动支持的视频格式 
void get_format()
{
	fmtdesc.index=0;
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;//设置帧类型为可以捕获的类型
	while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1)
	{
		printf("%d.%s\n",fmtdesc.index+1,fmtdesc.description);
		fmtdesc.index++;
	}
}

//4.set fmt
//设置视频采集的参数
int set_format()
{
	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = VIDEO_WIDTH;
	fmt.fmt.pix.height      = VIDEO_HEIGHT;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;//here is key code 1 ：chose from （V4L2_PIX_FMT_YUYV;V4L2_PIX_FMT_MJPEG）
	fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
	int ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
	if (ret < 0) {
		printf("VIDIOC_S_FMT failed (%d)\n", ret);
		return -1;
	}
	if(ioctl(fd,VIDIOC_G_FMT,&fmt) == -1){
		printf("unable to get format\n");
		return -1;
	}
    	//printf("------------VIDIOC_S_FMT---------------\n");
	printf("\nStream Format Informations:\n");
	printf("type: %d\n", fmt.type);
	
	printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF, (fmt.fmt.pix.pixelformat >> 8) & 0xFF,(fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
	
	printf("width: %d\n", fmt.fmt.pix.width);
	printf("height: %d\n", fmt.fmt.pix.height);
	printf("pix.field: \t\t%d\n", fmt.fmt.pix.field);
	return 0;
}
void set_fps()
{
	//set fps
	/*
		timeperframe字段用于指定想要使用的帧频率，它是一个结构体：?
		numerator?和denominator所描述的系数给出的是成功的帧之间的时间间隔。
		numerator分子，denominator分母。主要表达每次帧之间时间间隔。
		numerator/denominator秒一帧。
	*/
	setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  //每秒传输帧数
	setfps.parm.capture.timeperframe.numerator = 1;
	setfps.parm.capture.timeperframe.denominator = 5;
}
// VIDIOC_REQBUFS：分配内存 
void request_buf()
{
	reqbuf.count = BUFFER_COUNT;	// 2
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	//
	reqbuf.memory = V4L2_MEMORY_MMAP;			//

	int ret = ioctl(fd , VIDIOC_REQBUFS, &reqbuf);
	if(ret < 0) {
		printf("VIDIOC_REQBUFS failed (%d)\n", ret);
		return;
	}
	return;
}

// VIDIOC_QUERYBUF：把VIDIOC_REQBUFS中分配的数据缓存转换成物理地址 
// VIDIOC_QBUF：把数据从缓存中读取出来  /出队
void query_map_qbuf()
{
	int ret;

	unsigned int n_buffers;
	// buffers 全局指针
	buffers = (struct buffer *)calloc(reqbuf.count,sizeof(struct buffer));
   	if (NULL == buffers) {
      		printf ("malloc: Out of memory\n");
        	return;
   	}

	for (n_buffers = 0; n_buffers < reqbuf.count; n_buffers++)
	{
		struct v4l2_buffer buf;
		memset(&buf,0,sizeof(buf));

		buf.index = n_buffers;
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		//询问buffer状态
		//获取内核空间的视频缓冲区信息
		//Driver会填充v4l2_buffer 结构体内所有信息供用户使用。
		ret = ioctl(fd , VIDIOC_QUERYBUF, &buf);//buf取得内存缓冲区的信息
		if(ret < 0) {
			//printf("VIDIOC_QUERYBUF (%d) failed (%d)\n", i, ret);
			return;
		}
		//映射内核空间到用户空间 
		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = mmap(NULL, buf.length, PROT_READ | \
								PROT_WRITE, MAP_SHARED,\
								fd, buf.m.offset);
		if (MAP_FAILED == buffers[n_buffers].start) {
			printf("mmap failed\n");
			return;
		}
		//已经开始采集了
		//入队有效数据
		ret = ioctl(fd , VIDIOC_QBUF, &buf);
		if (ret < 0) {
			return;
		}
	}
}
void start_capture()
{
	int ret;
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;//设置采集方式

	//VIDIOC_STREAMON：开始视频显示函数 
	
	ret = ioctl(fd, VIDIOC_STREAMON, &type);//帧缓存入队
	if (ret < 0) {
		printf("VIDIOC_STREAMON failed (%d)\n", ret);
	}
	printf("----------------camera is working--------------------\n");
}
void camera_close()
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;//设置采集方式

	int j = 0;
	struct v4l2_buffer buf;
	//出队缓冲中所有
	for(;j < reqbuf.count;j++)
		ioctl(fd,VIDIOC_DQBUF,&buf);
	//VIDIOC_STREAMON：开始视频显示函数 
	int ret;
	ret = ioctl(fd, VIDIOC_STREAMOFF, &type);//帧缓存入队
	if (ret < 0) {
		printf("VIDIOC_STREAMON failed (%d)\n", ret);
	}
	printf("---camera is off--\n");

	for(j = 0; j < reqbuf.count;j++){
		ret = munmap(buffers[j].start,buffers[j].length);
		if(ret < 0){
			printf("munmap video buffers.\n");
			return;
		}
	}
	close(fd);
}

METHODDEF(void) init_destination(j_compress_ptr cinfo) {

    mjpg_dest_ptr dest =(mjpg_dest_ptr) cinfo->dest;
    dest->buffer =(JOCTET *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_IMAGE, OUTPUT_BUF_SIZE*sizeof(JOCTET));
    *(dest->written) = 0;
    dest->pub.next_output_byte = dest->buffer;
    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;

}

METHODDEF(boolean) empty_output_buffer(j_compress_ptr cinfo) {

    mjpg_dest_ptr dest =(mjpg_dest_ptr) cinfo->dest;
    memcpy(dest->outbuffer_cursor, dest->buffer, OUTPUT_BUF_SIZE);
    dest->outbuffer_cursor += OUTPUT_BUF_SIZE;
    *(dest->written) += OUTPUT_BUF_SIZE;
    dest->pub.next_output_byte = dest->buffer;
    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;
    return TRUE; 

}

METHODDEF(void) term_destination(j_compress_ptr cinfo) {

    mjpg_dest_ptr dest =(mjpg_dest_ptr) cinfo->dest;
    size_t datacount = OUTPUT_BUF_SIZE - dest->pub.free_in_buffer;
    memcpy(dest->outbuffer_cursor, dest->buffer, datacount);
    dest->outbuffer_cursor += datacount;
    *(dest->written) += datacount;

}


void dest_buffer(j_compress_ptr cinfo, unsigned char *buffer, int size, int *written) 
{
    mjpg_dest_ptr dest;
    if(cinfo->dest == NULL) {
        cinfo->dest =(struct jpeg_destination_mgr *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(mjpg_destination_mgr));

    }

    dest =(mjpg_dest_ptr)cinfo->dest;
    dest->pub.init_destination = init_destination;
    dest->pub.empty_output_buffer = empty_output_buffer;
    dest->pub.term_destination = term_destination;
    dest->outbuffer = buffer;
    dest->outbuffer_size = size;
    dest->outbuffer_cursor = buffer;
    dest->written = written;

}
void yuyv_2_rgb888(unsigned char *yuyv, LcdBuffer *rgb_buf)
{
	int i,j;
    	unsigned char y1,y2,u,v;
    	int r1,g1,b1,r2,g2,b2;
	unsigned char *pointer = yuyv;
	//pointer = buffers[0].start;
	
    	for(i=0;i<480;i++)
    	{
    		for(j=0;j<320;j++)
    		{
    			y1 = *( pointer + (i*320+j)*4);
    			u  = *( pointer + (i*320+j)*4 + 1);
    			y2 = *( pointer + (i*320+j)*4 + 2);
    			v  = *( pointer + (i*320+j)*4 + 3);
    		
    			r1 = y1 + 1.042*(v-128);
    			g1 = y1 - 0.34414*(u-128) - 0.71414*(v-128);
    			b1 = y1 + 1.772*(u-128);
    		
    			r2 = y2 + 1.042*(v-128);
    			g2 = y2 - 0.34414*(u-128) - 0.71414*(v-128);
    			b2 = y2 + 1.772*(u-128);
    		
    			if(r1>255)
    				r1 = 255;
    			else if(r1<0)
    				r1 = 0;
    		
    			if(b1>255)
    				b1 = 255;
    			else if(b1<0)
    				b1 = 0;	
   
    			if(g1>255)
    				g1 = 255;
    			else if(g1<0)
    				g1 = 0;	
    			
    			if(r2>255)
    				r2 = 255;
    			else if(r2<0)
    				r2 = 0;
    		
   	 		if(b2>255)
    				b2 = 255;
    			else if(b2<0)
    				b2 = 0;	
    		
    			if(g2>255)
    				g2 = 255;
    			else if(g2<0)
    				g2 = 0;		
		
	    		*(rgb_buf->start + ((480-1-i)*320+j)*6    ) = (unsigned char)b1;
    			*(rgb_buf->start + ((480-1-i)*320+j)*6 + 1) = (unsigned char)g1;
    			*(rgb_buf->start + ((480-1-i)*320+j)*6 + 2) = (unsigned char)r1;

    			*(rgb_buf->start + ((480-1-i)*320+j)*6 + 3) = (unsigned char)b2;
    			*(rgb_buf->start + ((480-1-i)*320+j)*6 + 4) = (unsigned char)g2;
    			*(rgb_buf->start + ((480-1-i)*320+j)*6 + 5) = (unsigned char)r2;
    	}
    }
    //printf("change to RGB OK \n");
}
/**********************************************************/
//函数名：yuv_jpg
//功能  ：YUV格式到JPG格式
//形参  ：quality(0~100之间的整数，表示压缩比率)
//返回值：SUCCESS/FAIURE
/***********************************************************/
status yuv_jpg(unsigned char *buf, unsigned char *buffer, int size, int quality) 
{

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
	
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);
	
	
    JSAMPROW row_pointer[1];
    unsigned char *line_buffer, *yuyv;
    int z;
    static int written;
	
    line_buffer = calloc(VIDEO_WIDTH * 3, 1);
    yuyv = buf;
		
    dest_buffer(&cinfo, buffer, size, &written);
	
    cinfo.image_width = VIDEO_WIDTH;
    cinfo.image_height = VIDEO_HEIGHT;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
	
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    z = 0;

    while(cinfo.next_scanline < VIDEO_HEIGHT) {
        int x;
        unsigned char *ptr = line_buffer;
		
        for(x = 0; x < VIDEO_WIDTH; x++) {
            int r, g, b;
            int y, u, v;
            if(!z)
            y = yuyv[0] << 8;
            else
            y = yuyv[2] << 8;
			
            u = yuyv[1] - 128;
            v = yuyv[3] - 128;
			
            r =(y +(359 * v)) >> 8;
            g =(y -(88 * u) -(183 * v)) >> 8;
            b =(y +(454 * u)) >> 8;
			
            *(ptr++) =(r > 255) ? 255 :((r < 0) ? 0 : r);
            *(ptr++) =(g > 255) ? 255 :((g < 0) ? 0 : g);
            *(ptr++) =(b > 255) ? 255 :((b < 0) ? 0 : b); 
            if(z++) {
                z = 0;
                yuyv += 4;
            }

        }

        row_pointer[0] = line_buffer;
        jpeg_write_scanlines(&cinfo, row_pointer, 1);

    }
	
	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
	free(line_buffer); 
	
	return(written);
}

/**********************************************************/
//函数名：get_camera_jpg
//功能  ：从缓冲区获取图像
//形参  ：fd、jpg_buf
//返回值：SUCCESS/FAIURE
/***********************************************************/
status get_camera_jpg(int fd, ImageBuffer *jpg_buf,LcdBuffer *rgb_buf)
{
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (-1 == ioctl(fd, VIDIOC_DQBUF, &buf))
    {}

	sem_wait(&sem_ca);
    yuyv_2_rgb888(buffers[buf.index].start,rgb_buf);
    sem_post(&sem_lcd);
    
	pthread_mutex_lock(&mutex_web);
    jpg_buf->length = yuv_jpg(buffers[buf.index].start, jpg_buf->start, \
		(VIDEO_WIDTH * VIDEO_HEIGHT), 25);
    pthread_mutex_unlock(&mutex_web);

	ioctl(fd, VIDIOC_QBUF, &buf); 

    return SUCCESS;

}
status camera_init(void)
{
	open_camera();	//打开设备

	get_capability();//查询信息

	get_format();	//查询支持格式

	set_format();	//设置视频格式 capture 和帧格式 yuyv 480 640
	set_fps();	//设置每秒传输的帧数
	request_buf();//请求V4L2驱动分配视频缓冲区
	query_map_qbuf();//建立映射
	
	start_capture();	//开始采集
	
	return fd;
}


