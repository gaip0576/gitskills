/*******************************************************************************
*  COPYRIGHT SGITG Co
********************************************************************************
* 源文件名:           sgitg_gps.c 
* 功能:                  
* 版本:                                                                  
* 编制日期:   20201124                           
* 作者:       孙孝波                                       
*******************************************************************************/
/************************** 包含文件声明 **********************************/
/**************************** 共用头文件* **********************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>  
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/time.h>
#include <pthread.h>
#include <sys/syscall.h>
//#include <include/linux/delay.h>
#include <unistd.h>
/**************************** 私用头文件* **********************************/
#include "bsp_types.h"
#include "bsp_gps.h"

/******************************* 局部宏定义 *********************************/
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <sys/time.h>


#define CSGITG_DEBUG 1
#define BSP_DATARSP_BUF_LEN         (1500)

extern s32 ubx_com_utc_time(void);
extern s32 ubx_get_utc_time(void);
extern s32 config_sys_time(void);

pthread_mutex_t g_mp_gps = PTHREAD_MUTEX_INITIALIZER;  
T_GpsAllData gUbloxGpsAllData;
u32 g_IntGpsUartFd;
#define GPS_DELAY(a)         {vu32 _i_;  for (_i_ = 0; _i_ < 4 * (a); _i_++) {__asm(" nop");}}

#define SGITG_GPIO 1
#if SGITG_GPIO
#define GPIO_DEV    "/dev/gpio"

#define DEV_NAME0    "/dev/ttyUSB0"    ///< 串口设备                                                                   
#define DEV_NAME1    "/dev/ttyUSB1" 
#define DEV_NAME2    "/dev/ttyUSB2" 

#define RALINK_GPIO_SET_DIR_IN                0x11
#define RALINK_GPIO_SET_DIR_OUT                0x12

#define RALINK_GPIO_ENABLE_INTP                0x08
#define RALINK_GPIO_DISABLE_INTP        0x09

#define RALINK_GPIO_REG_IRQ             0x0A
#define SGITG_GPS_TIME_SET              0xA1
#define SGITG_GPS_GAIP_CLR		        0xA2
#define SGITG_GPS_GAIP_READ		        0xA3
#define MAX_DELAY_STAT      	        200
#define sunxb_debug  0
/*
 * structure used at regsitration
 */
#define RALINK_GAIP_CONT        100
#define RALINK_GAIP_MAX_STR     1024

typedef struct {
	int seq;
	int one_lick_cnt;
	struct timeval tv;
	char buffer_gaip[RALINK_GAIP_CONT][RALINK_GAIP_MAX_STR];
} ralink_gaip_info;

typedef struct {
    int interval;
    int interval_kernel;
    int time_adjust_sign;
	struct timeval user_tv;
	struct timeval kernel_tv;
	struct timeval user_tv_old;
	struct timeval kernel_tv_old;
} pp1s_delay_stat;

typedef struct {
        unsigned int irq;                //request irq pin number
        pid_t pid;                        //process id to notify
} ralink_gpio_reg_info;

enum
{
    gpio_in,
    gpio_out,
};
enum
{
#if defined (CONFIG_RALINK_MT7620)
    gpio2300,
    gpio3924,
    gpio7140,
    gpio72,
#elif defined (CONFIG_RALINK_MT7621)
    gpio3100,
    gpio6332,
    gpio9564,
#elif defined (CONFIG_RALINK_MT7628)
    gpio3100,
    gpio6332,
    gpio9564,
#else
    gpio2300,
#endif
};


char gu8timeCurrent[20];                                                                                                                     
volatile int gu32timeokflag=0;
int gpio_set_dir(int r, int dir)
{
    int fd, req;

    fd = open(GPIO_DEV, O_RDONLY);
    if (fd < 0)
    {
        perror(GPIO_DEV);
        return -1;
    }
    if (dir == gpio_in)
    {

#if defined (CONFIG_RALINK_MT7620)
        if (r == gpio72)
            req = RALINK_GPIO72_SET_DIR_IN;
        else if (r == gpio7140)
            req = RALINK_GPIO7140_SET_DIR_IN;
        else if (r == gpio3924)
            req = RALINK_GPIO3924_SET_DIR_IN;
        else
#elif defined (CONFIG_RALINK_MT7621)
        if (r == gpio9564)
            req = RALINK_GPIO9564_SET_DIR_IN;
        else if (r == gpio6332)
            req = RALINK_GPIO6332_SET_DIR_IN;
        else
#elif defined (CONFIG_RALINK_MT7628)
        if (r == gpio9564)
            req = RALINK_GPIO9564_SET_DIR_IN;
        else if (r == gpio6332)
            req = RALINK_GPIO6332_SET_DIR_IN;
        else
#endif
            req = RALINK_GPIO_SET_DIR_IN;
    }
    else
    {

#if defined (CONFIG_RALINK_MT7620)
        if (r == gpio72)
            req = RALINK_GPIO72_SET_DIR_OUT;
        else if (r == gpio7140)
            req = RALINK_GPIO7140_SET_DIR_OUT;
        else if (r == gpio3924)
            req = RALINK_GPIO3924_SET_DIR_OUT;
        else
#elif defined (CONFIG_RALINK_MT7621)
        if (r == gpio9564)
            req = RALINK_GPIO9564_SET_DIR_OUT;
        else if (r == gpio6332)
            req = RALINK_GPIO6332_SET_DIR_OUT;
        else
#elif defined (CONFIG_RALINK_MT7628)
        if (r == gpio9564)
            req = RALINK_GPIO9564_SET_DIR_OUT;
        else if (r == gpio6332)
            req = RALINK_GPIO6332_SET_DIR_OUT;
        else
#endif
            req = RALINK_GPIO_SET_DIR_OUT;
    }
    if (ioctl(fd, req, 0xffffffff) < 0)
    {
        perror("ioctl");
        close(fd);
        return -1;
    }
    close(fd);
#if CSGITG_DEBUG
    printf(" === %s, === %d === req:%d, gaip\n",__FUNCTION__,__LINE__,req);
#endif    
    return 0;
}


int gpio_enb_irq(void)
{
    int fd;

#if CSGITG_DEBUG
    printf(" === %s, === %d === req:%d, gaip\n",__FUNCTION__,__LINE__,RALINK_GPIO_ENABLE_INTP);
#endif
    fd = open(GPIO_DEV, O_RDONLY);
    if (fd < 0)
    {
        perror(GPIO_DEV);
        return -1;
    }
    if (ioctl(fd, RALINK_GPIO_ENABLE_INTP) < 0)
    {
        perror("ioctl");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

int gpio_dis_irq(void)
{
    int fd;

#if CSGITG_DEBUG
    printf(" === %s, === %d === req:%d, gaip\n",__FUNCTION__,__LINE__,RALINK_GPIO_DISABLE_INTP);
#endif
    fd = open(GPIO_DEV, O_RDONLY);
    if (fd < 0)
    {
        perror(GPIO_DEV);
        return -1;
    }
    if (ioctl(fd, RALINK_GPIO_DISABLE_INTP) < 0)
    {
        perror("ioctl");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

int gpio_reg_info(int gpio_num)
{
    int fd;
    ralink_gpio_reg_info info;

#if CSGITG_DEBUG
    printf(" === %s, === %d === req:%d, gaip\n",__FUNCTION__,__LINE__,RALINK_GPIO_REG_IRQ);
#endif
    fd = open(GPIO_DEV, O_RDONLY);
    if (fd < 0)
    {
        perror(GPIO_DEV);
        return -1;
    }
    info.pid = getpid();
    info.irq = gpio_num;
    if (ioctl(fd, RALINK_GPIO_REG_IRQ, &info) < 0)
    {
        perror("ioctl");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}
#define STAT_CNT 6
int u32intcnt=0;
int accCnt=0;
struct timespec gaip_clock_gettime;
struct timeval gaip_gettimeofday;
ralink_gaip_info buf_out;
int loss_pp1s_int = 0;
int interval_pp1s_err = 0;
int time_adjust_sec;
int read_kernel_err = 0;

pp1s_delay_stat gaip_stat[MAX_DELAY_STAT];
#define MAX_DIFF 200

void signal_handler_usr1(int signum)
{
    int i;
    printf("kernel-do_gettimeofday == user-gettimeofday  compare read_kernel_err %d\n",read_kernel_err);
    for(i=0;i<MAX_DELAY_STAT;i++)
    {
        printf("[%03d] kernel %d:%06d -- user %d:%06d -- user_interval :%04dus -- kernel_interval %04dus\n",i,
            gaip_stat[i].kernel_tv.tv_sec,gaip_stat[i].kernel_tv.tv_usec,
            gaip_stat[i].user_tv.tv_sec,gaip_stat[i].user_tv.tv_usec,
            gaip_stat[i].interval,gaip_stat[i].interval_kernel);
        printf("[%03d] old_kernel %d:%06d -- old_user %d:%06d -- timeadjust_sign: %d\n",i,
            gaip_stat[i].kernel_tv_old.tv_sec,gaip_stat[i].kernel_tv_old.tv_usec,
            gaip_stat[i].user_tv_old.tv_sec,gaip_stat[i].user_tv_old.tv_usec,
            gaip_stat[i].time_adjust_sign);
    }
}

void signal_handler(int signum)
{
    int i,fd,req,ms_old,ms_cur,ms_old_kernel,ms_cur_kernel;
    ralink_gaip_info *pBufOutPut=NULL;
    struct timeval gaip_gettimeofday_old,do_gettimeofday_old;

   	clock_gettime(CLOCK_MONOTONIC, &gaip_clock_gettime);
    gaip_gettimeofday_old=gaip_gettimeofday;
    ms_old = (gaip_gettimeofday.tv_sec%10)*1000000+gaip_gettimeofday.tv_usec;
	gettimeofday(&gaip_gettimeofday,NULL);
    if((0==gaip_gettimeofday.tv_sec%10)&&(gaip_gettimeofday_old.tv_sec))
        ms_cur = (gaip_gettimeofday.tv_sec%10+10)*1000000+gaip_gettimeofday.tv_usec;
    else
        ms_cur = (gaip_gettimeofday.tv_sec%10)*1000000+gaip_gettimeofday.tv_usec;

    u32intcnt++;
#if CSGITG_DEBUG
    if((gaip_gettimeofday.tv_sec-gaip_gettimeofday_old.tv_sec>10)||
       (gaip_gettimeofday_old.tv_sec-gaip_gettimeofday.tv_sec>10)||
       (ms_cur-ms_old>1500000))
    {
        loss_pp1s_int++;
        printf("loss gps pp1s int : %d, ms_cur: %dms, ms_old: %dms\n",loss_pp1s_int,ms_cur,ms_old);
    }
    printf(" * clock_gettime tv_sec:tv_usec: %d : %d : gettimeofday tv_sec:tv_usec: %d : %d\r\n",
        (unsigned int)gaip_clock_gettime.tv_sec,(unsigned int)gaip_clock_gettime.tv_nsec/1000,
        (unsigned int)gaip_gettimeofday.tv_sec,(unsigned int)gaip_gettimeofday.tv_usec);
#endif 

    fd = open(GPIO_DEV, O_RDONLY);
    if (fd < 0)
    {
        read_kernel_err++;
        perror(GPIO_DEV);
        return;
    }

    req = SGITG_GPS_GAIP_READ;
    pBufOutPut=&buf_out;
    pBufOutPut->seq = 0;
    do_gettimeofday_old=pBufOutPut->tv;
    ms_old_kernel = (pBufOutPut->tv.tv_sec%10)*1000000+pBufOutPut->tv.tv_usec;
    if (ioctl(fd, req, (char*)pBufOutPut) < 0)
    {
        perror("ioctl");
        close(fd);
        read_kernel_err++;
        return;
    }
    if((0==pBufOutPut->tv.tv_sec%10)&&(do_gettimeofday_old.tv_sec))
        ms_cur_kernel = (pBufOutPut->tv.tv_sec%10+10)*1000000+pBufOutPut->tv.tv_usec;
    else
        ms_cur_kernel = (pBufOutPut->tv.tv_sec%10)*1000000+pBufOutPut->tv.tv_usec;
    //大于10s
    if(ms_cur-ms_old>10000000) ms_cur-=10000000;
    if(ms_cur_kernel-ms_old_kernel>10000000) ms_cur_kernel-=10000000;
    //小于-10s
    if(ms_cur-ms_old<-8500000) ms_cur+=10000000;
    if(ms_cur_kernel-ms_old_kernel<-8500000) ms_cur_kernel+=10000000;

    if(ms_old&&((ms_cur-ms_old>1000000+MAX_DIFF)||(ms_cur-ms_old<1000000-MAX_DIFF)||
                (ms_cur_kernel-ms_old_kernel>1000000+MAX_DIFF)||(ms_cur_kernel-ms_old_kernel<1000000-MAX_DIFF)))
    {
        interval_pp1s_err++;
        gaip_stat[interval_pp1s_err%MAX_DELAY_STAT].interval = ms_cur-ms_old-1000000;
        gaip_stat[interval_pp1s_err%MAX_DELAY_STAT].interval_kernel = ms_cur_kernel-ms_old_kernel-1000000;
        gaip_stat[interval_pp1s_err%MAX_DELAY_STAT].kernel_tv = pBufOutPut->tv;
        gaip_stat[interval_pp1s_err%MAX_DELAY_STAT].user_tv = gaip_gettimeofday;
        gaip_stat[interval_pp1s_err%MAX_DELAY_STAT].kernel_tv_old = do_gettimeofday_old;
        gaip_stat[interval_pp1s_err%MAX_DELAY_STAT].user_tv_old = gaip_gettimeofday_old;
        if((gaip_gettimeofday.tv_sec < time_adjust_sec+6)&&(gaip_gettimeofday.tv_sec > time_adjust_sec+1))
            gaip_stat[interval_pp1s_err%MAX_DELAY_STAT].time_adjust_sign = 1;
        else
            gaip_stat[interval_pp1s_err%MAX_DELAY_STAT].time_adjust_sign = 0;
    }
    if(pBufOutPut->seq == 0) 
    {
        printf(" *** pBufOutPut is null\n");
    }
    else
    {
#if CSGITG_DEBUG
        printf(" *** pBufOutPut seq is %d, one click cnt :%d, loss pp1s: %d, pp1s_err: %d, do_gettimeofday %d:%d\n",
            pBufOutPut->seq,pBufOutPut->one_lick_cnt,loss_pp1s_int,interval_pp1s_err,
            pBufOutPut->tv.tv_sec,pBufOutPut->tv.tv_usec);
        if(pBufOutPut->seq > RALINK_GAIP_CONT) pBufOutPut->seq = RALINK_GAIP_CONT;
        for(i=0;i<pBufOutPut->seq;i++){
//            printf("%s",pBufOutPut->buffer_gaip[i]);
        }
#endif
    }
    req = SGITG_GPS_GAIP_CLR;
    if (ioctl(fd, req, 0x00) < 0)
    {
        perror("ioctl");
        close(fd);
        return;
    }
    close(fd);

    if(0 == gu32timeokflag) 
    {
        //time_set(gu8timeCurrent);
        gu32timeokflag = 1;
    }
    else if(1 == gu32timeokflag) 
    {
        //time_set(gu8timeCurrent);
        ubx_com_utc_time();
        gu32timeokflag = 2;
    }
    else if (gu32timeokflag ==2)
    {
        //time_get();
        if (GPS_OK ==ubx_get_utc_time()) 
        {
        //ubx_get_utc_time();
            gu32timeokflag = 3;
        }
        else
        {
            gu32timeokflag = 1;
        }
    }
    else if (gu32timeokflag ==3)
    {
        config_sys_time(); 
        gu32timeokflag=4;  
        //获取校准的时间
        time_adjust_sec = gaip_gettimeofday_old.tv_sec;
        printf(" +++++++++++++++++ time_adjust_sec %d +++++++++++++ line: %d\n",time_adjust_sec,__LINE__);
    }
    else if (gu32timeokflag ==4)
    {
        gu32timeokflag=5;
    }
    else
    {
        gu32timeokflag=5;
//        printf("gps state: lock. config time is ok! line:%d\n",__LINE__);
    }
}


void gpio_test_intr(int gpio_num)
{
    //set gpio direction to input

#if defined (CONFIG_RALINK_MT7620)
    gpio_set_dir(gpio72, gpio_in);
    gpio_set_dir(gpio7140, gpio_in);
    gpio_set_dir(gpio3924, gpio_in);
    gpio_set_dir(gpio2300, gpio_in);
#elif defined (CONFIG_RALINK_MT7621)
    gpio_set_dir(gpio9564, gpio_in);
    gpio_set_dir(gpio6332, gpio_in);
    gpio_set_dir(gpio3100, gpio_in);
#elif defined (CONFIG_RALINK_MT7628)
    gpio_set_dir(gpio9564, gpio_in);
    gpio_set_dir(gpio6332, gpio_in);
    gpio_set_dir(gpio3100, gpio_in);
#else
    gpio_set_dir(gpio2300, gpio_in);
#endif

    //enable gpio interrupt
    gpio_enb_irq();

    //register my information
    gpio_reg_info(gpio_num);

    //issue a handler to handle SIGUSR1
    signal(SIGUSR1, signal_handler_usr1);
    signal(SIGUSR2, signal_handler);
#if 0
    //wait for signal
    pause();

    //disable gpio interrupt
    gpio_dis_irq();
#endif
}
#endif
/*******************************************************************************
* 函数名称: bsp_gps_mutex_init                        
* 功    能:                                     
* 相关文档:                    
* 函数类型:    s32                                
* 参    数:                                          
* 参数名称           类型                    输入/输出         描述        

* 返回值:     0 正常
* 说   明:
*******************************************************************************/
s32 bsp_gps_mutex_init(void)
{
    s32 s32ret = 0;
    s32ret |=pthread_mutex_init(&g_mp_gps, NULL);
    return s32ret;
}

/*******************************************************************************
* 函数名称: gps_uart_set
* 功    能: 
* 相关文档:
* 函数类型:
* 参    数:
* 参数名称          类型        输入/输出       描述
*******************************************************************************/
s32 gps_uart_set(void)
{
       struct termios Opt;
    
     //system("mknod /dev/ttyS1 c 4 65");

    g_IntGpsUartFd = open( "/dev/ttyS0", O_RDWR |O_NOCTTY | O_NDELAY);
    if (ERROR == g_IntGpsUartFd)
    {
        bsp_dbg( "Open  g_IntGpsUartFd failed!g_IntGpsUartFd = 0x%x\n",g_IntGpsUartFd);
        return GPS_ERROR;
    }

    if(tcgetattr(g_IntGpsUartFd, &Opt) != 0)
    {
        perror("tcgetattr fd");
        return GPS_ERROR;
    }

    Opt.c_cflag &= ~CSIZE;
    Opt.c_cflag |= CS8;

    Opt.c_cflag |= (CLOCAL | CREAD);

    Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    Opt.c_oflag &= ~OPOST;
    Opt.c_oflag &= ~(ONLCR | OCRNL); 

    Opt.c_iflag &= ~(ICRNL | INLCR);
    Opt.c_iflag &= ~(IXON | IXOFF | IXANY); 

    tcflush(g_IntGpsUartFd, TCIFLUSH);
    Opt.c_cc[VTIME] = 0; 
    Opt.c_cc[VMIN] = 0; 

    if(tcsetattr(g_IntGpsUartFd, TCSANOW, &Opt) != 0)
    {
        perror("tcsetattr g_IntGpsUartFd");
        return GPS_ERROR;
    }
    
    return GPS_OK;
    
}

/*******************************************************************************
* 函数名称: ubx_close_nmea
* 功    能:设置UBX CFG-MSG   关闭NEMA输出
* 相关文档:
* 函数类型: extern
* 参    数:
* 参数名称          类型        输入/输出       描述
* 返回值:
* BSP_OK:成功
* 说   明:
*******************************************************************************/
s32 ubx_close_nmea(void)
{
    u8 u8WriteLen;
    u8 close_nmea_gga[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0, 0xF0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0x23};
    u8 close_nmea_gll[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0, 0xF0, 1, 0, 0, 0, 0, 0, 0, 0, 0x2A};
    u8 close_nmea_gsa[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0, 0xF0, 2, 0, 0, 0, 0, 0, 0, 1, 0x31};
    u8 close_nmea_gsv[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0, 0xF0, 3, 0, 0, 0, 0, 0, 0, 2, 0x38};
    u8 close_nmea_rmc[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0, 0xF0, 4, 0, 0, 0, 0, 0, 0, 3, 0x3F};
    u8 close_nmea_vtg[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0, 0xF0, 5, 0, 0, 0, 0, 0, 0, 4, 0x46};
    u8 close_nmea_zda[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0, 0xF0, 8, 0, 0, 0, 0, 0, 0, 7, 0x5B};
    
    pthread_mutex_lock(&g_mp_gps);
    u8WriteLen = (u8)write(g_IntGpsUartFd, (char *)close_nmea_gga, 16);
    if (16 != u8WriteLen)
    {
        bsp_dbg("write error!%s,%d\n",__FUNCTION__,__LINE__);
        pthread_mutex_unlock(&g_mp_gps);
        return GPS_ERROR;
    }
    
      GPS_DELAY(10);

    u8WriteLen = (u8)write(g_IntGpsUartFd, (char *)close_nmea_gll, 16);
    if (16 != u8WriteLen)
    {
        bsp_dbg("write error!%s,%d\n",__FUNCTION__,__LINE__);
           pthread_mutex_unlock(&g_mp_gps);
        return GPS_ERROR;
    }
      GPS_DELAY(10);

    u8WriteLen = (u8)write(g_IntGpsUartFd, (char *)close_nmea_gsa, 16);
    if (16 != u8WriteLen)
    {
        bsp_dbg("write error!%s,%d\n",__FUNCTION__,__LINE__);
           pthread_mutex_unlock(&g_mp_gps);
        return GPS_ERROR;
    }
      GPS_DELAY(10);

    u8WriteLen = (u8)write(g_IntGpsUartFd, (char *)close_nmea_gsv, 16);
    if (16 != u8WriteLen)
    {
        bsp_dbg("write error!%s,%d\n",__FUNCTION__,__LINE__);
           pthread_mutex_unlock(&g_mp_gps);
        return GPS_ERROR;
    }
      GPS_DELAY(10);

    u8WriteLen = (u8)write(g_IntGpsUartFd, (char *)close_nmea_rmc, 16);
    if (16 != u8WriteLen)
    {
        bsp_dbg("write error!%s,%d\n",__FUNCTION__,__LINE__);
           pthread_mutex_unlock(&g_mp_gps);
        return GPS_ERROR;
    }
      GPS_DELAY(10);

    u8WriteLen = (u8)write(g_IntGpsUartFd, (char *)close_nmea_vtg, 16);
    if (16 != u8WriteLen)
    {
        bsp_dbg("write error!%s,%d\n",__FUNCTION__,__LINE__);
           pthread_mutex_unlock(&g_mp_gps);
        return GPS_ERROR;
    }
    GPS_DELAY(10);

    u8WriteLen = (u8)write(g_IntGpsUartFd, (char *)close_nmea_zda, 16);
    if (16 != u8WriteLen)
    {
        bsp_dbg("write error!%s,%d\n",__FUNCTION__,__LINE__);
           pthread_mutex_unlock(&g_mp_gps);
        return GPS_ERROR;
    }
    pthread_mutex_unlock(&g_mp_gps);

    GPS_DELAY(10);  
    return GPS_OK;
}




/*******************************************************************************
* 函数名称: ubx_com_utc_time
* 功    能: 下发获得utc时间的命令
* 相关文档:
* 函数类型: extern
* 参    数:
* 参数名称          类型        输入/输出       描述
* 返回值:
* BSP_OK:成功
* 说   明:
*******************************************************************************/
s32 ubx_com_utc_time(void)
{
    
    u8 read_utc_com[8] = {0xB5, 0x62, 0x01, 0x21, 0, 0, 0x22, 0x67};
    u8 u8WriteLen;
    pthread_mutex_lock(&g_mp_gps);
    u8WriteLen = (u8)write(g_IntGpsUartFd, (char *)read_utc_com, 8);
    if (8 != u8WriteLen)
    {
        bsp_dbg("write error!%s,%d\n",__FUNCTION__,__LINE__);
        pthread_mutex_unlock(&g_mp_gps);
        return GPS_ERROR;
    }
    pthread_mutex_unlock(&g_mp_gps);
    return GPS_OK;
}
    

/*******************************************************************************
* 函数名称: ubx_get_utc_time
* 功    能: 读取utc时间
* 相关文档:
* 函数类型: extern
* 参    数:
* 参数名称          类型        输入/输出       描述
* 返回值:
* BSP_OK:成功
* 说   明:
*******************************************************************************/
s32 ubx_get_utc_time(void)
{
    unsigned char response[300];
    int count,i;
    //count = TimedRead (g_IntGpsUartFd, &response[0], 28, 1);
    count = read(g_IntGpsUartFd, &response[0], 300);
    if(28<=count)
    {
        
        for (i=0;i<(count-27);i++)
        {
            if(0xB5==response[i]&&0x62==response[i+1]&&0x1==response[i+2]&&0x21==response[i+3])
                break;
        }
    
    }
    else
    {
#if CSGITG_DEBUG
    printf("read count %d, line:%d\n",count,__LINE__);
    
#endif 
        
        return -1;
    }
    if (i==(count-27))
    {
        return -1;
    }
#if CSGITG_DEBUG
    printf("read count %d, line:%d\n",count,__LINE__);
    printf("start index is %d, line:%d\n",i,__LINE__);
#endif    
                //解析时间，加时区
    unsigned char *pchar = &response[i+6];//数据区
        
    if(pchar[19]&0x07 == 0x07)//utc valid
    {
        //pthread_mutex_lock(&(gUbloxGpsAllData.m_mutex));
        gUbloxGpsAllData.Year = pchar[13]*0x100+pchar[12];//小端模式
        gUbloxGpsAllData.Month = pchar[14];
        gUbloxGpsAllData.Day = pchar[15];
        gUbloxGpsAllData.Hour = pchar[16];
        gUbloxGpsAllData.Minute = pchar[17];
        gUbloxGpsAllData.Second = pchar[18];
#if CSGITG_DEBUG
        printf("year=%d\n",gUbloxGpsAllData.Year);
        printf("Month=%d\n",gUbloxGpsAllData.Month);
        printf("Day=%d\n",gUbloxGpsAllData.Day);
        printf("Hour=%d\n",gUbloxGpsAllData.Hour);
        printf("Minute=%d\n",gUbloxGpsAllData.Minute);
        printf("Second=%d\n",gUbloxGpsAllData.Second);
#endif            
    }
    else
    {
        return -2;
#if CSGITG_DEBUG
        printf("read utc time is err,read count is %d!\n",count);
#endif
    }
    return GPS_OK;
}

/*******************************************************************************
* 函数名称: ubx_get_utc_time
* 功    能: 读取utc时间
* 相关文档:
* 函数类型: extern
* 参    数:
* 参数名称          类型        输入/输出       描述
* 返回值:
* BSP_OK:成功
* 说   明:
*******************************************************************************/
s32 ubx_read_empty(void)
{
    unsigned char response[300];
    int count=1,i;
    while(count)
    {
        //count = TimedRead (g_IntGpsUartFd, &response[0], 300, 1);
        usleep(1000*1000);
        count = read(g_IntGpsUartFd, &response[0], 300);
/*
        printf("\nread empty count %d, gps[]",count);
        for(i=0;i<count;i++) {
            if(i%8==0){
                if(i%16==0) printf("\n");
                else printf(" *");
            }
            printf(" 0x%02x",i,response[i]);
        }       
*/
    }
    return;
    
}
/*******************************************************************************
* 函数名称: sgitg_gps_time_set
* 功    能: 将用户态获取的时间信息传递到内核态，系统时间实际在内核态进行配置
* 相关文档:
* 函数类型: extern
* 参    数:
* 参数名称          类型        输入/输出       描述
* 返回值:
* BSP_OK:成功
* 说   明:
*******************************************************************************/

int sgitg_gps_time_set( time_t value)
{
    int fd, req;
    unsigned long p=&value;
    struct timespec ts;

    fd = open(GPIO_DEV, O_RDONLY);
    if (fd < 0)
    {
        perror(GPIO_DEV);
        return -1;
    }

    req = SGITG_GPS_TIME_SET;
    if (ioctl(fd, req, p) < 0)
    {
        perror("ioctl");
        close(fd);
        return -1;
    }
    close(fd);

#if CSGITG_DEBUG
	clock_gettime(CLOCK_MONOTONIC, &ts);
    printf(" === %s, === %d === req:%d, gaip\n",__FUNCTION__,__LINE__,req);
	printf("----func:%s, second:%d line:%d ----\r\n",__FUNCTION__,ts.tv_sec,__LINE__);
#endif
    return 0;
}
/*******************************************************************************
* 函数名称: config_sys_time
* 功    能: 读取utc时间
* 相关文档:
* 函数类型: extern
* 参    数:
* 参数名称          类型        输入/输出       描述
* 返回值:
* BSP_OK:成功
* 说   明:
*******************************************************************************/
#if 0
s32 config_sys_time(void)
{
    u32 gpsOffset=480;
    struct tm mytime;
    time_t  t_tick;
    struct timeval stime;
    memset(&mytime, 0, sizeof(mytime));
    mytime.tm_year = gUbloxGpsAllData.Year - 1900;//from 1900
    mytime.tm_mon  = gUbloxGpsAllData.Month - 1;//0-11
    mytime.tm_mday = gUbloxGpsAllData.Day;
    mytime.tm_hour = gUbloxGpsAllData.Hour ;
    mytime.tm_min  = gUbloxGpsAllData.Minute;
    mytime.tm_sec  = gUbloxGpsAllData.Second;                   
    t_tick = mktime(&mytime) +gpsOffset*60+1;  //sunxiaobo add 1   ,
    stime.tv_sec = t_tick;
    stime.tv_usec = 0;
    if (settimeofday(&stime,NULL)<0)
    
    {
#if CSGITG_DEBUG
        printf("error\n");
#endif
    }
    else
    {
#if CSGITG_DEBUG
        printf("OK!\n");
#endif

    }
}
#else
s32 config_sys_time(void)
{
    u32 gpsOffset=480;
    struct tm mytime;
    time_t  t_tick;
    struct timeval stime;
    memset(&mytime, 0, sizeof(mytime));
    mytime.tm_year = gUbloxGpsAllData.Year - 1900;//from 1900
    mytime.tm_mon  = gUbloxGpsAllData.Month - 1;//0-11
    mytime.tm_mday = gUbloxGpsAllData.Day;
    mytime.tm_hour = gUbloxGpsAllData.Hour ;
    mytime.tm_min  = gUbloxGpsAllData.Minute;
    mytime.tm_sec  = gUbloxGpsAllData.Second;                   
    t_tick = mktime(&mytime) +gpsOffset*60+3;  //sunxiaobo add 1   ,
    sgitg_gps_time_set(t_tick);
    
}
#endif

int sgitg_gps(void)
{
    s32 s32ret=GPS_OK;
    
    char cmd[64] = {0};
    char driver[32]= {0};
    char devnum[32]= {0};
    FILE * fp = NULL;
    int i=0;
    
    printf("**********************************\n");
    printf("**********************************\n");
    printf("*** 2020-02-23 13:49         *****\n");
    printf("* sgitg_gps V1.8 start line: %d\n",__LINE__);
    printf("* gu32timeokflag=0: recv gps int 1\n");
    printf("* gu32timeokflag=1: write utc time inscturction\n");
    printf("* gu32timeokflag=2: read utc time from uart1\n");
    printf("* gu32timeokflag=3: transfer time infomation to kernel mode\n");
    printf("* gu32timeokflag=4: set system time in kernel mode\n");
    printf("* gu32timeokflag=5: set ok or timeout\n");
    printf("**********************************\n");
    printf("**********************************\n");
    fp = popen("cat /proc/devices | grep gpio | sed  -r -n \"s/[0-9]+ //p\"", "r");
    fscanf(fp, "%s", driver);
    pclose(fp);

    fp = popen("cat /proc/devices | grep gpio | sed  -r -n \"s/ [a-z0-9]*//p\"", "r");
    fscanf(fp, "%s", devnum);
    pclose(fp);

    printf(" *** %s-%s",driver,devnum);
    if (strlen(driver) && strlen(devnum))
    {
        snprintf(cmd, sizeof(cmd), "[ -c /dev/gpio ] || mknod /dev/gpio c %d 0", atoi(devnum));
        system(cmd);
        printf("gpio module enabled!\n");
    }
    else
    {
        printf("gpio module not enabled!\n");
    }

    //bsp_gps_init();
    gps_uart_set();
        
    system("date");
    printf("  line: %d\n",__LINE__);
    s32ret = ubx_close_nmea();
    if(s32ret != GPS_OK)
    {
        bsp_dbg("ubx_close_nmea error!\n");
        return GPS_ERROR;
    }
    //usleep(1000*100);
    ubx_read_empty();
    printf("close nmea is ok!\n");
    //gps_NAV_TimeUTC(480);
    //ubx_com_utc_time();
    usleep(1000*1000);
    system("date");
    printf("  line: %d\n",__LINE__);

    gpio_test_intr(0);
    //ubx_com_utc_time();
    //printf("command utc is ok!\n");
    //system("date");
    //usleep(20000*1000);
    //usleep(1000*1000);
    //GPS_DELAY(100000000);
    
#if 0    
    if (GPS_OK ==ubx_get_utc_time())                                  
    {
    //system("date");
    //printf("get utc is ok!\n");
        config_sys_time();
        printf("set systime is ok!\n");
    }
#endif  
    while(1)
    {
        if (5 == gu32timeokflag)
        {
//            gpio_dis_irq();
            printf("gps state: lock. config time is ok! line:%d\n",__LINE__);
            break;
        } 
        sleep(1);
        i++;
        if (i>10){
            printf("gps state: unlock. config time is error!\n");
            break;
        }
    } 
    printf("copyright\n");
}
