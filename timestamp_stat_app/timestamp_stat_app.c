#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <signal.h>

#define BUF_LEN 400
#define INSERT_PORT 102
#define TIME_DBG_FUN 0
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

unsigned short g_insert_port = INSERT_PORT;
unsigned short g_debug_level = 0;
int g_time_delta = 0;
extern volatile int gu32timeokflag;

void stat_timestamp_data(unsigned char*data,unsigned int len)
{

}
//以1ms为单位统计延迟数据分布
#define DELAY_STATISTIC 1000
//统计延迟最大的8000个数据延迟数值和发生的时刻
#define MAX_DELAY_CNT   1000*8

struct statistic_time{
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
    unsigned char rsv;
};

struct radio_state{
    unsigned char rat;//记录无线信号类型4G还是5G
    unsigned char pci;//小区ID
    char sinr;//有效范围-20到50，单位dB
    char rsrp; //有效范围-50到-120，单位dBm
};

struct statistic_delay{
    struct statistic_time start_time;
    unsigned int iDelay[DELAY_STATISTIC];
	unsigned int time_err;
    struct statistic_time end_time;
};

struct statistic_packet{
    struct statistic_time cur_time;
    struct radio_state state;
    union{
        struct{//用于统计延迟的时间
            unsigned short msec;
            char reserve[4];
        }delay;
        struct{//用于统计丢包
            unsigned int seq;
            char reserve[2];
        }discard;
        struct{//用于统计乱序
            unsigned int seq;//错乱包序号
            unsigned short diff_seq;//实际到达临近包序号与错乱包序号差
        }miss_order;
    }un;
};

struct statistic_delay st_delay;
struct statistic_packet max_delay[MAX_DELAY_CNT];
struct statistic_packet discard[MAX_DELAY_CNT];
struct statistic_packet mis_order[MAX_DELAY_CNT];
u32 g_max_delay_index = 0;
u32 g_discard_index = 0;
u32 g_mis_order_index = 0;
u32 g_mis_order=0;

//struct sigaction act_usr1;
struct sigaction act_usr2;


extern void get_radio_state(struct radio_state*pst);
extern void nr5g_send_gtccinfo();
extern int nr5g_up();


typedef struct /*ipovly*/
{
	u32	id;
	u32	sec_send;
	u32	us_send;  
	u32	sec_rec;
	u32	us_rec;
}ST_INSERT;


#define BUF_SIZE (20*2000)//1s数据
//#define FLAG_A 0
//#define FLAG_B 1

#define YES 1
#define NO 0

u8 g_kernel_buf_A[BUF_SIZE];
//u8 g_kernel_buf_B[BUF_SIZE];
//u32 g_read_flag = FLAG_A;
//u32 g_bufB_readed_flag = NO;
u32 g_start_flag = NO;

void init_stat_data()
{
	g_start_flag= NO;
	memset(&st_delay,0,sizeof(st_delay));
	memset(&max_delay,0,sizeof(max_delay));
	memset(&discard,0,sizeof(discard));
	memset(&mis_order,0,sizeof(mis_order));
	g_max_delay_index = 0;
	g_discard_index = 0;
	g_mis_order_index = 0;

}


u32 set_insert_config()
{
	int fd,i;
	int r_len = 0;
	u8 *buffer = NULL;

	buffer = g_kernel_buf_A;
	fd = open("/dev/simple", O_RDWR);
	if (fd == -1){
//	  printf("get_kernel_buf open device button errr!, line:%d\n",__LINE__);
	  return 0;
	}
	buffer[0]=0x11;
	buffer[1]=0x11;
	buffer[2]=0x00;
	buffer[3]=0x00;
	buffer[4]=0x00;
	buffer[5]=0x00;
	r_len = read(fd,buffer,7);
/*    printf("get_kernel_buf sizeofbuf:7, line:%d\n buffer ",__LINE__);
	for(i=0;i<8;i++){
		printf("[%d]-0x%02x  ",i,buffer[i]);
	}
	printf("\n");
*/	
	close(fd);
	return r_len;
}

u32 get_kernel_buf()
{
	int fd,i;
	int r_len = 0;
	u8 *buffer = NULL;
	struct timespec now1,now2;

	buffer = g_kernel_buf_A;
	
	fd = open("/dev/simple", O_RDWR);

	if (fd == -1)
	{
//	  printf("get_kernel_buf open device button errr!, line:%d\n",__LINE__);
	  return 0;
	}
	buffer[0]=0x12;
	buffer[1]=0x12;
	clock_gettime(CLOCK_MONOTONIC, &now1);

	r_len = read(fd,buffer,sizeof(buffer));

	clock_gettime(CLOCK_MONOTONIC, &now2);
#if TIME_DBG_FUN
	printf("--- ++ *** get_kernel_buf r_len: %d---,line: %d\n",r_len,__LINE__);
	printf("--- ++ *** recv timestamp, line: %d, tv_sec:tv_usec: %d:%d\r\n",__LINE__,
		(unsigned int)now1.tv_sec,(unsigned int)now1.tv_nsec/1000);
	printf("--- ++ *** recv timestamp, line: %d, tv_sec:tv_usec: %d:%d, diff(us): %d\r\n",__LINE__,
		(unsigned int)now2.tv_sec,(unsigned int)now2.tv_nsec/1000,
		(unsigned int)now2.tv_nsec/1000-(unsigned int)now1.tv_nsec/1000);
#endif
//    printf("get_kernel_buf sizeofbuf:%d, line:%d\n buffer ",r_len,__LINE__);

#if 0
	for (i=0;i <r_len;i++)
	printf("%c",buffer[i]);
	printf("\r\n");
#endif
	close(fd);
	return r_len;
}

void pro_insert_data()
{
	u32 r_len,cnt_loop,i,id=0;
	int sec_delta,ms_delay;
	ST_INSERT *pstData =NULL;
	struct tm *local;
	u8* p=NULL;
	time_t t; 
	struct timespec now1,now2;

	t=time(NULL);
	local=localtime(&t);

	clock_gettime(CLOCK_MONOTONIC, &now1);

	r_len = get_kernel_buf();

#if TIME_DBG_FUN
	clock_gettime(CLOCK_MONOTONIC, &now2);
	printf("--- ++ get_kernel_buf---line: %d\n",__LINE__);
	printf("--- ++ recv timestamp, line: %d, tv_sec:tv_usec: %d:%d\r\n",__LINE__,
		(unsigned int)now1.tv_sec,(unsigned int)now1.tv_nsec/1000);
	printf("--- ++ recv timestamp, line: %d, tv_sec:tv_usec: %d:%d, diff(us): %d\r\n",__LINE__,
		(unsigned int)now2.tv_sec,(unsigned int)now2.tv_nsec/1000,
		(unsigned int)now2.tv_nsec/1000-(unsigned int)now1.tv_nsec/1000);
#endif

	if(r_len==BUF_SIZE)
	{
		if(g_start_flag==NO)
		{
			g_start_flag=YES;
			st_delay.start_time.year = (local->tm_year+1900);
			st_delay.start_time.month = (local->tm_mon+1);
			st_delay.start_time.day = local->tm_mday;
			st_delay.start_time.hour = local->tm_hour;
			st_delay.start_time.minute = local->tm_min;
			st_delay.start_time.second = local->tm_sec;
		}
		pstData = (ST_INSERT*)g_kernel_buf_A;
		cnt_loop = BUF_SIZE/sizeof(ST_INSERT);
		for(i=0;i<cnt_loop;i++)
		{
			//统计时延 ,默认收发双发时钟同步
			sec_delta = (pstData[i].sec_rec+g_time_delta)-pstData[i].sec_send;
			ms_delay = sec_delta*1000+(pstData[i].us_rec/1000)-(pstData[i].us_send/1000);
/*			printf("******************************************************\n");
			printf("***********          pstData[i].us_send/1000 %d  *****\n",pstData[i].us_send/1000);
			printf("***********          pstData[i].us_rec/1000  %d  *****\n", pstData[i].us_rec/1000);
			printf("******************************************************\n"); */
			if(ms_delay<0)
			{
				st_delay.time_err++;
			}
			else if(ms_delay<DELAY_STATISTIC)
			{
				st_delay.iDelay[ms_delay]++;
			}
			else
			{//时延大于1s
				max_delay[g_max_delay_index].cur_time.year = (local->tm_year+1900);
				max_delay[g_max_delay_index].cur_time.month = (local->tm_mon+1);
				max_delay[g_max_delay_index].cur_time.day = local->tm_mday;
				max_delay[g_max_delay_index].cur_time.hour = local->tm_hour;
				max_delay[g_max_delay_index].cur_time.minute = local->tm_min;
				max_delay[g_max_delay_index].cur_time.second = local->tm_sec;
				max_delay[g_max_delay_index].un.delay.msec = ms_delay;
				get_radio_state(&max_delay[g_max_delay_index].state);
				g_max_delay_index++;
				if(g_max_delay_index==MAX_DELAY_CNT)
					g_max_delay_index = 0;
				
			}
			//处理乱序
			if(i==0)
			{
			}
			else
			{
				if(pstData[i].id<pstData[i-1].id)//乱序
				{
					//printf("mis_order:%d,%d,%d,%d\r\n",i,g_mis_order_index,pstData[i-1].id,pstData[i].id);
					g_mis_order++;
					#if 0
					p = (u8*)&pstData[i-1];
					for(i=0;i<40;i++)
					{
						printf("%02x ",p[i]);
						if(i%8==0)
							printf("\r\n");
					}
					printf("\r\n");
					#endif
					mis_order[g_mis_order_index].cur_time.year = (local->tm_year+1900);
					mis_order[g_mis_order_index].cur_time.month = (local->tm_mon+1);
					mis_order[g_mis_order_index].cur_time.day = local->tm_mday;
					mis_order[g_mis_order_index].cur_time.hour = local->tm_hour;
					mis_order[g_mis_order_index].cur_time.minute = local->tm_min;
					mis_order[g_mis_order_index].cur_time.second = local->tm_sec;
					get_radio_state(&mis_order[g_mis_order_index].state);
					mis_order[g_mis_order_index].un.miss_order.seq = pstData[i].id;
					mis_order[g_mis_order_index].un.miss_order.diff_seq = pstData[i-1].id+1-pstData[i].id;
					g_mis_order_index++;
					if(g_mis_order_index==MAX_DELAY_CNT)
						g_mis_order_index = 0;
				}
			}
			//处理丢包，这个比较麻烦，与盖总商量暂时不处理，将时延大于1s为丢包
			
		}
		
	}
			
}

void store_stat_file()
{
	int i,j,ret,buf_point;

	char str_store[1024*1024]={0};
	char str_buf[200]={0};
	char file_name[32]={0};
	FILE *fp;
	time_t localTime;
	struct tm *pLocalTime;

	if(g_start_flag==NO)
	{
		printf("no stat data......\r\n");
		return;
	}
	time(&localTime);
	pLocalTime = localtime(&localTime);

	sprintf(file_name,"/usr/netfilter-%02d-%02d-%02d-%02d.log",(1+pLocalTime->tm_mon),
		pLocalTime->tm_mday,(pLocalTime->tm_hour+0), pLocalTime->tm_min);
	st_delay.end_time.year = (pLocalTime->tm_year+1900);
	st_delay.end_time.month = (pLocalTime->tm_mon+1);
	st_delay.end_time.day = pLocalTime->tm_mday;
	st_delay.end_time.hour = pLocalTime->tm_hour;
	st_delay.end_time.minute = pLocalTime->tm_min;
	st_delay.end_time.second = pLocalTime->tm_sec;
	
	fp=fopen(file_name,"w+");
	 if(NULL != fp)
	{
	  buf_point = 0;
	  sprintf(str_buf,"st_delay start_time:%04d-%02d-%02d-%02d-%02d-%02d,",st_delay.start_time.year,st_delay.start_time.month,\
	  	st_delay.start_time.day,st_delay.start_time.hour,st_delay.start_time.minute,st_delay.start_time.second);
	  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
	  buf_point+=strlen(str_buf);

	  sprintf(str_buf,"end_time:%04d-%02d-%02d-%02d-%02d-%02d\r\n",st_delay.end_time.year,st_delay.end_time.month,\
	  	st_delay.end_time.day,st_delay.end_time.hour,st_delay.end_time.minute,st_delay.end_time.second);
	  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
	  buf_point+=strlen(str_buf);

	  if(st_delay.time_err)
	  {
			sprintf(str_buf,"time_err_cnt:%08d\r\n",st_delay.time_err);
			memcpy(str_store+buf_point,str_buf,strlen(str_buf));
			buf_point+=strlen(str_buf);
	  }
	    
	  for(i=0;i<DELAY_STATISTIC;i++)
	  {
	  	  if(st_delay.iDelay[i]!=0)
	  	  {
			  sprintf(str_buf,"%04dms:%08d\r\n",i,st_delay.iDelay[i]);
			  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
			  buf_point+=strlen(str_buf);
	  	  }
	  }

	  sprintf(str_buf,"------max_delay data------\r\n");
	  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
	  buf_point+=strlen(str_buf);
	  for(i=0;i<MAX_DELAY_CNT;i++)
	  {
	  	  if(max_delay[i].un.delay.msec==0)
	  	  {
			  sprintf(str_buf,"max_delay data total:%d\r\n",i);
			  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
			  buf_point+=strlen(str_buf);
			  break;
		  }
		  sprintf(str_buf,"happen_time:%04d-%02d-%02d-%02d-%02d-%02d,",max_delay[i].cur_time.year,max_delay[i].cur_time.month,\
	  	  max_delay[i].cur_time.day,max_delay[i].cur_time.hour,max_delay[i].cur_time.minute,max_delay[i].cur_time.second);
		  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
		  buf_point+=strlen(str_buf);

		  sprintf(str_buf,"rat:%02d pci:%02d sinr:%02d rsrp:%02d,",max_delay[i].state.rat,max_delay[i].state.pci,max_delay[i].state.sinr,max_delay[i].state.rsrp);
		  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
		  buf_point+=strlen(str_buf);
		  
		  sprintf(str_buf,"delay:%d\r\n",max_delay[i].un.delay.msec);
		  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
		  buf_point+=strlen(str_buf);

	  }
	  
	  sprintf(str_buf,"------mis_order data------\r\n");
	  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
	  buf_point+=strlen(str_buf);
	  for(i=0;i<MAX_DELAY_CNT;i++)
	  {
	  	  if(mis_order[i].un.miss_order.seq==0)
	  	  {
			  sprintf(str_buf,"mis_order data total:%d\r\n",i);
			  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
			  buf_point+=strlen(str_buf);
			  break;
		  }
		  sprintf(str_buf,"happen_time:%04d-%02d-%02d-%02d-%02d-%02d,",mis_order[i].cur_time.year,mis_order[i].cur_time.month,\
	  	  mis_order[i].cur_time.day,mis_order[i].cur_time.hour,mis_order[i].cur_time.minute,mis_order[i].cur_time.second);
		  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
		  buf_point+=strlen(str_buf);

		  sprintf(str_buf,"rat:%02d pci:%02d sinr:%02d rsrp:%02d,",mis_order[i].state.rat,mis_order[i].state.pci,mis_order[i].state.sinr,mis_order[i].state.rsrp);
		  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
		  buf_point+=strlen(str_buf);
		  
		  sprintf(str_buf,"seq:%d,diff_seq:%d\r\n",mis_order[i].un.miss_order.seq,mis_order[i].un.miss_order.diff_seq);
		  memcpy(str_store+buf_point,str_buf,strlen(str_buf));
		  buf_point+=strlen(str_buf);

	  }
	  

	  fwrite(str_store, strlen(str_store), 1, fp);
	  fclose(fp);
	  init_stat_data();
	}
	else
		  printf("\n********Can't open log filer\n");

}

//interupt signal process program
void usr1_handler(int sig) {
	store_stat_file();
  
}

void usr2_handler(int sig) {
	struct timespec now1,now2;

	printf("-----[file]timestamp_stat usr_handler SIGINT, LINE: %d\n",__LINE__);
	clock_gettime(CLOCK_MONOTONIC, &now1);
	store_stat_file();
	clock_gettime(CLOCK_MONOTONIC, &now2);
	printf("------------------------------------------------------\n");
	printf("------------------------------------------------------\n");
	printf("---usr2_handler---line: %d\n",__LINE__);
	printf("---recv timestamp, line: %d, tv_sec:tv_usec: %d:%d\r\n",__LINE__,
		(unsigned int)now1.tv_sec,(unsigned int)now1.tv_nsec/1000);
	printf("---recv timestamp, line: %d, tv_sec:tv_usec: %d:%d, diff(us): %d\r\n",__LINE__,
		(unsigned int)now2.tv_sec,(unsigned int)now2.tv_nsec/1000,
		(unsigned int)now2.tv_nsec/1000-(unsigned int)now1.tv_nsec/1000);
	printf("------------------------------------------------------\n");
	printf("------------------------------------------------------\n");
	printf("------ end store stat file\n");
}



//signal handler
void set_sighandler() {
/*
    act_usr1.sa_handler=usr1_handler;
    printf("int handler handle signal: %d\n", SIGUSR1);
    if(sigaction(SIGUSR1,&act_usr1,NULL)==-1)
    {
        printf("SIGINT handler setting fails.");
		exit(1);
    }*/
	act_usr2.sa_handler=usr2_handler;
    printf("int handler handle signal: %d\n", SIGINT);
    //if(sigaction(SIGUSR2,&act_usr2,NULL)==-1)
    if(sigaction(SIGINT,&act_usr2,NULL)==-1) //sig 改为int
    {
        printf("SIGINT handler setting fails.");
		exit(1);
    }
}

void time_sync_pro()
{
	int iHour,iMinute,iSecond;
	struct timespec ts;
	u32 time_sync_sec_cnt=0,call_time_sync_cnt=0;

//gaip
	printf("\n == time sync task start... line :%d\n",__LINE__);
	sgitg_gps();
	printf("\n == time_sync end line: %d\r\n",__LINE__);

	while(1)
	{
		time_sync_sec_cnt++;
		if(time_sync_sec_cnt%3600==0)//校验时间
		{   //gaip 
			gu32timeokflag=0;
			clock_gettime(CLOCK_MONOTONIC, &ts);
			iHour = ts.tv_sec/3600;
			iMinute = ts.tv_sec/60 - iHour*60;
			iSecond = ts.tv_sec - iMinute * 60 - iHour * 3600;
			printf("\n == run time hour:mintue:second == %d:%d:%d \n",iHour,iMinute,iSecond);
			call_time_sync_cnt++;
		}
		sleep(1);
		
	}
		
}

int main()
{
	u32 sec_cnt=0;
	int time_delta = 0;
	char ch;
	int i,calc_cycle;
	int ret;
	pthread_t tid_time_sync = 0;
	struct timespec now1,now2;
	init_stat_data();
	u8* p;
	int fileReadCnt;
	printf("++++++++++++++++++++++++++++++++++\n");
	printf("++++++ 2020 01 19 change  ++++++++\n");
	printf("++++++ copyright v1.8  +++++++++++\n");
	printf("++++++++++++++++++++++++++++++++++\n");
	g_time_delta = time_delta;
/*
	while(1)
	{
		printf("\r\n V2.1 20210124-17:58 Please input calc cycle(ms):\n");
		scanf("%d",&calc_cycle);
		printf("your calc_cycle is:%d\n",calc_cycle);
		break;
	}
*/	
	calc_cycle = 500;
	set_sighandler();
//  gaip
//	nr5g_up();
	ret = pthread_create(&tid_time_sync, NULL, (void *)time_sync_pro, NULL);
	if (ret != 0) {
		printf( "FATAL: Failed to create a new thread (time_sync_pro) - exiting");
	}
	pthread_detach(tid_time_sync);

	while(1)
	{
		//60s set config
		if(sec_cnt%60==0) set_insert_config();

		clock_gettime(CLOCK_MONOTONIC, &now1);

		pro_insert_data();

#if TIME_DBG_FUN
		clock_gettime(CLOCK_MONOTONIC, &now2);
		printf("---pro_insert_data---sec_cnt: %d,line: %d\n",sec_cnt,__LINE__);
		printf("---recv timestamp, line: %d, tv_sec:tv_usec: %d:%d\r\n",__LINE__,
			(unsigned int)now1.tv_sec,(unsigned int)now1.tv_nsec/1000);
		printf("---recv timestamp, line: %d, tv_sec:tv_usec: %d:%d, diff(us): %d\r\n",__LINE__,
			(unsigned int)now2.tv_sec,(unsigned int)now2.tv_nsec/1000,
			(unsigned int)now2.tv_nsec/1000-(unsigned int)now1.tv_nsec/1000);
#endif
		//sleep(1);
		usleep(calc_cycle*1000);//0.5s
		sec_cnt++;
		if(sec_cnt%(3600)==0)//1小时存一个文件 gaip
		{
			store_stat_file();
		}
	}
}


