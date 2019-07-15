/**
 * @file usbserial3.cpp
 *
 * @brief This program is for Xbee wireless communication
 * @author K.Terae
 * @date 2019.7.9
 *
**/

// include
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <time.h>       // For measuring processing time
#include <sys/time.h>   // For measuring processing time
#include <random>       // For generating random number
#include <vector>
#include <signal.h>
#include <errno.h>

/* References
 makefile:
 http://www.ie.u-ryukyu.ac.jp/~e085739/c.makefile.tuts.html
 https://qiita.com/tortuepin/items/9861c75853b516c8a279

 c++ time.h:
 https://www.mm2d.net/main/prog/c/time-04.html
 https://fuku-dw.hatenadiary.org/entry/20111219/1324289674

 c++ termios.h:
 http://ssr-yuki.hatenablog.com/entry/2018/12/26/232817

 c++ escape sequence
 https://qiita.com/hideakitai/items/347985528656be03b620

 c++ signal.h
 https://uguisu.skr.jp/Windows/c_signal.html
 */


// define or declare constants
const uint8_t HEAD_BYTE  =0x7D;
const uint8_t ESCAPE_BYTE=0x7E;
const uint8_t ESCAPE_MASK=0x20;
#define SERIAL_PORT "/dev/ttyS16" // SDevice file corrensponding to serial interface
#define HZ 120                    // Communication frequency
#define LOOP_LENGTH 100           // loop length
#define DATA_SIZE 10              // unit : Bytes
#define MAX_DATA_TYPE 3

#define SIMULATE_WITHOUT_ROS

#define ENABLE_DBG             // Toggle when using printf() function
#ifndef ENABLE_DBG
#define DBG(...)
#else
#define DBG(...) printf("" __VA_ARGS__)
#endif

//#define DEBUG
#ifndef DEBUG
    #define debug( fmt, ... ) ((void)0)
#else
    #define debug( fmt, ... ) \
        fprintf( stderr, \
                  "[%s] %s:%u # " fmt "\n", \
                  __DATE__, __FILE__, \
                  __LINE__, ##__VA_ARGS__ \
        )
#endif

// declare global variable
typedef std::vector<uint8_t> vu8;
int loop_length,loop_count=0;
bool loop_count_enable=false;
volatile sig_atomic_t errorFlag=0;

void signalHandler(int signo)
{
  switch(signo)
  {
    case SIGQUIT:
    case SIGINT:
    case SIGKILL:
    case SIGILL:
    case SIGTERM:
      errorFlag=1;
      break;
    default:
      break;
  }
}

int createRandomNumber(int min, int max)
{
  int temp=0;
  if(min==max)
  {
    return min;
  }
  else if(min>max)
  {
    temp = min;
    min = max;
    max = temp;
  }

  std::uniform_int_distribution<int> dist(min,max);
  std::mt19937 mt{ std::random_device{}() };
  return dist(mt);
}

void setParameterFromCommandLine(int argc,char** argv,int *hz,int *loop_len)
{
  debug("setParameterFromCommandLine begin");
  if(argc>=2)
  {
    *hz=atoi(argv[1]);
    if(*hz<=0||100000<=*hz)
    {
      *hz=HZ;
    }
  }
  else
  {
    *hz=HZ;
  }

  if(argc>=3)
  {
    *loop_len=atoi(argv[2]);
    loop_count_enable=true;
    if(*loop_len<=0)
    {
      *loop_len=*hz;
      loop_count_enable=false;
    }
  }
  else
  {
    *loop_len=*hz;
      loop_count_enable=false;
  }
  printf("%d[Hz],loop %d times\n",*hz,*loop_len);
}


/*
 * data set 0 (datatype=0, velocity vector)
 * | 0-7(8) | 8-10(3) | 11-22(12) | 23-34(12) | 35-46(12) | 47-54(8) |
 * |--------|---------|-----------|-----------|-----------|----------|
 * |HEADER  |DATATYPE |X_VECTOR   |Y_VECTOR   |TH_VECTOR  |CHECKSUM  |
 * |--------|---------|-----------|-----------|-----------|----------|
 *
 * data set 1 (datatype=1, rotation calibration)
 * | 0-7(8) | 8-10(3) | 11-23(13) | 23-30(8) |
 * |--------|---------|-----------|----------|
 * |HEADER  |DATATYPE |CALIB_DATA | CHECKSUM |
 * |--------|---------|-----------|----------|
 *
 * data set 2 (datatype=2, kicker command & robot states)
 * | 0-7(8) | 8-10(3) | 11-15(5)  | 16-23(8) |
 * |--------|---------|-----------|----------|
 * |HEADER  |DATATYPE |COMMAND    |CHECKSUM  |
 * |--------|---------|-----------|----------|
 */

uint8_t datatype=0;
uint16_t x_vector=0,y_vector=0,th_vector=0,calib_data=0,command=0;

void vectorCallback()
{
  x_vector=createRandomNumber(0,pow(2,12)-1);
  y_vector=createRandomNumber(0,pow(2,12)-1);
  th_vector=createRandomNumber(0,pow(2,12)-1);
}

void visionCallback()
{
  calib_data=createRandomNumber(0,pow(2,13)-1);
}

void kickerCallback()
{
  command=createRandomNumber(0,pow(2,5)-1);
}

/* @setSendDataFromROSBus
 * @brief Split and store sending data in vector container
 * @param[out] buf Vector container. Each data must be split in 1byte(=8bit) data.
 * @detail This function doesn't add header and checksum data.
 */

void setSendDataFromROSBus(uint8_t datatype,vu8 *buf){
  uint8_t tmp=0;
  switch(datatype)
  {
    case 0:
      tmp=((datatype&0x7)<<5)|(x_vector>>7);
      buf->push_back(tmp);
      tmp=((x_vector&0x7F)<<1)|(y_vector>>11);
      buf->push_back(tmp);
      tmp=((y_vector>>3)&0xFF);
      buf->push_back(tmp);
      tmp=((y_vector&0x3F)<<5)|(th_vector>>8);
      buf->push_back(tmp);
      tmp=(th_vector&0xFF);
      buf->push_back(tmp);
      break;
    case 1:
      tmp=((datatype&0x7)<<5)|(calib_data>>8);
      buf->push_back(tmp);
      tmp=(calib_data&0xFF);
      buf->push_back(tmp);
      break;
    case 2:
      tmp=((datatype&0x7)<<5)|command;
      buf->push_back(tmp);
      break;
    default:
      break;
  }
//  datatype=(datatype+1)%MAX_DATA_TYPE;
}

/*
void setSendDataFromROSBus(vu8 *buf)
{
  // edit this function
  debug("setSendDataFromROSBus begin");
  static char increment=0;
  for(int i=0;i<DATA_SIZE-1;i++)
  {
    buf->push_back((uint8_t)createRandomNumber(0,255));
  }
  buf->push_back(increment);
  increment=(increment+1)%0x100;
}
*/

int main(int argc, char** argv)
{
  //ros::init(argc,argv,"sub_node_name");
  //ros::NodeHandle nh_vector,nh_vision,nh_kicker;
  //ros::Subscriber sub_vector=nh_vector.subscribe("vector_pub_node_name",1000,vectorCallback);
  //ros::Subscriber sub_vision=nh_vector.subscribe("vision_pub_node_name",1000,visionCallback);
  //ros::Subscriber sub_kicker=nh_vector.subscribe("kicker_pub_node_name",1000,kickerCallback);
  //ros::Rate rate(60);


  debug("main\n");
  int hz,looptime;
  setParameterFromCommandLine(argc,argv,&hz,&loop_length);
  debug("setParameterFromCommandLine end");
  const int microsecond=(int)(1000000.0f/hz);

  vu8 send_buffer;

  // initialize serial communication
  struct termios oldtio,newtio;         // Serial communication settings
  debug("openDevice begin\n");
  int fd=open(SERIAL_PORT,O_RDWR);      // open device
  debug("openDevice end\n");

  ioctl(fd,TCGETS,&oldtio);             // Evacuate current serial port settings
  newtio=oldtio;                        // Copy current serial port settings
  newtio.c_cflag=B115200|CREAD|CS8;         // Configure port settings. See man page termios(3)
  ioctl(fd,TCSETS,&newtio);             // Enable port settings

  struct timeval tv_start,tv_end; // for realtime sequence
  int checksum=0;
  uint8_t u8_checksum=0;


  if (signal(SIGINT, signalHandler) == SIG_ERR)
  {
    printf("\ncan't catch SIGINT\n");
  }
  if (signal(SIGQUIT, signalHandler) == SIG_ERR)
  {
    printf("\ncan't catch SIGQUIT\n");
  }
  if (signal(SIGILL, signalHandler) == SIG_ERR)
  {
    printf("\ncan't catch SIGILL\n");
  }
  if (signal(SIGTERM, signalHandler) == SIG_ERR)
  {
    printf("\ncan't catch SIGTERM\n");
  }


  while(!errorFlag)
  {
  // start timer
    gettimeofday(&tv_start,NULL);

    for(int i=0;i<MAX_DATA_TYPE;i++)
    {
    // get sending data from ROS bus
    // make sending byte-data buffer from integer-data
#ifdef SIMULATE_WITHOUT_ROS
      switch(i)
      {
        case 0:
          vectorCallback();break;
        case 1:
          visionCallback();break;
        case 2:
          kickerCallback();break;
        default:
          break;
      }
#endif
      setSendDataFromROSBus((uint8_t)i,&send_buffer);

    // send HEAD_BYTE as head-byte
      write(fd,&HEAD_BYTE,1);
      debug("write(fd,&HEAD_BYTE,1);");
      checksum=HEAD_BYTE;
      DBG("%4d", HEAD_BYTE);
    // while buffer isn't empty, sends byte-data from buffer
      for(auto itr=send_buffer.begin(); itr!=send_buffer.end(); ++itr)
      {
        debug("for loop begin");
    // if data compete with HEAD_BYTE or ESCAPE_BYTE, run escape sequence
        if(*itr==HEAD_BYTE||*itr==ESCAPE_BYTE)
        {
          write(fd,&ESCAPE_BYTE,1); // send escape byte
          DBG("%4d", ESCAPE_BYTE);
          *itr^=ESCAPE_MASK;        // escape mask
        }
    // send byte-data
        write(fd,(uint8_t*)&*itr,1);
        DBG("%4d", *itr);
        checksum+=*itr;
        debug("for loop end");
      }
      u8_checksum=checksum&0xFF;
      if(u8_checksum==HEAD_BYTE||u8_checksum==ESCAPE_BYTE)
      {
        write(fd,&ESCAPE_BYTE,1);
        DBG("%4d",ESCAPE_BYTE);
        u8_checksum^=ESCAPE_MASK;
      }
      write(fd,&u8_checksum,1);
      DBG("%4d\n", u8_checksum);

      send_buffer.erase(send_buffer.begin(), send_buffer.end());  //delete all elements of vector
    }

  // stop timer
    gettimeofday(&tv_end,NULL);

  // calculate wait_time to keep realtime sequence
    looptime=tv_end.tv_usec-tv_start.tv_usec;
    if(looptime<microsecond)
    {
      usleep(microsecond-looptime);
    }
    loop_count++;
    if(loop_count_enable&&loop_count>=loop_length)
    {
      printf("\nloopcount reach max\n");
      break;
    }
    debug("while loop end");
  }

  if(loop_count<loop_length)
  {
    fprintf(stderr,"[%s] %s:%u # Exit with signal error\n",__DATE__,__FILE__,__LINE__);
  }
  return 0;
}