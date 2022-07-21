#include<stdio.h>//标准输入输出定义 
#include<fcntl.h>//文件控制定义
#include<stdlib.h> //标准函数库定义
#include<unistd.h>//Unix标准函数定义
#include<errno.h>//错误好定义
#include<termios.h>//POSIX终端控制定义
#include<sys/ioctl.h>//ioctl函数定义
#include<string.h>//字符操作
#include<sys/types.h>
#include<sys/stat.h>
#include<pthread.h>
#include<sys/timeb.h>
#include<time.h>
#include<sys/select.h>
#include<linux/ioctl.h>
#include<linux/serial.h>
#include<asm-generic/ioctls.h>
#include<errno.h>
#include<getopt.h>
#define FALSE 0

//打开串口返回串口设备文件描述s
//port：串口号（ttyS),ttyS1,ttyS2）

typedef enum{DISABLE=0,ENABLE} RS485_ENABLE_t;

int UART0_open(char* port)
{
    int fd;
    fd=open(port,O_RDWR|O_NOCTTY|O_NDELAY);
    if(FALSE==fd)
    {
        perror("Can't Open Serial POrt");
        return FALSE;
    }
    //判断串口的状态是否为阻塞状态
    if(fcntl(fd,F_SETFL,0)<0)
    {
        printf("fcntl failed!\n");
        return FALSE;
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd,F_SETFL,0));
    }
    //测试是否为终端设备
    if(0==isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return FALSE;
    }
    else{
        printf("isatty success!\n");
    }
    printf("fd->open=%d\n",fd);
    return fd;
}



int Setopt(int fd,int nSpeed,int nBits,int nParity,int nStop)
{
    struct termios newtio,oldtio;
    if (tcgetattr(fd,&oldtio)!=0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio,sizeof(newtio));
    newtio.c_cflag |=CLOCAL | CREAD;
    newtio.c_cflag &=~CSIZE;
    switch(nBits)
    {
        case 7:
            newtio.c_cflag |=CS7;
            break;
        case 8:
            newtio.c_cflag |=CS8;
            break;
        default:
            fprintf(stderr,"Unsupported data size\n");
            return -1;
    }
    switch(nParity)
    {
        case 'o'://奇校验
        case 'O':
             newtio.c_cflag |=PARENB; //Enable parity bit
             newtio.c_cflag |=PARODD; //Use odd parity instead of even
             newtio.c_iflag |=(INPCK | ISTRIP); //INPCK:Enable parity check 将奇偶校验设置为有效
                                                 //ISTRIP:Strip parity bits 从接收字串中脱去奇偶校验位
             break;
        case 'e' :
        case 'E' :
             newtio.c_cflag |= PARENB;
             newtio.c_cflag &=~PARODD; //Use even parity 
             newtio.c_iflag |=(INPCK | ISTRIP);
             break;
        case 'n' :
        case 'N' :
             newtio.c_cflag &= ~PARENB; //无校验 disable parity bit
             break;
        default  :
             fprintf(stderr,"Unsupported parity\n"); //无校验 disable parity bit
             return -1;
    }
    switch (nStop)
    {
        case 1 :
             newtio.c_cflag &= ~CSTOPB;  
             break;
        case 2 :
             newtio.c_cflag |= CSTOPB;
             break;
        default :
             fprintf(stderr,"Unsupported stop bits\n");
             return -1;
    }
    switch(nSpeed)
    {
        case 2400:
            cfsetispeed(&newtio,B2400);
            cfsetospeed(&newtio,B2400);
            break;
        case 4800:
            cfsetispeed(&newtio,B4800);
            cfsetospeed(&newtio,B4800);break;
        case 9600:
            cfsetispeed(&newtio,B9600);
            cfsetospeed(&newtio,B9600);break;
        case 19200:
            cfsetispeed(&newtio,B19200);
            cfsetospeed(&newtio,B19200);break;
        case 38400:
            cfsetispeed(&newtio,B38400);
            cfsetospeed(&newtio,B38400);break;
        case 57600:
            cfsetispeed(&newtio,B57600);
            cfsetospeed(&newtio,B57600);break;
        case 115200:
            cfsetispeed(&newtio,B115200);
            cfsetospeed(&newtio,B115200);break;
        case 230400:
            cfsetispeed(&newtio,B230400);
            cfsetospeed(&newtio,B230400);break;
        case 1500000:
            cfsetispeed(&newtio,B1500000);
            cfsetospeed(&newtio,B1500000);break;
        default:
            printf("\tSorry,Unsupport baud rate ,set default 9600!\n\n");
            cfsetispeed(&newtio,B9600);
            cfsetospeed(&newtio,B9600);
            break;
        
    }
    newtio.c_cc[VTIME]=1;
    newtio.c_cc[VMIN]=1;
    tcflush(fd,TCIFLUSH);
    if(tcsetattr(fd,TCSANOW,&newtio)!=0)
    {
        perror("setupSerail 3");
        return -1;
    }
    printf("Serial set done !\n");
    return 0;

}

//UART0_Set
/*设置串口数据位，停止位校验位
  fd 串口文件描述符 
  speed 串口速度
  flow_ctrl 数据流控制
  databits 数据位 取值为 7或者8
  stopbits 停止位 取值为 1或者2
  parity 校验类型 取值为 N ，E，O，S
  出口参数： 正确返回0 错误返回为FLASE
*/
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int i;
    int status;
    int speed_arr[]={B38400,B19200,B38400,B19200,B9600,B4800,B2400,B1200,B300,B1500000};
    int name_arr[]={38400,19200,38400,19200,9600,4800,2400,1200,300,1500000};

    struct termios options;
    /*
    tcgetatteer(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options，该函数，
    还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */

   if(tcgetattr(fd,&options)!=0)
   {
    perror("SetupSerial 1\n");
    return FALSE;
   }
  

   //修改控制模式，保证程序不会占用串口
   options.c_cflag|=CLOCAL;
   //修改控制模式，使得能够从串口中读取输入数据
   options.c_cflag|=CREAD;

   //设置数据流控制
   switch(flow_ctrl)
   {
        case 0://不使用流控制
           options.c_cflag &=~CRTSCTS;
           break;
        case 1://使用硬件流控制
           options.c_cflag |=CRTSCTS;
           break;
        case 2://使用软件流控制
           options.c_cflag |=IXON|IXOFF|IXANY;
           break;
    }

    //设置数据位
    options.c_cflag &= ~CSIZE;//屏蔽其他标志位(清除数据位标志)
    switch(databits)
    {
        case 5 :
             options.c_cflag |=CS5;
             break;
        case 6 :
             options.c_cflag |=CS6;
             break;
        case 7 :
             options.c_cflag |=CS7;
             break;
        case 8:
             options.c_cflag |=CS8;
             break;
        default: 
             options.c_cflag |=CS8;
             break;
    }
    //设置校验位
    switch(parity)
    {
        case 'o'://奇校验
        case 'O':
             options.c_cflag |=PARENB; //Enable parity bit
             options.c_cflag |=PARODD; //Use odd parity instead of even
             options.c_iflag |=(INPCK | ISTRIP); //INPCK:Enable parity check 将奇偶校验设置为有效
                                                 //ISTRIP:Strip parity bits 从接收字串中脱去奇偶校验位
        case 'e' :
        case 'E' :
             options.c_cflag |= PARENB;
             options.c_cflag &=~PARODD; //Use even parity 
             options.c_iflag |=(INPCK | ISTRIP);
             break;
        case 'n' :
        case 'N' :
             options.c_cflag &= ~PARENB; //无校验 disable parity bit
             break;
        default  :
             options.c_cflag &= ~PARENB; //无校验 disable parity bit
             break;
    }

    //set the stop bits
    switch (stopbits)
    {
        case 1 :
             options.c_cflag &= ~CSTOPB;  
             break;
        case 2 :
             options.c_cflag |= CSTOPB;
             break;
        default :
             options.c_cflag &= ~CSTOPB;
             break;
    }
    //设置串口输入波特率和输出波特率
   for(i=0;i<sizeof(speed_arr)/sizeof(int);i++)
   {
    if(speed==name_arr[i])
    {
        cfsetispeed(&options,speed_arr[i]);
        cfsetospeed(&options,speed_arr[i]);
    }
   }
   
   //设置read读取最小字节数和超时时间
   //set timeout in deciseconds for nom-canonical read
   options.c_cc[VTIME] = 0; //读取一个字符等待0*（1/10）s
   //set minimum number of characters for non-canonical read
   options.c_cc[VMIN] = 0; //读取字符的最少个数位0

   tcflush(fd,TCIFLUSH); //清除缓存区

   /*set the parameters associated with the terminal from the termios 
      structure and the change occurs immdeiately*/
   if ((tcsetattr(fd,TCSANOW,&options))!=0)
   {
      perror("set_port/tcsettar");
      return FALSE;
   }
   return 0;
}

int rs485_enable(const int fd,const RS485_ENABLE_t enable)
{
    struct serial_rs485 rs485conf;
    int res;

    //获取设备485配置
    res =ioctl(fd,TIOCGRS485,&rs485conf);
    if(res<0)
    {
        perror("Ioctl error on getting 485 configure");
        close(fd);
        return res;
    }
    //设置485模式的使能/禁止
    if(enable)
    {
        //使能485模式
        rs485conf.flags |= SER_RS485_ENABLED;
    }
    else{
        //关闭485模式
        rs485conf.flags &= ~(SER_RS485_ENABLED);
    }
    rs485conf.delay_rts_before_send =0x00000004;
    
    //将485配置到设备中
    res=ioctl(fd,TIOCSRS485,&rs485conf);
    if(res<0)
    {
        perror("Ioctl error on setting 485 configure");
        close(fd);
    }
    return res;
}

int UART_Recv(int fd, char *rcv_buf,int data_len,int timeout)
{
    int len,fs_sel;
    fd_set fs_read;
    struct timeval time;

    time.tv_sec=timeout/1000;   //set the rcv wait time
    time.tv_usec = timeout % 1000 *1000; //100000us=0.1s

    FD_ZERO(&fs_read);  //每次循环都要清空集合，否则不能检测描述符变化
    FD_SET(fd,&fs_read);  //添加描述符

    //超时等待读变化， >0:就绪描述字的正数目，-1：出错，0：超时
    fs_sel=select(fd+1,&fs_read,NULL,NULL,&time);

    printf("fs_sel=%d",fs_sel);
    if(fs_sel)
    {
        len=read(fd,rcv_buf,data_len);
        return len;
    }
    else{
        printf("sorry,i am wrong\n");
        return -1;
    }
}

int UART_Send(int fd,char *send_buf,int data_len)
{
    ssize_t ret=0;
    ret =write(fd,send_buf,data_len);
    if(ret==data_len)
    {
        printf("send data is %s\n",send_buf);
        printf("%d\n",ret);
        return ret;
    }
    else{
        printf("write decivce error\n");
        tcflush(fd,TCOFLUSH);
        return -1;
    }
}

int main()
{
    int fd,i,nread;
    char read_buf[100];
    char *port="/dev/ttyS3";//选中串口的位置
    fd=UART0_open(port);//打开串口
    if(fd<0)
    {
        perror("open failed\n");
        return -1;
    }
    //rs485_enable(fd,ENABLE);

    //设置串口参数
    //i=UART0_Set(fd,9600,0,8,1,'N');
    i=Setopt(fd,9600,8,'N',1);
    if (i!=0){
        perror("set_port failed");
        return FALSE;
    }
    
    while(1)
    {
         
        //if new data is acailable on the serial port ,read and print it out
        
        
        nread = read(fd,read_buf,99);
        
        //nread=UART_Recv(fd,read_buf,99,10000);
        if (nread>0)
        {
            
           
           /*printf("RECV[%3d]: \n",nread);
            for (i=0;i<nread;i++)
                 printf("0x%02x",read_buf[i]);
            printf("\n");
            write(fd,read_buf,nread);*/
            read_buf[nread]='\0';
            for (i=0;i<nread;i++)
                 printf("0x%02x",read_buf[i]);
            printf("\n");
            printf("nread = %d\n",nread);
            

            UART_Send(fd,read_buf,nread);
        }
    }
    
    close(fd);
    return 0;


}



