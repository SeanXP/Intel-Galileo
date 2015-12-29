/****************************************************************
  Copyright (C) 2014 All rights reserved.

  > File Name:         < test_tty.c >
  > Author:            < Shawn Guo >
  > Mail:              < iseanxp@gmail.com >
  > Created Time:      < 2014/04/11 >
  > Last Changed: 
  > Description:   linux配置串口

  echo > /dev/ttyx  从这个终端, 向串口发送数据, 数据将显示在对应串口上. 
  write函数, 作用同上.

  cat /dev/ttyx 建立从这个终端到对应终端串口的通道, 如果对方有任何输入, 都将返回到这里.
  类似于read的作用.

  不能利用cat查看echo给此串口发出的消息, 同理, wirte给对方串口的消息, 不能再read到,除非串口自己转发

  screen 是登录到这个串口里, 所以你看到的,就是别人给此串口发的数据, 你发出的, 就是给别人监听到的消息
  因此screen可以与另一端的cat, echo交互.
 ****************************************************************/


#include <stdio.h>      /*标准输入输出定义*/ 
#include <stdlib.h>     /*标准函数库定义*/ 
#include <unistd.h>     /*Unix 标准函数定义*/ 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <fcntl.h>      /*文件控制定义*/ 
#include <termios.h>    /*PPSIX 终端控制定义*/ 
#include <errno.h>      /*错误号定义*/ 
#include <string.h>

//#define DEV_TTY_NAME    "/dev/tty.usbserial-A7027C8G"
#define DEV_TTY_NAME    "/dev/cu.usbserial-A7027C8G"
//#define DEV_TTY_NAME "/dev/ttys004"
//#define DEV_TTY_NAME "/dev/cu.usbmodem1451"
#define BAUDRATE        57600         //定义标准波特率

//打开串口,返回文件描述符号
int open_tty();

/*************************
    串口信息配置
    文件描述符号    fd
    波特率          baud_rate        (9600, 57600, 默认115200...)
    数据位          data_bits        (7, 默认8)
    奇偶校验位      parity           (默认N, O, E)
    停止位          stop_bits        (默认1, 2)   
*************************/
int set_com_config(int fd,int baud_rate, int data_bits,char parity,int stop_bits);

char buffer[1024] = {0}; 
char buffer2[] = "hello, this is sean from macbook!";
int main()
{
    int fd = 0;
    struct termios oldtio,newtio;
    int return_val = 0;
    char ch;

    fd = open_tty();
    set_com_config(fd, BAUDRATE, 8, 'N', 1);
    
    while(1)
    {
        return_val = read(fd, buffer, 1000);
        if(return_val > 0)
        {
            printf("%s", buffer);
        }
    }

 //   write(fd, buffer2, sizeof(buffer2));
    //close dev tty.
    close(fd);
    return 0;
}

//打开串口,返回文件描述符号
int open_tty()
{
    int fd;
    struct termios Opt;
    int return_val = 0;

    fd = open(DEV_TTY_NAME, O_RDWR | O_NOCTTY); //O_NDELAY
    // O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行；

    if(fd == -1)
    {
        //打开错误
        perror("Error: Can't open this device ");
        exit(1);
    }
    else
        return fd;
}

/*****************************************************************
  串口的设置主要是设置 struct termios 结构体的各成员值来实现。

  struct termios
  {
  Unsigned short c_iflag;            //输入模式标志
  Unsigned short c_oflag;           //输出模式标志
  Unsigned short c_cflag;           //控制模式标志
  Unsigned short c_lflag;            //本地模式标志
  Unsigned char c_line ;             //线路规则
  Unsigned char c_cc[NCC];              //控制特性
  Speed_t        c_ispeed;             //输入速度
  Speed_t        c_ospeed;            //输出速度
  };
  串口设置包括波特率设置，检验位和停止位设置，主要设置的是c_cflag结构体成员，
  注意，不能直接对c_cflag成员初始化，而要将其通过与或操作。

  一个默认值示例: eg.....................
  (termios) $2 = {
  c_iflag = 0
  c_oflag = 0
  c_cflag = 215808
  c_lflag = 0
  c_cc = {
  [0] = '\x04'
  [1] = '?'
  [2] = '?'
  [3] = '\x7f'
  [4] = '\x17'
  [5] = '\x15'
  [6] = '\x12'
  [7] = '?'
  [8] = '\x03'
  [9] = '\x1c'
  [10] = '\x1a'
  [11] = '\x19'
  [12] = '\x11'
  [13] = '\x13'
  [14] = '\x16'
  [15] = '\x0f'
  [16] = '\x01'
  [17] = '\0'
  [18] = '\x14'
  [19] = '?'
  }
  c_ispeed = 9600
  c_ospeed = 9600
  }
 ********************************************************************/

int set_com_config(int fd,int baud_rate, int data_bits,char parity,int stop_bits)
{
    struct termios cfg;
    int speed;

    //tcgetattr函数用于获取与终端相关的参数。参数fd为终端的文件描述符，返回的结果保存在termios 结构体中
    /*保存并测试现有串口参数设置，在这里如果串口号等出错，会有相关出错信息*/
    if(tcgetattr(fd, &cfg) != 0) /* 该函数得到fd指向的终端配置参数，并将它们保存到old_cfg变量中，成功返回0，否则-1*/
    {
        perror("tcgetttr");
        return -1;
    }

    /*设置波特率*/
    switch(baud_rate)
    {
        case 2400:
            speed = B2400;
            break;
        case 4800:
            speed = B4800;
            break;
        case 9600:
            speed = B9600;
            break;   
        case 19200:
            speed = B19200;
            break;                        
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        default:    //默认波特率为115200
        case 115200:
            speed = B115200;
            break;
    }
    cfsetispeed(&cfg,speed);
    cfsetospeed(&cfg,speed);


    //设置数据位
    //在设置数据位时，必须先使用CSIZE做位屏蔽,清空数据位, 才可以重新配置
    cfg.c_cflag &= ~CSIZE; // 用位掩码清空数据位的设置
    switch(data_bits)
    {
        case 7:
            cfg.c_cflag |= CS7;
            break;
        default:
        case 8:
            cfg.c_cflag |= CS8;
            break;
    }

/*
    //设置奇偶校验位, 可以选择奇校验, 偶校验, 空格, 无校验.
    //另外一种配置, 暂时不知道那个是好的.
    //无校验: opt.c_cflag &= ~PARENB;
    //奇校验: opt.c_cflag |= (PARODD | PARENB);
    //偶校验: opt.c_cflag &= ~ PARENB;opt.c_cflag &= ~PARODD;
    //空格: opt.c_cflag &= ~PARENB;opt.c_cflag &= ~CSTOPB;

    switch(parity)
    {
        default:
        case 'n':
        case 'N':
            {
                cfg.c_cflag &= ~PARENB;
                cfg.c_iflag &= ~INPCK;
            }
            break;
        case 'o':
       case 'O':   //奇校验ODD
            {
                cfg.c_cflag |= (PARODD | PARENB);
                cfg.c_iflag |= INPCK;
            }
            break;
        case 'e':
        case 'E':   //偶校验
            {
                cfg.c_cflag |=  PARENB;
                cfg.c_cflag &= ~PARODD;
                cfg.c_iflag |= INPCK;
            }
            break;

    }

    //设置停止位
    switch(stop_bits)
    {
        default:
        case 1:
            cfg.c_cflag &= ~CSTOPB;
            break;
        case 2:
            cfg.c_cflag |= CSTOPB;
            break;
    }
*/


    //设置等待时间和最小接收字符
    //决定了对串口读取的函数read（）的一些功能
    //（VTIME和VMIN）对应的元素不是控制符，并且只是在原始模式下有效。
    //只有在原始模式下，他们决定了read（）函数在什么时候返回
    //在标准模式下,除非设置了O_NONBLOCK选项，否则只有当遇到文件结束符或各行的字符都已经编辑完毕后才返回

    //VTIME定义要求等待的零到几百毫秒的时间量（通常是一个8位的unsigned char变量，取值不能大于cc_t
    //VMIN定义了要求等待的最小字节数（不是要求读的字节数 read()的第三个参数才是指定要求读的最大字节数),这个字节数可能是0.
    cfg.c_cc[VTIME]=0;
    cfg.c_cc[VMIN]=50; //建议此值应该大于等于每次发送的字符串, 以免读两次截断.

    //如果VTIME取0, VMIN定义了要求等待读取的最小字节数, 
    //函数read()只有在[ 读取了VMIN个字节的数据 ] 或者 [ 收到一个信号 ] 的时候才返回。

    //如果VMIN取0, VTIME定义了即使没有数据可以读取, read()函数返回前也要等待几百毫秒的时间量。
    //此时read()函数不需要像其通常情况那样要遇到一个文件结束标志才返回0.

    //如果VTIME和VMIN都不取0，VTIME定义的是当接收到第一个字节的数据后开始计算等待的时间量。
    //如果当调用read函数时可以得到数据，计时器马上开始计时。
    //如果当调用read函数时还没有任何数据可读，则等接收到第一个字节的数据后，计时器开始计时.
    //函数read可能会在读取到VMIN个字节的数据后返回, 也可能在计时完毕后返回, 
    //这主要取决于哪个条件首先实现.不过函数至少会读取到一个字节的数据，因为计时器是在读取到第一个数据时开始计时的。

    //如果VTIME和VMIN都取0，即使读取不到任何数据，函数read也会立即返回。
    //同时，返回值0表示read函数不需要等待文件结束标志就返回了。

    //数据流控制, 使用何种方法来标志数据传输的开始和结束
    //不使用数据流控制 
    cfg.c_cflag &= ~CRTSCTS;
    // 硬件 opt.c_cflag |= CRTSCTS (需要相应连接的电缆（就是RTS，CTS那两根线))
    // 软件 opt.c_cflag | = IXON|IXOFF|IXANY

    
    //原始模式
    //在原始模式下，它们决定了read()函数在什么时候return.
    cfg.c_lflag &=~ICANON;


    /*处理未接收字符
     int tcflush(int filedes，int quene);
       quene数该当是下列三个常数之一:
            *TCIFLUSH  刷清输入队列
            *TCOFLUSH  刷清输出队列
            *TCIOFLUSH 刷清输入、输出队列
     */
//    tcflush(fd,TCIFLUSH);

    /*激活新配置*/
    //tcsetattr函数用于设置终端的相关参数。参数fd为打开的终端文件描述符，
    //参数optional_actions用于控制修改起作用的时间，而结构体termios_p中保存了要修改的参数。
    //成功返回0, 失败返回-1.
    //TCSANOW：不等数据传输完毕就立即改变属性。 
    if((tcsetattr(fd,TCSANOW,&cfg))!=0)
    {
        perror("tcsetattr");
        return -1;
    }
    return 0;
}
