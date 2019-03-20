#include "serialport.h"

SerialPort::SerialPort()
{
    cout<<"The Serial set ......"<<endl;
}

SerialPort::~SerialPort(void)
{
    if (!close(fd))
        printf("Close Serial Port Successful\n");
}

void SerialPort::serialSet(int port_No)
{
    /** 打开串口
     *  @param:  port_No 串口编号,默认值为1,即COM1,注意,尽量不要大于9
     *  @return
     */
    /** @brief: 初始化串口函数
    * @Default
    *  @param:  波特率,默认为115200
    * @Default
    *  @param:  char parity 是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验
    * @Default
    *  @param:  int databits 数据位的个数,默认值为8个数据位
    *
    *  @return: bool  初始化是否成功
    *  @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
    *　　　　　   函数提供了一些常用的串口参数设置
    *           本串口类析构时会自动关闭串口,无需额外执行关闭串口
    * @author: Hzkkk
    */
    const char* DeviceName[4] = {"", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};

     /* WARNING :  终端设备默认会设置为控制终端，因此open(O_NOCTTY不作为控制终端)
     * Terminals'll default to be set as Control Terminals
     */
    struct termios newstate, option;
     /*打开串口*/
    fd=open(DeviceName[port_No], O_RDWR|O_NONBLOCK|O_NOCTTY|O_NDELAY);
    if (fd == -1)
    {
        perror("Can't Open Serial Port\n");
    }
    else
        printf("Open Serial Port %s Successful\n", DeviceName[port_No]);

    /*改为阻塞模式*/
    if (fcntl(fd, F_SETFL, 0) < 0)
        printf("fcntl failed!\n");
    else
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));

    tcgetattr(fd, &newstate);
    /*设置发送波特率*/
    cfsetospeed(&newstate, B115200);
    cfsetispeed(&newstate, B115200);
    //获取波特率
    tcgetattr(fd, &option);

    //本地连线, 取消控制功能 | 开始接收
    newstate.c_cflag |= CLOCAL | CREAD;
    //设置字符大小
    newstate.c_cflag &= ~CSIZE;
    //设置停止位1
    newstate.c_cflag &= ~CSTOPB;
    //设置数据位8位
    newstate.c_cflag |= CS8;
    //设置无奇偶校验位，N
    newstate.c_cflag &= ~PARENB;

    /*阻塞模式的设置*/
    newstate.c_cc[VTIME]=0;
    newstate.c_cc[VMIN]=1;

    tcsetattr(fd, TCSANOW, &newstate);
}

void SerialPort::RMSerialWrite(int x,int y,int SendDataFlag)
/**
 *@brief: RM串口发送格式化函数
 *
 * @param: x 坐标的ｘ值
 * @param: y 坐标的ｙ值
 * @param: SendDataFlag 发送的标志
 *
 * @authors: Rcxxx
 *           Hzkkk
 */
{
    switch (SendDataFlag)
    {
    case 0:
    {
        sprintf(g_buf,"%s%03d%s%03d","S",x,",",y);
        std::cout<<std::endl<<g_buf<<std::endl;
        write(fd,g_buf,sizeof(g_buf));
        sleepUS(1);
    }
        break;
    case 1:
    {
        sprintf(g_buf,"%s","N1000000");
        std::cout<<std::endl<<g_buf<<std::endl;
        write(fd,g_buf,sizeof(g_buf));
        sleepUS(1);
    }
        break;
    case 2:
    {
        sprintf(g_buf,"%s%03d%s%03d","S",x,",",y);
        std::cout<<std::endl<<g_buf<<std::endl;
        write(fd,g_buf,sizeof(g_buf));
        sleepUS(1);
    }
        break;
    default:
        break;
    }
}

void SerialPort::sleepUS(unsigned int secs)
/**
 * @brief: 串口发送延时函数
 * @param: secs 延时时间　微秒为单位
  */
{
    struct timeval tval;
    tval.tv_sec=secs/1000;
    tval.tv_usec=(secs*1000)%1000000;
    select(0,NULL,NULL,NULL,&tval);
}
