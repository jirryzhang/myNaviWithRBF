/*
 * serial.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#include "serial.h"

serial::serial() {
    // TODO Auto-generated constructor stub

}

serial::~serial() {
    // TODO Auto-generated destructor stub
}
int serial::set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;

    if(tcgetattr(fd,&oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }

    bzero(&newtio, sizeof( newtio ) );
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~CSIZE;

    switch( nBits ){
		
    case 7: newtio.c_cflag |= CS7;
        break;
    case 8: newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent ) {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /* 波特率选择 */
    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 38400:
        cfsetispeed(&newtio, B38400);
        cfsetospeed(&newtio, B38400);
        break;
    case 57600:
        cfsetispeed(&newtio, B57600);
        cfsetospeed(&newtio, B57600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }

    if(nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |= CSTOPB;
    //set the c_lflag
    /* local modes settings */
    if ( nSpeed == 9600 ) {
        newtio.c_lflag &= ~(ICANON | ECHOE | ECHO | ISIG);
    }
    /* blue wait the data time */
    else if(nSpeed == 19200 ){
        newtio.c_lflag &= ~(ICANON | ECHOE | ECHO | ISIG);
        newtio.c_cc[VTIME] = 50;
        newtio.c_cc[VMIN] = 0;
    }
    else {
        newtio.c_lflag &= ~(ECHOE | ECHO | ISIG);
        newtio.c_lflag |= (ICANON | ECHOE);
//        newtio.c_cc[VTIME] = 0;
//        newtio.c_cc[VMIN] = 0;
    }

    //set the input
    newtio.c_oflag &= ~OPOST;
    newtio.c_oflag &= ~(ONLCR | OCRNL);

    newtio.c_iflag |= (ICRNL | INLCR);
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcflush(fd,TCIFLUSH);

    if((tcsetattr(fd,TCSANOW,&newtio)) !=0 )
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}
/* 打开串口的函数 */
int serial::open_port(int fd,int comport)
{
#if 0
    if(comport == 1) {
        fd = open( "/dev/ttyO1", O_RDWR|O_NOCTTY|O_NDELAY|O_NONBLOCK);
        if (-1 == fd) {
            perror("Can't Open Serial Port");
            return -1;
        }
        else
            printf("open ttyO1 .....\n");
    }
    else if(comport == 2) {
        fd = open( "/dev/ttyO2", O_RDWR|O_NOCTTY|O_NDELAY|O_NONBLOCK);
        if (-1 == fd) {
            perror("Can't Open Serial Port");
            return -1;
        }
        else
            printf("open ttyO2 .....\n");}
    else if (comport==3) {
        fd = open( "/dev/ttyO3", O_RDWR|O_NOCTTY|O_NDELAY|O_NONBLOCK);

        if (-1 == fd) {
            perror("Can't Open Serial Port");
            return -1;
        }
        else
            printf("open ttyO3 .....\n");
    }
    else if (comport==4) {
        fd = open( "/dev/ttyO4", O_RDWR|O_NOCTTY|O_NDELAY|O_NONBLOCK);
        if (-1 == fd) {
            perror("Can't Open Serial Port");
            return -1;
        }
        else
            printf("open ttyO4 .....\n");
    }
#endif
    /* serial function */
    char devstr[20] = "0";
    sprintf(devstr,"%s%d", "/dev/ttyO", comport);
    fd = open( devstr, O_RDWR|O_NOCTTY);
    if (-1 == fd) {
        perror("Can't Open Serial Port");
        return -1;
    }
    else
        printf("open %s.....\n", devstr);

    /* locale modes settings */
    int flags = fcntl(fd, F_GETFL, 0);

    if(comport != 3 ) {
        if ( (flags = fcntl(fd, F_SETFL, flags | O_NONBLOCK)) < 0) {
            perror("fcntl is failed!");
        }
    }

    /* this is block modes */
//    if (comport == 3) {
//        if ( (flags = fcntl(fd, F_SETFL, flags & ~O_NONBLOCK)) < 0) {
//            perror("fcntl is failed!");
//        }
//    }

    if(isatty(STDIN_FILENO)==0)
        printf("standard input is not a terminal device\n");
    else
        printf("isatty success!\n");
    printf("fd-open=%d\n",fd);
    return fd;
}
/* 关闭函数 */
void serial::Close(int fd) {
    close(fd);
}

/* 发送函数和接收函数 */
int serial::Send(int fd, char *send_buf,int data_len)
{
    int ret;
    ret = write(fd,send_buf,data_len);
    return ret;
}

int serial::Recv(int fd, char *recv_buf,int data_len) {
    int len;
    len = read(fd,recv_buf,data_len);
    return len;
}
#if 0
//int serial::BCDToInt(unsigned char m) {
//    int a=0,b=1;
//    unsigned char n;
//    while(m) {
//        n=0xF&m;
//        m=m>>4;
//        a=a+b*n;
//        b=b*10;
//    }
//    return a;
//}
//double serial::buffToDouble(char *buff) {
//    int a,b;
//    double c;
//    a=BCDToInt(buff[2]);
//    b=BCDToInt(buff[3]);
//    c=(a*100+b)/10.0;
//    return c;
//}
#endif
/* 485 read data to Double */
double serial::buffToDouble(char *buff) {
    unsigned char a, b;
    double c;
    a = buff[1];
    b = buff[2];
    c = (a*16*16 +b)/100.0;
    return c;
}
