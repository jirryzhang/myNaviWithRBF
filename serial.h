/*
 * serial.h
 *
 *  Created on: Nov 8, 2016
 *      Author: root
 */

#ifndef SERIAL_H_
#define SERIAL_H_
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

class serial {
public:
	serial();
	virtual ~serial();
    int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
    int open_port(int fd,int comport);
    int Recv(int fd, char *recv_buf,int data_len);
    int Send(int fd, char *send_buf,int data_len);
    void Close(int fd);
//	int BCDToInt(unsigned char m);
    double buffToDouble(char *buff);
};

#endif /* SERIAL_H_ */
