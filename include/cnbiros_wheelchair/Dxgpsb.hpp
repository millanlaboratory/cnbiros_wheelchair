#ifndef CNBIROS_WHEELCHAIR_DXGPSB_HPP
#define CNBIROS_WHEELCHAIR_DXGPSB_HPP

#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string>

namespace cnbiros {
	namespace wheelchair {

class Dxgpsb {
	public:
		Dxgpsb(const std::string& port = "/dev/ttyUSB0");
		~Dxgpsb(void);
    
		int setVelocities(float v, float w);	// v, w E [-1, 1]
		int setVelocities(char v, char w);		//0x01 = -MAX, 0x80 = 0, 0xFF = +MAX
		int getVelocities(float &v, float &w);
		int getVelocities(char &v, char &w);
		int sendVelocities();
		int readVelocities();
		int update();
		void main();
		int closePort(void);
	
	private:
		int setupPort(const char* port);
    	int readDevice(unsigned char* return_data);
    	bool seekByte(char b, int search_limit);
    	int writeToDevice(int n_bytes, unsigned char* data);
		int writeToDeviceNOCHKSUM(int n_bytes, unsigned char* data);

	private:
		int fd;
		char v_tx;
		char w_tx;
		char v_rx;
		char w_rx;
};

	}
}

#endif
