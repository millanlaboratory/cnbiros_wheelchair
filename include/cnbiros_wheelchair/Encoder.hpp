#ifndef CNBIROS_WHEELCHAIR_ENCODER_HPP
#define CNBIROS_WHEELCHAIR_ENCODER_HPP

#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <cassert>

namespace cnbiros {
	namespace wheelchair {

class Encoder {
	public:
		Encoder(const std::string& port, bool debug_msgs = false);
		~Encoder();
		
		int readEncoder(); // This is a blocking function!

		int flushEncoder();
		int ClosePort();
	
	private:
		/* Internal states */
		int fd;
		//char encport[1024];
		std::string encport;
		int dist, ldist, deltadist;
		bool verbose;
		bool first_time;
		
		/* Setup/configuration functions */
		int setupPort(const char* port);
		int setupEncoder();
		int toggleVelocityOutput();
		int toggleDistanceOutput();
		
		
		/* test functions */
		int getEncoderVersion();
		int testEncoder();
		
		void debugOut(const char* str);
		
	/* OLD Implementation */		
	/*
	private:
		int fd;
		int dist, ldist, deltadist;
		bool verbose;
		bool first_time;
		
		
		int setupPort(const char* port);
		int setupEncoder();
		int toggleVelocityOutput();
		int toggleDistanceOutput();
		int flushEncoder();
		int getEncoderVersion();
		int testEncoder();
	*/			
};

	}
}
#endif
