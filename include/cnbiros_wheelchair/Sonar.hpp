#ifndef CNBIROS_WHEELCHAIR_SONAR_HPP
#define CNBIROS_WHEELCHAIR_SONAR_HPP

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
#include <string>
#include <iostream>

const int MAX_SONAR = 5; // // MAX 16 per I2C bus

namespace cnbiros {
	namespace wheelchair {

class Sonar {
	public:
		/* use address_book if the sonars are not sequentially addressed from 0xE0
				e.g. address_book = {'0xE0', '0xF2', '0xE4' ...}
			 optionally use max_gains to change the usable range (also implicitly changes the beamwidth - 
			 must be in the rage 0x00 - 0x10 and there must be one per sonar
		*/
		Sonar(const char* port = "/dev/ttyUSB0", int n_sonar = MAX_SONAR, const unsigned char* addresses = NULL, const unsigned char* max_gains = NULL);
		~Sonar();

		int getRange();
		int getRevision();
		int requestRegisterValues(unsigned char addr, bool flushbuf = false);
		int requestRangeCm(unsigned char addr);
		int setAnalogueGain(unsigned char addr, unsigned char gain);
		int getSonarReadings(int range[]);
		int changeAddress(unsigned char old_addr, unsigned char new_addr);
		
		int getN_SONAR();

		int closePort(void);

	private:
		int fd;
		int setupPort(const char* port);
		int N_SONAR;
		unsigned char address_book[MAX_SONAR];// = {0xe0, 0xe2, 0xe4, 0xe6};		

};

	}
}
#endif
