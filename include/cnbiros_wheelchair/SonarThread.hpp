#ifndef CNBIROS_WHEELCHAIR_SONAR_THREAD_HPP
#define CNBIROS_WHEELCHAIR_SONAR_THREAD_HPP

#include <pthread.h>
#include <iostream>
#include <assert.h>

#include "cnbiros_wheelchair/Sonar.hpp"

namespace cnbiros {
	namespace wheelchair {

class SonarThread {

	public:
		/* use address_book if the sonars are not sequentially addressed from 0xE0
				e.g. address_book = {'0xE0', '0xF2', '0xE4' ...}
			 optionally use max_gains to change the usable range (also implicitly changes the beamwidth - 
			 must be in the rage 0x00 - 0x10 and there must be one per sonar
		*/
		SonarThread(const char* port = "/dev/ttyUSB0", int n_sonar = MAX_SONAR, const unsigned char* address_book = NULL, const unsigned char* max_gains = NULL);
		~SonarThread();

		int getReadings(int range[]);

		void startThread();
		void shutdownThread();

	private:	
	/* Thread stuff */
		static void* runThread(void* data);
		bool run;
		pthread_t srthread;
		pthread_mutex_t mtx;

 		Sonar* sr;
		int readings[MAX_SONAR];
		int N_SONAR;
		int nchanged;
};

	}
}

#endif
