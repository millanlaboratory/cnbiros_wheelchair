#ifndef CNBIROS_WHEELCHAIR_DXGPSB_THREAD_HPP
#define CNBIROS_WHEELCHAIR_DXGPSB_THREAD_HPP

#include <pthread.h>
#include <string>
#include "cnbiros_wheelchair/Dxgpsb.hpp"

namespace cnbiros {
	namespace wheelchair {

class DxgpsbThread {

	public:
		DxgpsbThread(const std::string& port = "/dev/ttyUSB0");
		~DxgpsbThread(void);

		int setVelocities(float v, float w);	// v, w E [-1, 1]
		int setVelocities(char v, char w);		//0x01 = -MAX, 0x80 = 0, 0xFF = +MAX
		int getVelocities(float &v, float &w);
		int getVelocities(char &v, char &w);

		void stopThread();
		void startThread();

	protected:
		Dxgpsb* dxgpsb;
		
	private:	
		/* Thread stuff */
		static void* runThread(void* data);
		bool run;
		pthread_t dxthread;
		pthread_mutex_t mtx;
		
		char v_tx;
		char w_tx;
		char v_rx;
		char w_rx;
};

	}
}

#endif
