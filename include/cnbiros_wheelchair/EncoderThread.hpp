#ifndef CNBIROS_WHEELCHAIR_ENCODER_THREAD_HPP
#define CNBIROS_WHEELCHAIR_ENCODER_THREAD_HPP

#include "Encoder.hpp"

namespace cnbiros {
	namespace wheelchair {

class EncoderThread  {

	public:
		EncoderThread(const std::string& port);
		~EncoderThread();

		/* Manage the start/stop of the thread */
		void startThread();
		void shutdownThread();
		
		/* Return the "delta" distance travelled since this function was last called. */
		double getDelta(int &sequence);
		double getDelta(void);

	private:	
		/* Thread stuff */
		static void* runThread(void*);
		bool run;
		pthread_t encthread;
		pthread_mutex_t mtx;

		/* Encoder stuff */
		Encoder *enc;
		double delta;
		int counter;
		int seq;
		
};

	}
}

#endif
