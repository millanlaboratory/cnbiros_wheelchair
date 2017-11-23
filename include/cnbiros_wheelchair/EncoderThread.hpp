#ifndef CNBIROS_WHEELCHAIR_ENCODER_THREAD_HPP
#define CNBIROS_WHEELCHAIR_ENCODER_THREAD_HPP

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "Encoder.hpp"

namespace cnbiros {
	namespace wheelchair {

class EncoderThread /* : public Encoder */ {

	public:
		EncoderThread(const std::string& port);
		~EncoderThread();

		double getDelta();

		void startThread();
		void shutdownThread();

	private:	
	/* Thread stuff */
		void runThread();
		volatile bool run;
		volatile bool quit;
		boost::shared_ptr<boost::thread> encthread;
		boost::mutex mtx;
		boost::condition cond;

		Encoder *enc;
		double delta;
		
};

	}
}

#endif
