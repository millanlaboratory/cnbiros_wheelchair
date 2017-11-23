#ifndef CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_HPP
#define CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_HPP

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "cnbiros_wheelchair/EncoderThread.hpp"

namespace cnbiros {
	namespace wheelchair {

class OdometryThread {

	public:
		OdometryThread(const std::string& lport = "/dev/ttyUSB0", 
					   const std::string& rport = "/dev/ttyUSB1");
		~OdometryThread();

		int getOdometry(double *x, double *y, double *theta);

		void startThread();
		void shutdownThread();

	private:	
	/* Thread stuff */
		void runThread();
		volatile bool run;
		volatile bool quit;
		boost::shared_ptr<boost::thread> odothread;
		boost::mutex mtx;
		boost::condition cond;

		EncoderThread *lenc;
		EncoderThread *renc;
		
		double x, y, theta;
		
};

	}
}
#endif
