#ifndef CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_HPP
#define CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_HPP

#include <pthread.h>
#include <iostream>
#include <assert.h>
#include <math.h>

#include "cnbiros_wheelchair/EncoderThread.hpp"

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 180 / M_PI)
#endif

namespace cnbiros {
	namespace wheelchair {

class OdometryThread {

	public:
		OdometryThread(const std::string& lport, const std::string& rport, 
					   double axle_width, double delta, bool invert_left_wheel = true);
		~OdometryThread();

		/* Manage the thread start/stop */
		void startThread();
		void shutdownThread();
		
		/* Interaction functions */
		int getOdometry(double *x, double *y, double *theta);
		int setOdometry(double x, double y, double theta);
		int shiftOdometry(double x, double y);
		int resetOdometry();
		

	private:	
		/* Thread stuff */
		static void* runThread(void*);
		bool run;
		pthread_t odothread;
		pthread_mutex_t mtx;

		/* Encoder stuff */
		EncoderThread *enc_right;
		EncoderThread *enc_left;
		
		double x, y, theta;			/* Estimated position from odometry data */
		double axle, resolution;	/* axle = axle width, resolution = delta */
		
		bool invert;
		
};

	}
}
#endif
