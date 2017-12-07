#ifndef CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_HPP
#define CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_HPP

#include <pthread.h>
#include <iostream>
#include <assert.h>
#include <math.h>
#include <chrono>

#include "cnbiros_wheelchair/EncoderThread.hpp"

#define CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_SLEEP	20000	// [us] Wait period in the main
															//		thread cycle

namespace cnbiros {
	namespace wheelchair {

class OdometryThread {

	public:
		OdometryThread(const std::string& lport, const std::string& rport, 
					   double axle, double diameter, int revolution, 
					   bool invert_left_wheel = true);
		virtual ~OdometryThread(void);

		
		/* Interaction functions */
		int getOdometry(double *x, double *y, double *theta,
						double *vx = nullptr, double *vy = nullptr, 
						double *vth = nullptr);
		int setOdometry(double x, double y, double theta);
		int shiftOdometry(double x, double y);
		int resetOdometry(void);
	
	private:
		/* Manage the thread start/stop */
		void startThread(void);

	private:	
		/* Thread stuff */
		static void* runThread(void*);
		bool run;
		pthread_t odothread;
		pthread_mutex_t mtx;

		/* Encoder stuff */
		EncoderThread *enc_right;
		EncoderThread *enc_left;
		
		/* Estimated position from odometry data */
		double x;
		double y; 
		double theta;			

		/* Estimated velocity from odometry data */
		double vx;
		double vy;
		double vtheta;

		/* Wheelchair parameters */
		bool	invert;			// Invert counts read by left encoder
		double	axle;			// Distance between wheels [m]
		double	diameter;		// Diameter of the wheel [m]
		int		revolution;		// Number of thicks for one wheel revolution
		double	DistancePerCount;

		
};

	}
}
#endif
