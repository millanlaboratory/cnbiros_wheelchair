#ifndef CNBI_ODOMETRY_THREAD
#define CNBI_ODOMETRY_THREAD

#include <pthread.h>
#include "CNBIWheelEncoderThread.h"

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 180 / M_PI)
#endif


class CNBIOdometryThread {

	public:
		CNBIOdometryThread(double axle_width, double delta, bool invert_left_wheel = true, const char* lport = "/dev/ttyUSB0", const char* rport = "/dev/ttyUSB1");
		~CNBIOdometryThread();

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
		CNBIWheelEncoderThread *enc_right;
		CNBIWheelEncoderThread *enc_left;
		
		double x, y, theta; /* Estimated position from odometry data */
		double axle, resolution; /* axle = axle width, resolution = delta */
		
		bool invert;
		
		
		
};
#endif
