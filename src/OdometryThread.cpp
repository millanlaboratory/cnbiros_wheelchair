#ifndef CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_CPP
#define CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_CPP


#include "cnbiros_wheelchair/OdometryThread.hpp"

namespace cnbiros {
	namespace wheelchair {

OdometryThread::OdometryThread(const std::string& lport, const std::string& rport, 
							   double axle_width, double delta, bool invert_left_wheel) 
{
	this->enc_left = new EncoderThread(lport);
	this->enc_right = new EncoderThread(rport);
		
	this->x = 0;
	this->y = 0;
	this->theta = 0;
	
	this->axle = axle_width;
	this->resolution = delta;
	
	this->invert = invert_left_wheel;
	pthread_mutex_init(&this->mtx, NULL);

	this->startThread();
}

OdometryThread::~OdometryThread()
{
	this->shutdownThread();
	pthread_mutex_destroy(&this->mtx);
}

void OdometryThread::startThread()
{
	pthread_mutex_lock(&this->mtx);
	assert(!this->run);

	this->run = true;
	pthread_create(&this->odothread, NULL, runThread, this);
	pthread_mutex_unlock(&this->mtx);
}

void OdometryThread::shutdownThread()
{
	std::cout << "Killing OdometryThread" << std::endl;

	// Notify the thread to stop			
	pthread_mutex_lock(&this->mtx);
	this->run = false;
	pthread_mutex_unlock(&this->mtx);

	// Wait for the thread completion
	pthread_join(this->odothread, NULL);
}

void* OdometryThread::runThread(void* data)
{
	OdometryThread* odoth = (OdometryThread*)data;
	double dl = 0, dr = 0;
	int lseq = 0, rseq = 0;
	double b = odoth->axle;
	double d = odoth->resolution;
	
	double dx = 0, dy = 0, dtheta = 0;
	double ltheta, sltheta, cltheta;
	
	double mag = 0.0;
	bool bquit = false;
	
	
	while (1) {
		pthread_mutex_lock(&odoth->mtx);
		ltheta = odoth->theta;
		bquit = !odoth->run;
		pthread_mutex_unlock(&odoth->mtx);
		if (bquit)
			break;
		
 		dl = d * odoth->enc_left->getDelta(lseq);
 		dr = d * odoth->enc_right->getDelta(rseq);
		if (odoth->invert){
			dl *= -1;
		}
		//printf("Delta (left, right) ( % 3.0f, % 3.0f )\n", dl, dr);
		//printf("Delta (left, right, lseq, rseq) ( % 3.0f, % 3.0f, %d, %d)\n", dl, dr, lseq, rseq);
		
		dtheta = (dl - dr ) / b;
		sltheta = sin(ltheta);
		cltheta = cos(ltheta);
		
		if (dl == dr){
			// Heading straight
			dx = dl * cltheta;
			dy = dl * sltheta;
		} else {
			// Following a curve
			mag = b * (dr + dl) / (2 * (dr - dl));
			dx = mag * (sin(((dr - dl) / b) + ltheta) - sltheta);
			dy = mag * (cos(((dr - dl) / b) - ltheta) - cltheta);
		}
		
		pthread_mutex_lock(&odoth->mtx);
		odoth->x += dx;
		odoth->y -= dy;
		odoth->theta += dtheta;
		//printf("DEBUG: Position (x, y, theta)( % 4.1f, % 4.1f, % 4.1f )\n", odoth->x, odoth->y, RAD2DEG(odoth->theta));
		pthread_mutex_unlock(&odoth->mtx);
		//printf("%4f\n", enc_thread1.getDelta());
		
		usleep(20000);
	}
}


int OdometryThread::getOdometry(double *x, double *y, double *theta)
{
	pthread_mutex_lock(&this->mtx);
	*x = this->x;
	*y = this->y;
	*theta = this->theta;
	//printf("Position (x, y, theta)( % 4.1f, % 4.1f, % 4.1f )\n", this->x, this->y, RAD2DEG(this->theta));
	pthread_mutex_unlock(&this->mtx);
	
	return 0;
}

int OdometryThread::setOdometry(double x, double y, double theta)
{
	pthread_mutex_lock(&this->mtx);
	this->x = x;
	this->y = y;
	this->theta = theta;
	pthread_mutex_unlock(&this->mtx);
	
	return 0;
}

int OdometryThread::shiftOdometry(double x, double y)
{
	pthread_mutex_lock(&this->mtx);
	this->x += x;
	this->y += y;
	pthread_mutex_unlock(&this->mtx);
	
	return 0;
}


int OdometryThread::resetOdometry()
{
	this->setOdometry(0.0, 0.0, 0.0);
	
	return 0;
	
}


	}
}
#endif
