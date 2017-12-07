#ifndef CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_CPP
#define CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_CPP


#include "cnbiros_wheelchair/OdometryThread.hpp"

namespace cnbiros {
	namespace wheelchair {

OdometryThread::OdometryThread(const std::string& lport, const std::string& rport, 
							   double axle, double diameter, int revolution, 
							   bool invert_left_wheel) 
{
	try {
		this->enc_left = new EncoderThread(lport);
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}
	try {
		this->enc_right = new EncoderThread(rport);
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}
		
	this->x = 0.0f;
	this->y = 0.0f;
	this->theta = 0.0f;
	
	this->axle			= axle;
	this->diameter		= diameter;
	this->revolution	= revolution;
	this->invert		= invert_left_wheel;

	/* 2*PI*radius / NumberOfCountsInARevolution */
	this->DistancePerCount = (2.0f * M_PI * (this->diameter / 2.0f)) / (double)this->revolution;

	pthread_mutex_init(&this->mtx, NULL);
	this->startThread();
}

OdometryThread::~OdometryThread(void)
{
	printf("Killing OdometryThread\n");

	printf("Stopping encoder thread left\n");
	this->enc_left->stopThread();
	printf("Stopping encoder thread right\n");
	this->enc_right->stopThread();

	// Delete the two encoders (the related thread will stop and join)
	delete this->enc_left;
	delete this->enc_right;

	// Notify the thread to stop			
	pthread_mutex_lock(&this->mtx);
	this->run = false;
	pthread_mutex_unlock(&this->mtx);

	// Wait for the thread completion
	pthread_join(this->odothread, NULL);

	// Destroy the mutex
	pthread_mutex_destroy(&this->mtx);
}

void OdometryThread::startThread(void)
{
	pthread_mutex_lock(&this->mtx);
	assert(!this->run);

	this->run = true;
	pthread_create(&this->odothread, NULL, runThread, this);
	pthread_mutex_unlock(&this->mtx);
}

void* OdometryThread::runThread(void* data)
{
	OdometryThread* odoth = (OdometryThread*)data;
	bool bquit = false;

	double deltaLeft  = 0.0f, deltaRight = 0.0f;
	double velLeft, velRight;
	double velx, vely, velth;
	double deltax, deltay, deltath;

	double DistancePerCount = odoth->DistancePerCount;
	double axle = odoth->axle;
	double theta = 0.0f;

	std::chrono::high_resolution_clock::time_point ctime;
	std::chrono::high_resolution_clock::time_point ptime;
	long int	elapsedms;
	double		deltaT;

	ptime = std::chrono::high_resolution_clock::now();

	odoth->resetOdometry();
	
	while (1) {

		// Get current elapsed time
		ctime		= std::chrono::high_resolution_clock::now();
		elapsedms	= std::chrono::duration_cast<std::chrono::milliseconds>(ctime-ptime).count();
		deltaT		= (double)elapsedms / 1000.0f;
		ptime		= ctime;

		// If not time elapsed, wait and continue the cycle
		if(deltaT == 0.0f) {
			usleep(CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_SLEEP);
			continue;
		}

		// Thread running check
		pthread_mutex_lock(&odoth->mtx);
		bquit = !odoth->run;
		pthread_mutex_unlock(&odoth->mtx);
		
		if (bquit)
			break;
	
		// Get current encoders readings
		deltaLeft  = odoth->enc_left->getDelta();
		deltaRight = odoth->enc_right->getDelta();

		// If both encoders return 0, don't update the odometry
		if ((deltaLeft == 0.0f) && (deltaRight == 0.0f)) {
			usleep(CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_SLEEP);
			continue;
		}

		// Invert encoder count for the left wheel, if required
		if (odoth->invert) 
			deltaLeft *= -1.0f;
		
		// Get wheel velocities - DeltaCount*DistancePerCount/Time
		velLeft  = (deltaLeft*DistancePerCount)  / deltaT;
		velRight = (deltaRight*DistancePerCount) / deltaT;

		// Transform wheel velocity in world coordinates (only velocity along x
		// is allowed - not holonomic device)
		velx  = ((velRight + velLeft) / 2.0f);
		vely  = 0.0f;							
		velth = (velRight - velLeft) / axle;
		
		// Get current theta
		pthread_mutex_lock(&odoth->mtx);
		theta = odoth->theta;
		pthread_mutex_unlock(&odoth->mtx);
		
		// Transform wheel counts in delta distances
		deltax  = (velx * cos(theta)) * deltaT;
		deltay  = (velx * sin(theta)) * deltaT;
		deltath = velth * deltaT;
		
		// Update x, y, theta, vx, vy, vtheta
		pthread_mutex_lock(&odoth->mtx);
		odoth->x		= odoth->x + deltax;
		odoth->y		= odoth->y - deltay;
		odoth->theta	= odoth->theta - deltath;
		odoth->vx		= velx;
		odoth->vy		= vely;
		odoth->vtheta	= velth;
		pthread_mutex_unlock(&odoth->mtx);

		usleep(CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_SLEEP);
	}
}

int OdometryThread::getOdometry(double *x, double *y, double *theta,
								double *vx, double *vy, double *vtheta)
{
	pthread_mutex_lock(&this->mtx);
	*x		= this->x;
	*y		= this->y;
	*theta	= this->theta;

	if(vx != nullptr)
		*vx = this->vx;

	if(vy != nullptr)
		*vy = this->vy;
	
	if(vtheta != nullptr)
		*vtheta = this->vtheta;
	
	pthread_mutex_unlock(&this->mtx);
	
	return 0;
}

int OdometryThread::setOdometry(double x, double y, double theta)
{
	pthread_mutex_lock(&this->mtx);
	this->x			= x;
	this->y					= y;
	this->theta		= theta;
	this->vx		= 0.0f;
	this->vy		= 0.0f;
	this->vtheta	= 0.0f;
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


int OdometryThread::resetOdometry(void)
{
	this->setOdometry(0.0, 0.0, 0.0);
	
	return 0;
}


	}
}
#endif
