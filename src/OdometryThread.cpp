#ifndef CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_CPP
#define CNBIROS_WHEELCHAIR_ODOMETRY_THREAD_CPP

#include "cnbiros_wheelchair/OdometryThread.hpp"

namespace cnbiros {
	namespace wheelchair {

OdometryThread::OdometryThread(char* port)
{
	//this->enc = new CNBIWheelEncoder(port);

	{
		boost::mutex::scoped_lock lock(this->mtx);
		this->delta = 0;
	}

	this->startThread();
}

OdometryThread::~OdometryThread()
{
}

void OdometryThread::startThread()
{
	assert(!this->encthread);
	this->odothread = boost::shared_ptr<boost::thread> (new boost::thread(boost::bind( &OdometryThread::runThread, this)));
	{
		boost::mutex::scoped_lock lock(this->mtx);
		this->quit = false;
		this->run = true;
	}
	this->cond.notify_one();
}

void OdometryThread::shutdownThread()
{
	{
		boost::mutex::scoped_lock lock(this->mtx);
		this->quit = true;
		this->run = true;
	}
	this->cond.notify_one();
	this->odothread->join();
}

void OdometryThread::runThread()
{
	
	for(;;){
		{
			boost::mutex::scoped_lock lock(this->mtx);
			while(!this->run){
				this->cond.wait(lock);
			}
			if(this->quit == true){
				break;
			}
		}

		// your stuff goes here...



	}

	
}

int OdometryThread::getOdometry(double *x, double *y, double *theta)
{
	*x = 0.0;
	*y = 0.0;
	*theta = 0.0;
	
	return 0;
}

	}
}

#endif

