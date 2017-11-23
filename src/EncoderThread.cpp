#ifndef CNBIROS_WHEELCHAIR_ENCODER_THREAD_CPP
#define CNBIROS_WHEELCHAIR_ENCODER_THREAD_CPP

#include "cnbiros_wheelchair/EncoderThread.hpp"

namespace cnbiros {
	namespace wheelchair {

EncoderThread::EncoderThread(const std::string& port)
{
	this->enc = new Encoder(port);

	{
		boost::mutex::scoped_lock lock(this->mtx);
		this->delta = 0;
	}

	this->startThread();
}

EncoderThread::~EncoderThread()
{
}

void EncoderThread::startThread()
{
	assert(!this->encthread);
	this->encthread = boost::shared_ptr<boost::thread> (new boost::thread(boost::bind( &EncoderThread::runThread, this)));
	{
		boost::mutex::scoped_lock lock(this->mtx);
		this->quit = false;
		this->run = true;
	}
	this->cond.notify_one();
}

void EncoderThread::shutdownThread()
{
	{
		boost::mutex::scoped_lock lock(this->mtx);
		this->quit = true;
		this->run = true;
	}
	this->cond.notify_one();
	this->encthread->join();
}

void EncoderThread::runThread()
{
	int d = 0;
	
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
		d = this->enc->readEncoder();
		{
			boost::mutex::scoped_lock lock(this->mtx);
			this->delta += d;
		}
		


	}

	
}

double EncoderThread::getDelta()
{
	double ret = 0.0;
	
	{
		boost::mutex::scoped_lock lock(this->mtx);
		ret = this->delta;
		this->delta = 0.0;
	}
	
	return ret;
}

	}
}

#endif

