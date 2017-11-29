#ifndef CNBIROS_WHEELCHAIR_ENCODER_THREAD_CPP
#define CNBIROS_WHEELCHAIR_ENCODER_THREAD_CPP

#include "cnbiros_wheelchair/EncoderThread.hpp"

namespace cnbiros {
	namespace wheelchair {

EncoderThread::EncoderThread(const std::string& port)
{
	this->enc = new Encoder(port);

	pthread_mutex_init(&this->mtx, NULL);
	this->delta = 0;
	this->counter = 0;
	this->seq = 0;
		
	
	this->startThread();
}

EncoderThread::~EncoderThread()
{
	pthread_mutex_destroy(&this->mtx);
	
}

void EncoderThread::startThread()
{
	pthread_mutex_lock(&this->mtx);
	assert(!this->run);

	this->run = true;
	pthread_create(&this->encthread, NULL, runThread, this);
	printf("Started EncoderThread\n");
	pthread_mutex_unlock(&this->mtx);


}

void EncoderThread::shutdownThread()
{
	printf("Killing EncoderThread");
	pthread_mutex_lock(&this->mtx);
	this->run = false;
	pthread_mutex_unlock(&this->mtx);

	pthread_join(this->encthread, NULL);
}

void* EncoderThread::runThread(void* data)
{
	EncoderThread* encth = (EncoderThread*)data;
	int d = 0;
	bool bquit = false;

	
	while (!bquit) {
		// Read the wheel encoder
		d = encth->enc->readEncoder();
		
		//HACK to supress spurious readings
 		if ((d > 60) || (d < -60))
			d = 0;
		
		pthread_mutex_lock(&encth->mtx);
		encth->delta += d;
		encth->seq++;
		bquit = !encth->run;
		pthread_mutex_lock(&encth->mtx);
	}

	return NULL;
}

/* Return the "delta" distance travelled since this function was last called. */
double EncoderThread::getDelta(int &sequence)
{
	double ret = 0.0f;
	pthread_mutex_lock(&this->mtx);
	sequence = this->seq;
	pthread_mutex_unlock(&this->mtx);

	ret = this->getDelta();	

	return ret;
}

double EncoderThread::getDelta(void)
{
	double ret = 0.0;
	
	pthread_mutex_lock(&this->mtx);
	ret = this->delta;
	this->delta = 0.0;
	pthread_mutex_unlock(&this->mtx);
	
	return ret;
}

	}
}

#endif

