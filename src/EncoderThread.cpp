#ifndef CNBIROS_WHEELCHAIR_ENCODER_THREAD_CPP
#define CNBIROS_WHEELCHAIR_ENCODER_THREAD_CPP

#include "cnbiros_wheelchair/EncoderThread.hpp"

namespace cnbiros {
	namespace wheelchair {

EncoderThread::EncoderThread(const std::string& port)
{
	try {
		this->enc = new Encoder(port);
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}

	this->delta = 0;
	this->counter = 0;
	this->seq = 0;
		
	pthread_mutex_init(&this->mtx, NULL);
	this->startThread();
}

EncoderThread::~EncoderThread()
{
	// Notify the thread to stop	
	pthread_mutex_lock(&this->mtx);
	this->run = false;
	pthread_mutex_unlock(&this->mtx);

	// close port and delete encoder object
	delete this->enc;
	
	// Wait the thread to join
	pthread_join(this->encthread, NULL);

	// Destroy the mutex
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

void* EncoderThread::runThread(void* data)
{
	EncoderThread* encth = (EncoderThread*)data;
	int d = 0;
	bool bquit = false;

	
	while (!bquit) {
		// Read the wheel encoder
		d = encth->enc->readEncoder();
		
		//HACK to supress spurious readings
 		if ((d > 100) || (d < -100))
			d = 0;
		
		pthread_mutex_lock(&encth->mtx);
		encth->delta += d;
		encth->seq++;
		bquit = !encth->run;
		pthread_mutex_unlock(&encth->mtx);
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

