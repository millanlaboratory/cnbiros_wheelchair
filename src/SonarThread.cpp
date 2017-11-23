#ifndef CNBIROS_WHEELCHAIR_SONAR_THREAD_CPP
#define CNBIROS_WHEELCHAIR_SONAR_THREAD_CPP

#include "cnbiros_wheelchair/SonarThread.hpp"

namespace cnbiros {
	namespace wheelchair {

SonarThread::SonarThread(const char* port, int n_sonar, const unsigned char* address_book, const unsigned char* max_gains)
{
 	//printf("SRT: CALLED %d\n", (int) address_book[0]);
	
	this->N_SONAR = n_sonar;
	this->sr = new Sonar(port, n_sonar, address_book, max_gains);

	for (int i = 0; i < N_SONAR; i++){
		this->readings[i] = 0;
	}

	pthread_mutex_init(&this->mtx, NULL);
	this->nchanged = 0;
	this->startThread();
}

SonarThread::~SonarThread()
{
	pthread_mutex_destroy(&this->mtx);
}

void SonarThread::startThread()
{
	pthread_mutex_lock(&this->mtx);
	assert(!this->run);
	this->run = true;

	pthread_create(&this->srthread, NULL, runThread, this);
	pthread_mutex_unlock(&this->mtx);
}

void SonarThread::shutdownThread()
{
	std::cout << "Killing SonarThread" << std::endl;

	// notify the thread to stop
	pthread_mutex_lock(&this->mtx);
	this->run = false;
	pthread_mutex_unlock(&this->mtx);

	// Wait the thread to stop
	pthread_join(this->srthread, NULL);
}


void* SonarThread::runThread(void* data)
{
	int i, n;
	bool bquit = false;
	SonarThread* srth = (SonarThread*)data;
	int N_SONAR = srth->N_SONAR;
	int range[N_SONAR];

	while (!bquit) {
		// Read current values from sensors
 		n = srth->sr->getSonarReadings(range);

		// Synchronize with out of the thread
		pthread_mutex_lock(&srth->mtx);
		srth->nchanged = n;
		for (i=0; i<N_SONAR; i++)
			srth->readings[i] = range[i];

		bquit = !srth->run;
		pthread_mutex_unlock(&srth->mtx);
		//usleep(2000);
	}

	return NULL;
}


int SonarThread::getReadings(int range[])
{
	static int lchanged = 0;
	int i, n, ret;

	pthread_mutex_lock(&this->mtx);
	n = this->nchanged;
	for (i=0; i<N_SONAR; i++)
		range[i] = this->readings[i];
	pthread_mutex_unlock(&this->mtx);

	ret = (lchanged == 0 && n == 0) ? -1 : 0;
	lchanged = n;

	/* Return -1 if the previous two readings were identical ( this does
			not necessarily indicate an error occurred, but it may have done )*/
	return ret;
}

int SonarThread::setAnalogueGain(unsigned char addr, unsigned char gain) {
	
	int retcode;
	pthread_mutex_lock(&this->mtx);
	retcode = this->sr->setAnalogueGain(addr, gain);
	pthread_mutex_unlock(&this->mtx);

	return retcode;
}
	

	}
}

#endif
