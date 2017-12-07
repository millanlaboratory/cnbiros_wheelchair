#ifndef CNBIROS_WHEELCHAIR_DXGPSB_THREAD_CPP
#define CNBIROS_WHEELCHAIR_DXGPSB_THREAD_CPP

#include <iostream>
#include <assert.h>
#include "cnbiros_wheelchair/DxgpsbThread.hpp"

namespace cnbiros {
	namespace wheelchair {


DxgpsbThread::DxgpsbThread(const std::string& port) {
 	std::cout << "DxgpsbRT: CALLED" << std::endl;

	try {
		this->dxgpsb = new Dxgpsb(port);
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}
	
	pthread_mutex_init(&this->mtx, NULL);
	this->startThread();
}

DxgpsbThread::~DxgpsbThread(void) {
	
	printf("Killing DxgpsbThread\n");

	// notify the thread to stop
	pthread_mutex_lock(&this->mtx);
	this->run = false;
	pthread_mutex_unlock(&this->mtx);
	
	// close port and delete encoder object
	delete this->dxgpsb;
	
	// Wait the thread to stop
	pthread_join(this->dxthread, NULL);
	
	// Destroy the mutex
	pthread_mutex_destroy(&this->mtx);
}

void DxgpsbThread::startThread() {
	pthread_mutex_lock(&this->mtx);
	assert(!this->run);
	this->run = true;

	pthread_create(&this->dxthread, NULL, runThread, this);
	pthread_mutex_unlock(&this->mtx);
}

void* DxgpsbThread::runThread(void* data) {
	char v, w;
	bool bquit = false;
	DxgpsbThread* dxth = (DxgpsbThread*)data;

	while (!bquit) {
		// Read current values from sensors
		dxth->dxgpsb->readVelocities();
		dxth->dxgpsb->getVelocities(v, w);

		pthread_mutex_lock(&dxth->mtx);
			dxth->v_rx = v;
			dxth->w_rx = w;
			v = dxth->v_tx;
			w = dxth->w_tx;
		pthread_mutex_unlock(&dxth->mtx);

		dxth->dxgpsb->setVelocities(v, w);
		dxth->dxgpsb->sendVelocities();
		
		// Synchronize with out of the thread
		pthread_mutex_lock(&dxth->mtx);

		bquit = !dxth->run;
		pthread_mutex_unlock(&dxth->mtx);
		//usleep(2000);
	}

	return NULL;
}

int DxgpsbThread::setVelocities(float v, float w) {
	pthread_mutex_lock(&this->mtx);
		this->v_tx = (char)(v * 127.0 + 128.0);
		this->w_tx = (char)(w * 127.0 + 128.0);
	pthread_mutex_unlock(&this->mtx);

	return 0;
}

int DxgpsbThread::setVelocities(char v, char w) {

	pthread_mutex_lock(&this->mtx);
		this->v_tx = v;
		this->w_tx = w;
	pthread_mutex_unlock(&this->mtx);
	return 0;
}

int DxgpsbThread::getVelocities(char &v, char &w) {
	pthread_mutex_lock(&this->mtx);
		v = this->v_rx;
		w = this->w_rx;
	pthread_mutex_unlock(&this->mtx);
	
	return 0;
}

int DxgpsbThread::getVelocities(float &v, float &w) {
	pthread_mutex_lock(&this->mtx);
		v = (this->v_rx - 128) / 127.0;
		w = (this->w_rx - 128) / 128.0;
	pthread_mutex_unlock(&this->mtx);
	
	return 0;
}

	}
}

#endif
