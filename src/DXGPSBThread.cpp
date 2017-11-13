#ifndef CNBIROS_WHEELCHAIR_DXGPSB_THREAD_CPP
#define CNBIROS_WHEELCHAIR_DXGPSB_THREAD_CPP

#include <iostream>
#include <assert.h>
#include "cnbiros_wheelchair/DXGPSBThread.hpp"

namespace cnbiros {
	namespace wheelchair {


DXGPSBThread::DXGPSBThread(const std::string& port) {
 	std::cout << "DXGPSBRT: CALLED" << std::endl;

	try {
		this->dxgpsb = new DXGPSB(port);
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}
	
	pthread_mutex_init(&this->mtx, NULL);
	this->startThread();
}

DXGPSBThread::~DXGPSBThread(void) {
	pthread_mutex_destroy(&this->mtx);
}

void DXGPSBThread::startThread() {
	pthread_mutex_lock(&this->mtx);
	assert(!this->run);
	this->run = true;

	pthread_create(&this->dxthread, NULL, runThread, this);
	pthread_mutex_unlock(&this->mtx);
}

void DXGPSBThread::shutdownThread() {
	std::cout << "Killing DXGPSBThread" << std::endl;

	// notify the thread to stop
	pthread_mutex_lock(&this->mtx);
	this->run = false;
	pthread_mutex_unlock(&this->mtx);

	// Wait the thread to stop
	pthread_join(this->dxthread, NULL);
}


void* DXGPSBThread::runThread(void* data) {
	char v, w;
	bool bquit = false;
	DXGPSBThread* dxth = (DXGPSBThread*)data;

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

int DXGPSBThread::setVelocities(float v, float w) {
	pthread_mutex_lock(&this->mtx);
		this->v_tx = (char)(v * 127.0 + 128.0);
		this->w_tx = (char)(w * 127.0 + 128.0);
	pthread_mutex_unlock(&this->mtx);

	return 0;
}

int DXGPSBThread::setVelocities(char v, char w) {

	pthread_mutex_lock(&this->mtx);
		this->v_tx = v;
		this->w_tx = w;
	pthread_mutex_unlock(&this->mtx);
	return 0;
}

int DXGPSBThread::getVelocities(char &v, char &w) {
	pthread_mutex_lock(&this->mtx);
		v = this->v_rx;
		w = this->w_rx;
	pthread_mutex_unlock(&this->mtx);
	
	return 0;
}

int DXGPSBThread::getVelocities(float &v, float &w) {
	pthread_mutex_lock(&this->mtx);
		v = (this->v_rx - 128) / 127.0;
		w = (this->w_rx - 128) / 128.0;
	pthread_mutex_unlock(&this->mtx);
	
	return 0;
}

	}
}

#endif
