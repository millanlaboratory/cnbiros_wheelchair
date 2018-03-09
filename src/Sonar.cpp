#ifndef CNBIROS_WHEELCHAIR_SONAR_CPP
#define CNBIROS_WHEELCHAIR_SONAR_CPP

#include "cnbiros_wheelchair/Sonar.hpp"

namespace cnbiros {
	namespace wheelchair {

int Sonar::getN_SONAR() {
	return this->N_SONAR;
}


Sonar::Sonar(const char* port, int n_sonar, 
						 const  unsigned char* addresses,
						 const unsigned char* max_gains)
{
	//printf("SR: %d\n", addresses[0]);
	
	this->N_SONAR = n_sonar;

	try {
		this->setupPort(port);
		printf("Created a Sonar object\n");
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}
	
	if (addresses != NULL){
		for (int i = 0; i < N_SONAR; i++){
			this->address_book[i] = addresses[i];
		}
	} else {
		for (int i = 0; i < N_SONAR; i++){
			this->address_book[i] = 0xe0 + (i * 2);
		}		
	}
	
	//for (int i = 0; i < N_SONAR; i++){
	//	if (max_gains != NULL) {
	//		//this->setAnalogueGain(this->address_book[i], max_gains[i]);
	//		this->setAnalogueGain(this->address_book[i], 0x04);
	//	} else {
	//		this->setAnalogueGain(this->address_book[i], 0x04);
	//	}
	//}
}

Sonar::~Sonar() {
	this->closePort();
}

int Sonar::closePort(void) {

	if(this->fd != -1) {
		tcflush( this->fd, TCIFLUSH );
		close(this->fd);
	}

	return 0;
}

int Sonar::getRevision()
{
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	unsigned short revision;
	unsigned char sbuf[8];
	

	for( totalread = 0, ndata = 8, nread = 0, tries = 0, maxtries = 10;
						totalread < ndata && tries < maxtries; tries++ ) {

		nread = read( this->fd, sbuf + totalread, ndata - totalread );

		if ( nread == -1 ) {
			myerrno = errno;
			//printf( "read() failed: %s\n", strerror( myerrno ) );
			return myerrno;
		}
		if ( nread == 0 ) {
			//printf( "read() failed, unexpected EOF\n" );
			return ENODATA;
		}
		totalread += nread;
	}
	if ( tries > maxtries && nread != ndata ) {
		//printf( "only read %d of %d bytes after %d tries\n", nread, ndata, tries );
		return ENODATA;
	}
	//printf( "sbuf = { %.2x, %.2x, %.2x, %.2x, %.2x, %.2x, %.2x, %.2x }\n", sbuf[0], sbuf[1], sbuf[2], sbuf[3], sbuf[4], sbuf[5], sbuf[6], sbuf[7] );
	revision = sbuf[1]; 
	//unused = sbuf[2];
	//range = 0 | sbuf[3] << 8 | sbuf[4];
	//range = 0 | sbuf[4] << 8 | sbuf[5]; // HACK should be [3] and [4], but this gets around the timing problem (otherwise you need to wit 20ms between sending 'range in cm' requests)
	//minimum = 0 | sbuf[5] << 8 | sbuf[6];
	
	return (int) revision;	
}


int Sonar::getRange()
{
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	unsigned short range;
	unsigned char sbuf[8];
	

	for( totalread = 0, ndata = 8, nread = 0, tries = 0, maxtries = 10;
						totalread < ndata && tries < maxtries; tries++ ) {

		nread = read( this->fd, sbuf + totalread, ndata - totalread );

		if ( nread == -1 ) {
			myerrno = errno;
			//printf( "read() failed: %s\n", strerror( myerrno ) );
			return myerrno;
		}
		if ( nread == 0 ) {
			//printf( "read() failed, unexpected EOF\n" );
			return ENODATA;
		}
		totalread += nread;
	}
	if ( tries > maxtries && nread != ndata ) {
		//printf( "only read %d of %d bytes after %d tries\n", nread, ndata, tries );
		return ENODATA;
	}
	//printf( "sbuf = { %.2x, %.2x, %.2x, %.2x, %.2x, %.2x, %.2x, %.2x }\n", sbuf[0], sbuf[1], sbuf[2], sbuf[3], sbuf[4], sbuf[5], sbuf[6], sbuf[7] );
	//revision = sbuf[1]; 
	//unused = sbuf[2];
	range = 0 | sbuf[3] << 8 | sbuf[4];
	//range = 0 | sbuf[4] << 8 | sbuf[5]; // HACK should be [3] and [4], but this gets around the timing problem (otherwise you need to wit 20ms between sending 'range in cm' requests)
	//minimum = 0 | sbuf[5] << 8 | sbuf[6];
	
	return (int) range;	
}

int Sonar::setupPort(const char* port)
{
	int myerrno;
	struct termios termInfo;
	
	this->fd = open( port, O_RDWR | O_NOCTTY | O_SYNC );
	if ( this->fd == -1 ) {
		this->closePort();
		myerrno = errno;
		throw std::runtime_error("Cannot open sonars at " + std::string(port) + ": " +  strerror(myerrno));
		return myerrno;
	}

	if ( ioctl( this->fd , TCGETA, & termInfo ) == -1 ) {
		this->closePort();
		myerrno = errno;
		throw std::runtime_error("ioctl(TCGETA) failed: " + std::string(strerror(myerrno)));
		return myerrno;
	}

	if( tcgetattr( this->fd, & termInfo ) == -1 ) {
		this->closePort();
		myerrno = errno;
		throw std::runtime_error("tcgetattr() failed: " + std::string(strerror(myerrno)));
		return myerrno;
	}

	cfsetispeed( & termInfo , B19200 );

	termInfo.c_cflag |=  ( CLOCAL | CREAD );
	termInfo.c_lflag &= ~( ICANON | ECHO | ISIG );
	termInfo.c_cflag &= ~  PARENB;
	termInfo.c_cflag &= ~  CSTOPB;
	termInfo.c_cflag &= ~  CSIZE;
	termInfo.c_cflag |=    CS8;
	termInfo.c_iflag  =    IGNBRK | IGNPAR | INLCR;

	tcflush( this->fd, TCIFLUSH );
	
	if ( tcsetattr( this->fd, TCSANOW, &termInfo) == -1 ) {
		this->closePort();
		myerrno = errno;
		throw std::runtime_error("tcsetattr() failed: " + std::string(strerror(myerrno)));
		return myerrno;
	}	
	
	return 0;
}

int Sonar::setAnalogueGain(unsigned char addr, unsigned char gain)
{
	unsigned char sbuf[8];
	int nwritten;
	
	sbuf[0] = 0x55;
	sbuf[1] = addr;
	sbuf[2] = 1; // Register 1 (analogue gain register)
	sbuf[3] = 1;
	sbuf[4] = gain;
	
	nwritten = write( this->fd, sbuf, 5 );
	
	tcflush( fd, TCIFLUSH );		

	if ( nwritten != 5 ) {
		throw std::runtime_error("Failed to set analogue gain to " + std::to_string(gain) + " for sonar " + std::to_string(addr));
		return -1;
	}
	
	return 0;
}

int Sonar::requestRangeCm(unsigned char addr)
{
	unsigned char sbuf[8];
	int nwritten;
	
	sbuf[0] = 0x55;
	sbuf[1] = addr;
	sbuf[2] = 0;
	sbuf[3] = 1;
	sbuf[4] = 0x51;
	
	nwritten = write( this->fd, sbuf, 5 );
	
	tcflush( fd, TCIFLUSH );		
	
	if ( nwritten != 5 ) {
		printf( "Failed to request 'range in cm' from %d\n", (int) addr );
		return -1;
	}
	
	return 0;
}

int Sonar::requestRegisterValues(unsigned char addr, bool flushbuf)
{
	unsigned char sbuf[8];
	int nwritten;
	
	sbuf[0] = 0x55;
	sbuf[1] = addr+0x01;
	sbuf[2] = 0;
	sbuf[3] = 8;
	
	nwritten = write( this->fd, sbuf, 4 );
	
	if (flushbuf){
		tcflush( fd, TCIFLUSH );		
	}
			
	if ( nwritten != 4 ) {
		printf( "Failed to request register values from %d\n", (int) addr );		
		return -1;
	}
	
	return 0;
}


int Sonar::getSonarReadings(int range[])
{
	
	int delay = 95000;
	int count = 0;	
	int r;

	delay -= N_SONAR * 20000;
	
	for (int i = 0; i < N_SONAR; i++){
		this->requestRangeCm(address_book[i]);
		
		//usleep( 25000 );
		usleep( 17000 );		
	}
	if (delay > 0){
		usleep( delay );
	}	
	for (int i = 0; i < N_SONAR; i++){
		this->requestRegisterValues(address_book[i]);
		r = this->getRange();

		if ( r != range[i] ){
			range[i] = r;
			count++;
		}
		
		if (range[i] == 65535){
			range[i] = 0;
		}
	}
	
	return count;

}

int Sonar::changeAddress(unsigned char old_addr, unsigned char new_addr)
{
	unsigned char sbuf[8];
	int nwritten;
 	int nbytes = 5;
	
// 	sbuf[0] = 0xA0;
// 	sbuf[1] = 0xAA;
// 	sbuf[2] = 0xA5;
// 	sbuf[3] = new_addr;
	
	sbuf[0] = 0x55;
	sbuf[1] = old_addr;
	sbuf[2] = 0;
	sbuf[3] = 1;	
	sbuf[4] = 0xA0;
	

	nwritten = write( this->fd, sbuf, nbytes );
	tcflush( fd, TCIFLUSH );
	if ( nwritten != nbytes ) {
		printf( "Failed to request change of address from %#x to %#x\n",  old_addr, new_addr );
		return -1;
	}

	sbuf[0] = 0x55;
	sbuf[1] = old_addr;
	sbuf[2] = 0;
	sbuf[3] = 1;	
	sbuf[4] = 0xAA;

	nwritten = write( this->fd, sbuf, nbytes );
	tcflush( fd, TCIFLUSH );
	if ( nwritten != nbytes ) {
		printf( "Failed to request change of address from %#x to %#x\n",  old_addr, new_addr );
		return -1;
	}

	sbuf[0] = 0x55;
	sbuf[1] = old_addr;
	sbuf[2] = 0;
	sbuf[3] = 1;
	sbuf[4] = 0xA5;

	nwritten = write( this->fd, sbuf, nbytes );
	tcflush( fd, TCIFLUSH );
	if ( nwritten != nbytes ) {
		printf( "Failed to request change of address from %#x to %#x\n",  old_addr, new_addr );
		return -1;
	}
	
	sbuf[0] = 0x55;
	sbuf[1] = old_addr;
	sbuf[2] = 0;
	sbuf[3] = 1;
	sbuf[4] = new_addr;

	nwritten = write( this->fd, sbuf, nbytes );
	tcflush( fd, TCIFLUSH );


	if ( nwritten != nbytes ) {
		printf( "Failed to request change of address from %#x to %#x\n",  old_addr, new_addr );
		return -1;
	}
	
	printf( "Requested change of address from %#x to %#x\n",  old_addr, new_addr );
	return 0;
}



	}
}

#endif
