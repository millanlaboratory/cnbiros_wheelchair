#ifndef CNBIROS_WHEELCHAIR_DXGPSB_CPP
#define CNBIROS_WHEELCHAIR_DXGPSB_CPP

#include "cnbiros_wheelchair/Dxgpsb.hpp"

namespace cnbiros {
	namespace wheelchair {

Dxgpsb::Dxgpsb(const std::string& port) {
  
	// Initialise stationary velocities;
	v_tx = 0x80;
	w_tx = 0x80;
	v_rx = 0x80;
	w_rx = 0x80;

	try {
		this->setupPort(port.c_str());
		printf("Created a Dxgpsb object\n");
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}
}

Dxgpsb::~Dxgpsb(void){
	this->closePort();
}

int Dxgpsb::closePort(void) {

	if(this->fd != -1) {
		tcflush( this->fd, TCIOFLUSH );
		close(this->fd);
	}

	return 0;
}

bool Dxgpsb::seekByte(char b, int search_limit = 24) {
	int myerrno, nread;
  	unsigned int ndata, totalread, tries, maxtries;
  	unsigned char sbuf[10];

  	bool found = false;
  
	for(totalread = 0, nread = 0, tries = 0, maxtries = 24;
		totalread < search_limit && tries < maxtries; tries++ ) {

		nread = read( this->fd, sbuf, 1);
  
		unsigned char c = sbuf[0];
  
		if (nread == -1) {
			myerrno = errno;
			throw std::runtime_error("read() failed:: " + std::string(strerror(myerrno)));
			return myerrno;
		}
    
		if (nread == 0) {
			throw std::runtime_error("read() failed, unexpected EOF");
			return ENODATA;
		}
    
		if (sbuf[0] == b){
			found = true;
			break;
		}
	}
  
	return found;
}

/* Writes to device and automatically generates and appends a sequence byte and
 * Dynamics checksum */
int Dxgpsb::writeToDevice(int n_bytes, unsigned char* data) {

	int retval = 0;
	unsigned char crem;
	static int sequence = 0;
	unsigned int len_packet = n_bytes + 2;
	unsigned char sdata[len_packet];  
	unsigned char csum = 0x00;
  
	for (int i = 0; i < n_bytes; i++){
		sdata[i] = data[i];
  	  	csum += sdata[i];
  	}

	sequence++;
	if (sequence >= 255)
		sequence = 0;
	
	sdata[n_bytes] = sequence;
  	csum += sequence;
  	  
  	crem = csum % 0xFF;

  	sdata[n_bytes + 1] = 0xFE - crem; 
  	
  	tcflush( this->fd, TCOFLUSH );
  	retval = write(this->fd, sdata, len_packet);
  	tcflush( this->fd, TCOFLUSH );

  	//printf("Sent packet:     0x");
  	//for (int i = 0; i < len_packet; i++){
	//	//printf("%02X ", sdata[i]);
  	//}
  	//printf("\n");

  	return retval;
}

/* Writes plain bytes to device without a checksum */
int Dxgpsb::writeToDeviceNOCHKSUM(int n_bytes, unsigned char* data) {
	int retval = 0;

  	tcflush( this->fd, TCOFLUSH );
  	retval = write(this->fd, data, n_bytes);
  	tcflush( this->fd, TCOFLUSH );

  	//printf("Sent packet:     0x");
  	for (int i = 0; i < n_bytes; i++){
		//printf("%02X ", data[i]);
  	}
  	//printf("\n");

  	return retval;
}

/* Reads regular packets from DX-GPSB */
int Dxgpsb::readDevice(unsigned char *return_data) {

	int myerrno, nread;
  	unsigned int ndata, totalread, tries, maxtries;  
  	const int NDATA = 4;
  	unsigned char sbuf[NDATA];
  	 
  	// Check for start byte
  	if (!seekByte('@')){
		throw std::runtime_error("Missing sample from Dxgpsb (start byte not found)");
  	  	return -1;
  	}

  	// Get Dxgpsb readings
  	for(totalread = 0, nread = 0, tries = 0, maxtries = 24;
		totalread < NDATA  && tries < maxtries; tries++ ) {

		nread = read( this->fd, sbuf + totalread, NDATA - totalread);
  	
  	  	if (nread == -1) {
  	  	  myerrno = errno;
  	  	  //printf("read() failed: %s\n", strerror( myerrno ));
  	  	  return myerrno;
  	  	}

  	  	if (nread == 0) {
  	  	  //printf( "read() failed, unexpected EOF\n" );
  	  	  return ENODATA;
  	  	}
  	  	
  	  	totalread += nread;
  	}
  
	for (int i = 0; i < NDATA; i++){
  	  return_data[i] = sbuf[i];
  	}

  return 0;
}

int Dxgpsb::setupPort(const char* port) {

	int myerrno;
  	struct termios termInfo;
  	unsigned char wbuf[24];
  	int wn = 0;
  	unsigned char rbuf[24];
  	unsigned char tmp;
  	
	this->fd = open( port, O_RDWR | O_NOCTTY | O_SYNC );
  	
	if (this->fd == -1) {
		myerrno = errno;
  	  	throw std::runtime_error("Cannot open motors at " + std::string(port) + ": " + std::string(strerror(myerrno)));
  	  	return myerrno;
  	}

  	if ( ioctl( this->fd , TCGETA, & termInfo ) == -1 ) {
		myerrno = errno;
  	  	throw std::runtime_error("ioctl(TCGETA) failed: " + std::string(strerror(myerrno)));
  	  	return myerrno;
  	}

  	if( tcgetattr( this->fd, & termInfo ) == -1 ) {
		myerrno = errno;
  	  	throw std::runtime_error("tcgetattr() failed: " + std::string(strerror(myerrno)));
  	  	return myerrno;
  	}

  	cfsetspeed( & termInfo , B38400 );
  	 
  	termInfo.c_cflag |=  ( CLOCAL | CREAD );
  	termInfo.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );
  	termInfo.c_cflag &= ~ PARENB;
  	termInfo.c_cflag &=  ~CSTOPB;
  	termInfo.c_cflag &= ~ CSIZE;
  	termInfo.c_cflag |=   CS8;
  	termInfo.c_iflag  =    IGNBRK | IGNPAR;
  	termInfo.c_oflag &= ~OPOST;
 
  	tcflush( this->fd, TCIFLUSH );
  	
  	if ( tcsetattr( this->fd, TCSANOW, &termInfo) == -1 ) {
		myerrno = errno;
  	  	throw std::runtime_error("tcsetattr() failed: " + std::string(strerror(myerrno)));
  	  	return myerrno;
  	}	
  	
  	printf("Connected to a Dxgpsb on port: %s\n", port);
  	
  	printf("Logging on in standard mode... \n");
  	
  	wbuf[0] = '@';	// '@' start byte;
  	wbuf[1] = 0x39; 
  	wbuf[2] = 0x39; 
  	wbuf[3] = 0x39; 
  	wbuf[4] = 0x30; // 1 + 2 + 3 + 4 % 8 == 0 for standard mode
  	wbuf[5] = 0x38; 
  	wbuf[6] = 0x38; 
  	wbuf[7] = 0x38; 
  	wbuf[8] = 0x30; // 1 + 2 + 3 + 4 % 7 == 0 for standard mode
  	wn = 9;

  	writeToDeviceNOCHKSUM(wn, wbuf);     
  	
  	wbuf[0] = 0x25; // '%' start byte;
  	wbuf[1] = 0x80; // Speed : 0x80 = stop
  	wbuf[2] = 0x80; // Direction : 0x80 = stop 
  	wbuf[3] = 0x00; // Status A (0x02 = shared)
  	wbuf[4] = 0x00; // Status B (0x04 = horn)
  	wn = 5;

	// Stop wheelchair
	writeToDevice(wn, wbuf);
	
	printf("\nFinished setupPort()\n");
  
	return 0;
}

int Dxgpsb::update() {
	this->readVelocities();
	this->sendVelocities();
	
	return 0;
}

int Dxgpsb::sendVelocities() {
	unsigned char wbuf[24];
	int wn = 5;
	
	// Write Velocities 
	wbuf[0] = 0x25;			// '%' start byte;
	wbuf[1] = this->v_tx;	// Speed : 0x80 = stop
	wbuf[2] = this->w_tx;	// Direction : 0x80 = stop 
	wbuf[3] = 0x00;			// Status A (0x02 = shared)
	wbuf[4] = 0x00;			// Status B (0x04 = horn)
	writeToDevice(wn, wbuf);
}
	
int Dxgpsb::readVelocities() {
	unsigned char rbuf[24];
	
	// Read Joystick
	readDevice(rbuf);
	//printf("Received packet: 0x40 ");
	for (int j = 0; j < 4; j++){
		//printf("%02X ", rbuf[j]);
	}
	//printf("\n");
	this->v_rx = rbuf[0];
	this->w_rx = rbuf[1];
	
	return 0;
}

int Dxgpsb::setVelocities(float v, float w) {

	this->v_tx = (char)(v * 127.0 + 128.0);
	this->w_tx = (char)(w * 127.0 + 128.0);
	
	return 0;
}

int Dxgpsb::setVelocities(char v, char w) {

	this->v_tx = v;
	this->w_tx = w;
	
	return 0;
}

int Dxgpsb::getVelocities(char &v, char &w) {
	v = this->v_rx;
	w = this->w_rx;
	
	return 0;
}

int Dxgpsb::getVelocities(float &v, float &w) {
	v = (this->v_rx - 128) / 127.0;
	w = (this->w_rx - 128) / 128.0;
	
	return 0;
}

	}
}

#endif
