#ifndef CNBIROS_WHEELCHAIR_ENCODER_CPP
#define CNBIROS_WHEELCHAIR_ENCODER_CPP

#include "cnbiros_wheelchair/Encoder.hpp"

namespace cnbiros {
	namespace wheelchair {

Encoder::Encoder(const std::string& port, bool debug_msgs) {
	int myerrno;
	
	this->dist = 0;
	this->ldist = 0;
	this->deltadist = 0;
	this->encport = port;
	this->verbose = debug_msgs;
	this->first_time = true;


	try {
		this->setupPort(this->encport.c_str());
		printf("Created a Encoder object for %s\n", this->encport.c_str());
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}
	
	if (this->verbose){
		// This just runs some start-up checks, but is not necessary (and can
		// cause problems on MT versions...)
		this->setupEncoder();
	}
}


Encoder::~Encoder() {	
	this->ClosePort();
}

int Encoder::ClosePort() {
	
	if (this->fd != -1) {
		close(this->fd);
		this->fd = -1;
	}
	return 0;
}

int Encoder::setupPort(const char* port)
{
	int myerrno;
	struct termios termInfo;

	this->fd = open( port, O_RDWR | O_NOCTTY | O_SYNC );
	
	if ( this->fd == -1 ) {
		this->ClosePort();
		myerrno = errno;
		throw std::runtime_error("Cannot open encoder at " + std::string(port) + ": " +  strerror(myerrno));
		return myerrno;
	}

	if ( ioctl( this->fd , TCGETA, & termInfo ) == -1 ) {
		this->ClosePort();
		myerrno = errno;
		throw std::runtime_error("ioctl(TCGETA) failed: " + std::string(strerror(myerrno)));
		return myerrno;
	}

	if( tcgetattr( this->fd, & termInfo ) == -1 ) {
		this->ClosePort();
		myerrno = errno;
		throw std::runtime_error("tcgetattr() failed: " + std::string(strerror(myerrno)));
		return myerrno;
	}

	cfsetispeed( & termInfo , B38400 );
	termInfo.c_cflag |=  ( CLOCAL | CREAD );
	termInfo.c_lflag &= ~( ICANON | ECHO | ISIG );
	termInfo.c_cflag &= ~  PARENB;
	termInfo.c_cflag &= ~  CSTOPB;
	termInfo.c_cflag &= ~  CSIZE;
	termInfo.c_cflag |=    CS8;
	termInfo.c_iflag  =    IGNBRK | IGNPAR | INLCR;

	tcflush( this->fd, TCIFLUSH );

	if ( tcsetattr( this->fd, TCSANOW, &termInfo) == -1 ) {
		this->ClosePort();
		myerrno = errno;
		throw std::runtime_error("tcsetattr() failed: " + std::string(strerror(myerrno)));
		return myerrno;
	}	

	return 0;
}

/* Perform basic checks and flush the communications buffer */
int Encoder::setupEncoder()
{
	try {
		this->flushEncoder();
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}

	try {
		this->testEncoder();
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}
	
	if(this->verbose){
		try {
			this->getEncoderVersion();
		} catch (std::runtime_error& e) {
			throw std::runtime_error(e.what());
		}
	}

	try {
		this->flushEncoder();
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}
	
	return 0;
}

/* Toggle the automatic output of encoder velocity readings (there is no way
	of telling the current state, without checking for ASCII 'V' characters
	in the serial protocol) This is a problem with the NuBotics 
	protocol */
int Encoder::toggleVelocityOutput()
{
	unsigned char sbuf[2];
	int nwritten;
	
	sbuf[0] = 'V';
	sbuf[1] = '\n';
	
	nwritten = write( this->fd, sbuf, 2 );
	
	tcflush( this->fd, TCIFLUSH );		
	
	if ( nwritten != 2 ) {
		if(this->verbose){
			throw std::runtime_error("Failed to toggle velocity output on wheel encoders");
		}
		return -1;
	} else {
		if(this->verbose){
			printf("Toggled velocity output on wheel encoders\n");
		}
	}
	
	return 0;
}

/* Toggle the automatic output of encoder distance readings (there is no way
	of telling the current state since the indicator is an ASCII 'D', which is
	also used as a data nibble (a hex character) This is a problem with the
	NuBotics protocol */
int Encoder::toggleDistanceOutput()
{
	unsigned char sbuf[10];
	int nwritten;
	
	sbuf[0] = 'D';
	sbuf[1] = '\n';
	
	nwritten = write( this->fd, sbuf, 2 );
	
	tcflush( this->fd, TCIFLUSH );		
	
	if ( nwritten != 2 ) {
		if(this->verbose){
			throw std::runtime_error("Failed to toggle distance output on wheel encoders");
		}
		return -1;
	} else {
		if(this->verbose){
			printf("Toggled distance output on wheel encoders\n");
		}
	}
	
	return 0;
}

/* Tell the wheel encoder to flush its buffer and check that it does it. Re-synchronise 
	with an ASCII '.' */
int Encoder::flushEncoder()
{
	unsigned char sbuf[8];
	int nwritten;
	int myerrno, nread;

	unsigned int tries;
	
	char c;
	
	sbuf[0] = '.';
	
	int N_send = 1;
	
	tcflush( this->fd, TCIFLUSH );
	
	nwritten = write( this->fd, sbuf, N_send );
	
	tcflush( this->fd, TCIFLUSH );		
	
	printf("Attempting to flush encoder buffer %d\n", fd);
				
	if ( nwritten != N_send ) {
		throw std::runtime_error("Failed to flush encoder");
		return -1;
	} 
	

	for( tries = 0, nread = 0; tries < 256; tries++ ) {
		printf("reading...\n");
		nread = read( this->fd, &c, 1);
		printf("...done\n");
		if ( nread == -1 ) {
			myerrno = errno;
			throw std::runtime_error("FLUSH: read() failed: " + std::string(strerror(myerrno)));
			return myerrno;
		}
		if ( nread == 0 ) {
			throw std::runtime_error("FLUSH: read() failed, unexpected EOF");
			return ENODATA;
		}

		if ( c == '.'){
			if(this->verbose){
				printf("Flushed encoder buffer\n");
			}
			break;
		}
	}
	
	printf("done\n");	
	return 0;
}

/* Output wheelencoder version info (not very useful, maybe for debug or 
	additional comms check)*/
int Encoder::getEncoderVersion()
{
	unsigned char sbuf[8];
	unsigned char rbuf[8];
	int nwritten;
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	
	sbuf[0] = 'N';
	sbuf[1] = '\n';
	
	int N_send = 2;
	int N_rec = 6;
	
	nwritten = write( this->fd, sbuf, N_send );
	
	tcflush( this->fd, TCIFLUSH );		
	
	if ( nwritten != N_send ) {
		throw std::runtime_error("Failed to request encoder version");
		return -1;
	} 
	
	for( totalread = 0, ndata = N_rec, nread = 0, tries = 0, maxtries = 10;
						totalread < ndata && tries < maxtries; tries++ ) {
		nread = read( fd, rbuf + totalread, ndata - totalread );
		if ( nread == -1 ) {
			myerrno = errno;
			throw std::runtime_error("read() failed: " + std::string(strerror(myerrno)));
			return myerrno;
		}
		if ( nread == 0 ) {
			throw std::runtime_error("read() failed, unexpected EOF");
			return ENODATA;
		}
		totalread += nread;
		printf("Wheelencoder Version : %s\n", rbuf);
	}
	
	return 0;
}

/* Check encoder communications */
int Encoder::testEncoder()
{
	unsigned char sbuf[8];
	unsigned char rbuf[8];
	int nwritten;
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	
	sbuf[0] = 'E';
	sbuf[1] = '5';
	sbuf[2] = 'A';
	sbuf[3] = '\n';
	
	int N_send = 4;
	int N_rec = 4;
	
	nwritten = write( fd, sbuf, N_send );
	
	tcflush( this->fd, TCIFLUSH );		
	
	if ( nwritten != N_send ) {
		throw std::runtime_error("Failed to send test message");
		return -1;
	} 
	
	for( totalread = 0, ndata = N_rec, nread = 0, tries = 0, maxtries = 10;
						totalread < ndata && tries < maxtries; tries++ ) {
		nread = read( fd, rbuf + totalread, ndata - totalread );
		if ( nread == -1 ) {
			myerrno = errno;
			throw std::runtime_error("read() failed: " + std::string(strerror(myerrno)));
			return myerrno;
		}
		if ( nread == 0 ) {
			throw std::runtime_error("read() failed, unexpected EOF");
			return ENODATA;
		}
		totalread += nread;
//		printf("%d : %s\n", totalread, rbuf);
	}
	
	if (!((rbuf[0] == sbuf[0]) && (rbuf[1] == sbuf[2]) && (rbuf[2] == sbuf[1]))){
		// If the test succeeds, nibbles 1 and 2 should swap positions
		// e.g. if you send E5A, you should receive EA5.
		throw std::runtime_error("Encoder self-test failed");
		return -1;
	}
	
	if(this->verbose){
		printf("Wheel encoder startup test passed!\n");
	}
	return 0;
}


/* This is a BLOCKING function! 
	It returns the "delta" distance travelled, since the function was last called */
int Encoder::readEncoder()
{
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	const int N_rbuf = 60;
	char rbuf[N_rbuf], hexstr[19];
	long int absdist = 0;
	
	rbuf[0] = 0;
	this->deltadist = 0;
	
	for( totalread = 0, ndata = N_rbuf, nread = 0, tries = 0, maxtries = 20;
						totalread < ndata && tries < maxtries; tries++ ) {

		nread = read( this->fd, rbuf + totalread, 1 );
		if ( nread == -1 ) {
			myerrno = errno;
			throw std::runtime_error("read() failed: " + std::string(strerror(myerrno)));
			return myerrno;
		}
		if ( nread == 0 ) {
			throw std::runtime_error("read() failed, unexpected EOF");
			return ENODATA;
		}

		totalread += nread;

		if (totalread == 6 && rbuf[0] == 'V'){
			// Ignore velocity readings
			this->toggleVelocityOutput();
			this->flushEncoder();
			break;
		} else if ((totalread >= 9) && rbuf[totalread] == 13 ){
			// LF character found: assume correct reading obtained
// 			if (!(totalread >=10 && rbuf[totalread-10] == 'D')){
// 			  printf("WARNING: no start byte found! Using data anyway...\n");
// 			}
			rbuf[totalread] = 0;
			sprintf(hexstr, "0x%s", &rbuf[totalread - 8]);
			//this->debugOut(&rbuf[totalread- 8]);;
			hexstr[18] = 0;
			absdist = strtol((const char*) hexstr, NULL, 16);
			//deltadist = strtod((const char*) hexstr, NULL);
			
			this->dist = (signed int) absdist;
			if (this->first_time){
				this->deltadist = 0;
				this->first_time = false;
			} else {
				this->deltadist = this->dist - this->ldist;
			}
			ldist = dist;
			//printf(" Distance %s %ld\n", &rbuf[1], absdist);
			if(this->verbose){
			//	printf(" Distance (%d): Raw = 0x%s, Total = %10d, Delta = %3d\n", fd,  &rbuf[1], this->dist, this->deltadist );
			}
			totalread = 0;
			break;
		}
	}
	
	return this->deltadist;
			
}
	
			
void Encoder::debugOut(const char* str)
{
  int limit = 20;
  int i = 0;
  //printf("Debug out: ");
  fprintf(stderr, "%d", this->fd);
  
  while(i < limit && str[i] != '\n'){
    fprintf(stderr, " %x", (unsigned int) str[i]); 
    i++;
  }
  
  fprintf(stderr, "\n");
  
}


	}
}

#endif
