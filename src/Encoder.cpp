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
	
	this->verbose = debug_msgs;
	this->first_time = true;
	
	try {
		this->setupPort(port.c_str());
		printf("Created a Encoder object\n");
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
}

int Encoder::setupPort(const char* port)
{
	int myerrno;
	struct termios termInfo;

	this->fd = open( port, O_RDWR | O_NOCTTY | O_SYNC );
	
	if ( this->fd == -1 ) {
		myerrno = errno;
		//printf( "open() of %s failed: %s\n", port, strerror( myerrno ) );
		return myerrno;
	}

	if ( ioctl( this->fd , TCGETA, & termInfo ) == -1 ) {
		myerrno = errno;
		//printf( "ioctl(TCGETA) failed: %s\n", strerror( myerrno ) );
		return myerrno;
	}

	if( tcgetattr( this->fd, & termInfo ) == -1 ) {
		myerrno = errno;
		//printf( "tcgetattr() failed: %s\n", strerror( myerrno ) );
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
		myerrno = errno;
		//printf( "tcsetattr() failed: %s\n", strerror( myerrno ) );
		return myerrno;
	}	

	return 0;
}

int Encoder::setupEncoder()
{
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	unsigned short range;
	const int N_rbuf = 60;
	char rbuf[N_rbuf], hexstr[19];
	
	this->flushEncoder();
	
	this->testEncoder();
	
	if(this->verbose){
		this->getEncoderVersion();
	}

	this->flushEncoder();
	
	// Keep distance readings active
//  	toggleDistanceOutput();
//  	toggleDistanceOutput();


// 	// Check to see what mode we're in (turn off velocity mode, because we don't need it!)
// 	rbuf[0] = 0;
// 	tries = 2;
// 	bool velon = true;
// 	while (tries > 0 && velon){
// 		// search for <LF>
// 		while (rbuf[0] != 13){
// 			nread = read( fd, rbuf, 1 );
// 			if ( nread == -1 ) {
// 				myerrno = errno;
// 				printf( "read() failed: %s\n", strerror( myerrno ) );
// 				return myerrno;
// 			}
// 			if ( nread == 0 ) {
// 				printf( "read() failed, unexpected EOF whilst checking for velocity readings\n" );
// 				return ENODATA;
// 			}
// 		}
// 		if(this->verbose){
// 			printf("Checking for velocity readings\n");
// 		}
// 		rbuf[0] = 0;
// 		if (read( fd, rbuf, 1 ) == 1){
// 			if (rbuf[0] == 'V'){
// 				toggleVelocityOutput();
// 				if(this->verbose){
// 					printf ("Turned velocity readings off\n");
// 				}
// 				velon = false;
// 			}
// 		}
// 		tries--;
// 	}
// 	// End velocity mode check
// 
// 	flushEncoder();
	
	return 0;
}

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
			printf( "Failed to toggle velocity output on wheel encoders\n");
		}
		return -1;
	} else {
		if(this->verbose){
			printf("Toggled velocity output on wheel encoders\n");
		}
	}
	
	return 0;
}

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
			printf( "Failed to toggle distance output on wheel encoders\n");
		}
		return -1;
	} else {
		if(this->verbose){
			printf("Toggled distance output on wheel encoders\n");
		}
	}
	
	return 0;
}

int Encoder::flushEncoder()
{
	unsigned char sbuf[8];
	unsigned char rbuf[8];
	int nwritten;
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	
	char c;
	
	sbuf[0] = '.';
	
	int N_send = 1;
	int N_rec = 100;
	
	nwritten = write( this->fd, sbuf, N_send );
	
	tcflush( this->fd, TCIFLUSH );		
	
	if ( nwritten != N_send ) {
		printf( "Failed to flush endoer\n");
		return -1;
	} 
	

	for( tries = 0, nread = 0; tries < 256; tries++ ) {
		nread = read( this->fd, &c, 1);
		if ( nread == -1 ) {
			myerrno = errno;
			printf( "FLUSH: read() failed: %s\n", strerror( myerrno ) );
			return myerrno;
		}
		if ( nread == 0 ) {
			printf( "read() failed, unexpected EOF\n" );
			return ENODATA;
		}

		if ( c == '.'){
			if(this->verbose){
				printf("Flushed encoder buffer\n");
			}
			break;
		}
	}
	
	return 0;
}

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
		printf( "Failed to request encoder version\n");
		return -1;
	} 
	
	for( totalread = 0, ndata = N_rec, nread = 0, tries = 0, maxtries = 10;
						totalread < ndata && tries < maxtries; tries++ ) {
		nread = read( fd, rbuf + totalread, ndata - totalread );
		if ( nread == -1 ) {
			myerrno = errno;
			printf( "read() failed: %s\n", strerror( myerrno ) );
			return myerrno;
		}
		if ( nread == 0 ) {
			printf( "read() failed, unexpected EOF\n" );
			return ENODATA;
		}
		totalread += nread;
		printf("Wheelencoder Version : %s\n", rbuf);
	}
	
	return 0;
}

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
		printf( "Failed to send test message\n");
		return -1;
	} 
	
	for( totalread = 0, ndata = N_rec, nread = 0, tries = 0, maxtries = 10;
						totalread < ndata && tries < maxtries; tries++ ) {
		nread = read( fd, rbuf + totalread, ndata - totalread );
		if ( nread == -1 ) {
			myerrno = errno;
			printf( "read() failed: %s\n", strerror( myerrno ) );
			return myerrno;
		}
		if ( nread == 0 ) {
			printf( "read() failed, unexpected EOF\n" );
			return ENODATA;
		}
		totalread += nread;
//		printf("%d : %s\n", totalread, rbuf);
	}
	
	if (!((rbuf[0] == sbuf[0]) && (rbuf[1] == sbuf[2]) && (rbuf[2] == sbuf[1]))){
		// If the test succeeds, nibbles 1 and 2 should swap positions
		// e.g. if you send E5A, you should receive EA5.
		printf("Encoder self-test failed, received %d characters : %s\n", totalread, rbuf);
		return -1;
	}
	
	if(this->verbose){
		printf("Wheel encoder startup test passed!\n");
	}
	return 0;
}


/* This is a BLOCKING function! */
int Encoder::readEncoder()
{
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	unsigned short range;
	const int N_rbuf = 60;
	char rbuf[N_rbuf], hexstr[19];
	long int absdist = 0;
	
	rbuf[0] = 0;
	this->deltadist = 0;
	
	for( totalread = 0, ndata = N_rbuf, nread = 0, tries = 0, maxtries = 10;
						totalread < ndata && tries < maxtries; tries++ ) {
		nread = read( this->fd, rbuf + totalread, 1 );
		if ( nread == -1 ) {
			myerrno = errno;
			printf( "read() failed: %s\n", strerror( myerrno ) );
			return myerrno;
		}
		if ( nread == 0 ) {
			printf( "read() failed, unexpected EOF\n" );
			return ENODATA;
		}
		totalread += nread;
// 			printf("%d : %d\n", nread, totalread);
		if (totalread == 6 && rbuf[0] == 'V'){
			// Ignore velocity readings
			totalread = 0;
			this->toggleVelocityOutput();
			this->flushEncoder();
			break;
		} else if (totalread == 10){
			// Check for <LF>
			if (rbuf[9] == 13){
				// Correct reading obtained			
				rbuf[9] = 0;
				sprintf(hexstr, "0x%s", &rbuf[1]);
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
					printf(" Distance: Raw = 0x%s, Total = %10d, Delta = %3d\n", &rbuf[1], this->dist, this->deltadist );
				}
// 					for (int i = 0; i < 9; i++){
// 						printf("'%d' ", rbuf[i]);
// 					}
// 					printf("\n");
			} else {
				// Lost sync; flush the encoder buffer
				this->flushEncoder();
			}
		}
	}
	
	return this->deltadist;
			
}
	
			
/*
int fd;

int setupPort(char* port)
{
	int myerrno;
	struct termios termInfo;

	fd = open( port, O_RDWR | O_NOCTTY | O_SYNC );
	
	if ( fd == -1 ) {
		myerrno = errno;
		printf( "open() of %s failed: %s\n", port, strerror( myerrno ) );
		return myerrno;
	}

	if ( ioctl( fd , TCGETA, & termInfo ) == -1 ) {
		myerrno = errno;
		printf( "ioctl(TCGETA) failed: %s\n", strerror( myerrno ) );
		return myerrno;
	}

	if( tcgetattr( fd, & termInfo ) == -1 ) {
		myerrno = errno;
		printf( "tcgetattr() failed: %s\n", strerror( myerrno ) );
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

	tcflush( fd, TCIFLUSH );

	if ( tcsetattr( fd, TCSANOW, &termInfo) == -1 ) {
		myerrno = errno;
		printf( "tcsetattr() failed: %s\n", strerror( myerrno ) );
		return myerrno;
	}	

	return 0;
}


int toggleVelocityOutput()
{
	unsigned char sbuf[8];
	int nwritten;
	
	sbuf[0] = 'V';
	sbuf[1] = '\n';
	
	nwritten = write( fd, sbuf, 2 );
	
	tcflush( fd, TCIFLUSH );		
	
	if ( nwritten != 2 ) {
		printf( "Failed to toggle velocity output on wheel encoders\n");
		return -1;
	} else {
		printf("Toggled velocity output on wheel encoders\n");
	}
	
	return 0;
}

int toggleDistanceOutput()
{
	unsigned char sbuf[10];
	int nwritten;
	
	sbuf[0] = 'D';
	sbuf[1] = '\n';
	
	nwritten = write( fd, sbuf, 2 );
	
	tcflush( fd, TCIFLUSH );		
	
	if ( nwritten != 2 ) {
		printf( "Failed to toggle distance output on wheel encoders\n");
		return -1;
	} else {
		printf("Toggled distance output on wheel encoders\n");
	}
	
	return 0;
}

int flushEncoder()
{
	unsigned char sbuf[8];
	unsigned char rbuf[8];
	int nwritten;
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	
	char c;
	
	sbuf[0] = '.';
	
	int N_send = 1;
	int N_rec = 100;
	
	nwritten = write( fd, sbuf, N_send );
	
	tcflush( fd, TCIFLUSH );		
	
	if ( nwritten != N_send ) {
		printf( "Failed to flush endoer\n");
		return -1;
	} 
	

	for( tries = 0, nread = 0; tries < 256; tries++ ) {
		nread = read( fd, &c, 1);
		if ( nread == -1 ) {
			myerrno = errno;
			printf( "FLUSH: read() failed: %s\n", strerror( myerrno ) );
			return myerrno;
		}
		if ( nread == 0 ) {
			printf( "read() failed, unexpected EOF\n" );
			return ENODATA;
		}

		if ( c == '.'){
				printf("Flushed encoder buffer\n");
				break;
		}
	}
	
	return 0;
}

int getEncoderVersion()
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
	
	nwritten = write( fd, sbuf, N_send );
	
	tcflush( fd, TCIFLUSH );		
	
	if ( nwritten != N_send ) {
		printf( "Failed to request endoer version\n");
		return -1;
	} 
	
	for( totalread = 0, ndata = N_rec, nread = 0, tries = 0, maxtries = 10;
						totalread < ndata && tries < maxtries; tries++ ) {
		nread = read( fd, rbuf + totalread, ndata - totalread );
		if ( nread == -1 ) {
			myerrno = errno;
			printf( "read() failed: %s\n", strerror( myerrno ) );
			return myerrno;
		}
		if ( nread == 0 ) {
			printf( "read() failed, unexpected EOF\n" );
			return ENODATA;
		}
		totalread += nread;
		printf("Wheelencoder Version : %s\n", rbuf);
	}
	
	return 0;
}

int testEncoder()
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
	
	tcflush( fd, TCIFLUSH );		
	
	if ( nwritten != N_send ) {
		printf( "Failed to send test message\n");
		return -1;
	} 
	
	for( totalread = 0, ndata = N_rec, nread = 0, tries = 0, maxtries = 10;
						totalread < ndata && tries < maxtries; tries++ ) {
		nread = read( fd, rbuf + totalread, ndata - totalread );
		if ( nread == -1 ) {
			myerrno = errno;
			printf( "read() failed: %s\n", strerror( myerrno ) );
			return myerrno;
		}
		if ( nread == 0 ) {
			printf( "read() failed, unexpected EOF\n" );
			return ENODATA;
		}
		totalread += nread;
//		printf("%d : %s\n", totalread, rbuf);
	}
	
	if (!((rbuf[0] == sbuf[0]) && (rbuf[1] == sbuf[2]) && (rbuf[2] == sbuf[1]))){
		// If the test succeeds, nibbles 1 and 2 should swap positions
		// e.g. if you send E5A, you should receive EA5.
		printf("Test failed, received %d characters : %s\n", totalread, rbuf);
		return -1;
	}
	
	printf("Wheel encoder startup test passed!\n");
	return 0;
}

int main( int argc, char *argv[] ) 
{
	printf("CNBI wheel encoder test program\n");
	int myerrno, nread;
	unsigned int ndata, totalread, tries, maxtries;
	unsigned short range;
	const int N_rbuf = 60;
	char rbuf[N_rbuf], hexstr[19];
	bool first_time = true;
	
	//const long MAX_HEX = exp(2, 32) - 1;
	long int absdist = 0;
	int dist = 0, ldist = 0, deltadist = 0;
	
	myerrno = setupPort("/dev/ttyUSB0");
	if (myerrno != 0){
		printf("Setup failed, quitting...\n");
		exit(myerrno);
	}
	
	flushEncoder();
	
	testEncoder();
	
 	getEncoderVersion();

	flushEncoder();
	
	// Keep distance readings active
//  	toggleDistanceOutput();
//  	toggleDistanceOutput();


	// Check to see what mode we're in (turn off velocity mode, because we don't need it!)
	rbuf[0] = 0;
	tries = 2;
	bool velon = true;
	while (tries > 0 && velon){
		// search for <LF>
		while (rbuf[0] != 13){
			nread = read( fd, rbuf, 1 );
			if ( nread == -1 ) {
				myerrno = errno;
				printf( "read() failed: %s\n", strerror( myerrno ) );
				return myerrno;
			}
			if ( nread == 0 ) {
				printf( "read() failed, unexpected EOF whilst checking for velocity readings\n" );
				return ENODATA;
			}
		}
		printf("Checking for velocity readings\n");
		rbuf[0] = 0;
		if (read( fd, rbuf, 1 ) == 1){
			if (rbuf[0] == 'V'){
				toggleVelocityOutput();
				printf ("Turned velocity readings off\n");
				velon = false;
			}
		}
		tries--;
	}
	// End velocity mode check

	flushEncoder();
	
	for (;;){

		rbuf[0] = 0;
		
		for( totalread = 0, ndata = N_rbuf, nread = 0, tries = 0, maxtries = 10;
							totalread < ndata && tries < maxtries; tries++ ) {
			nread = read( fd, rbuf + totalread, 1 );
			if ( nread == -1 ) {
				myerrno = errno;
				printf( "read() failed: %s\n", strerror( myerrno ) );
				return myerrno;
			}
			if ( nread == 0 ) {
				printf( "read() failed, unexpected EOF\n" );
				return ENODATA;
			}
			totalread += nread;
// 			printf("%d : %d\n", nread, totalread);
			
			if (totalread == 10){
				// Check for <LF>
				if (rbuf[9] == 13){
					// Correct reading obtained			
					rbuf[9] = 0;
					sprintf(hexstr, "0x%s", &rbuf[1]);
					hexstr[18] = 0;
					absdist = strtol((const char*) hexstr, NULL, 16);
					//deltadist = strtod((const char*) hexstr, NULL);
					dist = (signed int) absdist;
					if (first_time){
						deltadist = 0;
						first_time = false;
					} else {
						deltadist = dist - ldist;
					}
					ldist = dist;
					//printf(" Distance %s %ld\n", &rbuf[1], absdist);
					printf(" Distance: Raw = 0x%s, Total = %10d, Delta = %3d\n", &rbuf[1], dist, deltadist );
// 					for (int i = 0; i < 9; i++){
// 						printf("'%d' ", rbuf[i]);
// 					}
// 					printf("\n");
				} else {
					// Lost sync; flush the encoder buffer
					flushEncoder();
				}
			}
			
		
			
			
// 			printf("%d : %s\n", totalread, rbuf);
		}
		

		
//		printf("%s\n", rbuf);
//		flushEncoder();
// 		tcflush( fd, TCIFLUSH );		
	}
	
	return 0;
}
*/

	}
}

#endif
