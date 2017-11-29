#include "cnbiros_wheelchair/OdometryThread.hpp"

#define WHEEL_DIAMETER			0.3556f	// [m]
#define WHEEL_REVOLUTION_COUNTS	32		// Number of counted tick for a wheel revolution
#define WHEEL_AXLE				0.5240f	// [m] distance between wheels

void usage( char *x ) {
	printf( "usage: %s /dev/ttyUSB0 /dev/ttyUSB1\n", x );
}

int main( int argc, char *argv[] ) 
{
	double dl = 0.0, dr = 0.0;

	double resolution;
	double x, y, theta;
	
	printf("CNBI odometry test program\n");
 
	if (argc != 3) {
		printf("please provide 2 aurguments\n");
		usage(argv[0]);
		return EINVAL;
	}


	resolution = (float(WHEEL_DIAMETER)*M_PI)/float(WHEEL_REVOLUTION_COUNTS);

	cnbiros::wheelchair::OdometryThread odometry(argv[1], argv[2], WHEEL_AXLE, resolution);

	
	for (;;){
		
		odometry.getOdometry(&x, &y, &theta);
		printf("Position (x, y, theta)=> (%3.0f, %3.0f, %3.0f) [m, m, deg]\n", x, y, theta*180.0f / M_PI);
			
		usleep(100000);
	}
	
	return 0;
}

