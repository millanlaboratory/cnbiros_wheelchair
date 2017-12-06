#include "cnbiros_wheelchair/OdometryThread.hpp"

#define WHEEL_DIAMETER			0.3556f	// [m]
#define WHEEL_REVOLUTION_COUNTS	280		// Number of counted tick for a wheel revolution
#define WHEEL_AXLE				0.5240f	// [m] distance between wheels

void usage( char *x ) {
	printf( "usage: %s /dev/ttyUSB0 /dev/ttyUSB1\n", x );
}

int main( int argc, char *argv[] ) 
{

	double x, y, theta;
	double vx, vy, vtheta;

	printf("CNBI odometry test program\n");
 
	if (argc != 3) {
		printf("please provide 2 aurguments\n");
		usage(argv[0]);
		return EINVAL;
	}

	cnbiros::wheelchair::OdometryThread* odometry;
	try {
		odometry = new cnbiros::wheelchair::OdometryThread(argv[1], argv[2], 
						WHEEL_AXLE, WHEEL_DIAMETER, WHEEL_REVOLUTION_COUNTS);
	} catch (std::runtime_error& e) {
		fprintf(stderr, "%s\n", e.what());
		exit(EXIT_FAILURE);
	}
	
	for (;;){
		
		odometry->getOdometry(&x, &y, &theta, &vx, &vy, &vtheta);
		printf("Position (x,   y,  theta)=> (%3.2f, %3.2f, %3.2f) [m, m, deg]\n", 
				x, y, theta*180.0f / M_PI);
		printf("Velocity (vx, vy, vtheta)=> (%3.2f, %3.2f, %3.2f) [m/s, m/s, deg/s]\n", 
				vx, vy, vtheta*180.0f / M_PI);
			
		usleep(100000);
	}

	delete odometry;
	return 0;
}

