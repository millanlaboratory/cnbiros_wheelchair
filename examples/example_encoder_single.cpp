#include "cnbiros_wheelchair/EncoderThread.hpp"

void usage( char *x ) {
	printf( "usage: %s /dev/ttyUSB0\n", x );
}

int main( int argc, char *argv[] ) 
{
	double counts = 0.0;
	
	
	printf("CNBI wheel encoder test program\n");
 
	if (argc != 2) {
		printf("please provide 1 aurgument\n");
		usage(argv[0]);
		return EINVAL;
	}


	cnbiros::wheelchair::EncoderThread encoder(argv[1]);
	
	for (;;){
		counts = encoder.getDelta();
		printf("Counts on %s: % 3.0f\n", argv[1], counts);
			
		//printf("%4f\n", enc_thread1.getDelta());
			
		usleep(100000);
	}
	
	return 0;
}

