#include "cnbiros_wheelchair/EncoderThread.hpp"

void usage( char *x ) {
	printf( "usage: %s /dev/ttyUSB0 /dev/ttyUSB1\n", x );
}

int main( int argc, char *argv[] ) 
{
	double dl = 0.0, dr = 0.0;
	
	
	printf("CNBI wheel encoder test program\n");
 
	if (argc != 3) {
		printf("please provide 2 aurguments\n");
		usage(argv[0]);
		return EINVAL;
	}


	cnbiros::wheelchair::EncoderThread encright(argv[1]);
	cnbiros::wheelchair::EncoderThread encleft(argv[2]);
	
	for (;;){
 		dl = encleft.getDelta();
 		dr = encright.getDelta();
		printf("Delta (left, right) ( % 3.0f, % 3.0f )\n", dl, dr);
			
		//printf("%4f\n", enc_thread1.getDelta());
			
		usleep(100000);
	}
	
	return 0;
}

