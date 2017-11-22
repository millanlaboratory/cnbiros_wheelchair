
#include "cnbiros_wheelchair/SonarThread.hpp"

#include <iostream>

void usage( char *x ) {
	printf( "usage: %s /dev/ttyUSB0\n", x );
}

int main( int argc, char *argv[] ) 
{

	int N_SONAR = 2;
	cnbiros::wheelchair::SonarThread *sr;
	
	if ( argc != 2 ) {
		printf( "please provide exactly 1 argument\n" );
		usage( argv[0] );
		return EINVAL;
	}

	sr = new cnbiros::wheelchair::SonarThread(argv[1], N_SONAR);
				
	int range[N_SONAR];
		
	while ( 1 ) { 

		sr->getReadings(range);

		for (int i = 0; i < N_SONAR; i++){		
			std::cout << range[i] << "\t";
		}
		std::cout << std::endl;

		usleep(100000);
	}	

}


// 
// #include "sonar.h"
// 
// #include <iostream>
// 
// int fd;
// 
// void usage( char *x ) {
// 	printf( "usage: %s /dev/ttyUSB0\n", x );
// }
// 
// int main( int argc, char *argv[] ) 
// {
// 
// 	int N_SONAR = 4;
// 	Sonar *sr;
// 	
// 	if ( argc != 2 ) {
// 		printf( "please provide exactly 1 argument\n" );
// 		usage( argv[0] );
// 		return EINVAL;
// 	}
// 
// 	sr = new Sonar(argv[1], N_SONAR);
// 				
// 	int range[N_SONAR];
// 		
// 	while ( 1 ) { 
// 
// 		sr->getSonarReadings(range);
// 
// 		for (int i = 0; i < N_SONAR; i++){		
// 			std::cout << range[i] << "\t";
// 		//printf("%d\t%d\n", range[0], range[1]);
// 		}
// 		std::cout << std::endl;
// 	}	
// }
