
#include "cnbiros_wheelchair/SonarThread.hpp"

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


void usage( char *x ) {
	printf( "usage: %s /dev/ttyUSB0 5\n", x );
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

int main( int argc, char *argv[] ) 
{

	int N_SONAR;
	char input;
	cnbiros::wheelchair::SonarThread *sr;
	
	if ( argc != 3 ) {
		printf( "please provide exactly 2 arguments (port and number of sensors)\n" );
		usage( argv[0] );
		return EINVAL;
	}
	N_SONAR = std::stoi(std::string(argv[2]));

	try {
		sr = new cnbiros::wheelchair::SonarThread(argv[1], N_SONAR);
	} catch (std::runtime_error& e) {
		fprintf(stderr, "%s\n", e.what());
		exit(EXIT_FAILURE);
	}
				
	printf("Checking %d sensors on port %s\n", N_SONAR, argv[1]);

	int range[N_SONAR];

	while (!kbhit()) { 

		sr->getReadings(range);

		for (int i = 0; i < N_SONAR; i++){		
			std::cout << range[i] << "\t";
		}
		std::cout << std::endl;

		usleep(100000);
	}	

	delete sr;
	usleep(100000);
	return 0;
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
