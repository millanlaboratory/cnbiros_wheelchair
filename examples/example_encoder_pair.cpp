#include "cnbiros_wheelchair/EncoderThread.hpp"
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

void usage( char *x ) {
	printf( "usage: %s /dev/ttyUSB0 /dev/ttyUSB1\n", x );
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
	double dl = 0.0, dr = 0.0;
	
	
	printf("CNBI wheel encoder test program\n");
 
	if (argc != 3) {
		printf("please provide 2 aurguments\n");
		usage(argv[0]);
		return EINVAL;
	}

	cnbiros::wheelchair::EncoderThread* encright;
	cnbiros::wheelchair::EncoderThread* encleft;

	try {
		encright = new cnbiros::wheelchair::EncoderThread(argv[1]);
	} catch (std::runtime_error& e) {
		fprintf(stderr, "%s\n", e.what());
		exit(EXIT_FAILURE);
	}

	try {
		encleft = new cnbiros::wheelchair::EncoderThread(argv[2]);
	} catch (std::runtime_error& e) {
		fprintf(stderr, "%s\n", e.what());
		exit(EXIT_FAILURE);
	}
	
	while (!kbhit()) { 
 		dl = encleft->getDelta();
 		dr = encright->getDelta();
		printf("Delta (left, right) ( % 3.0f, % 3.0f )\n", dl, dr);
			
		//printf("%4f\n", enc_thread1.getDelta());
			
		usleep(100000);
	}

	delete encright;
	delete encleft;
	return 0;
}

