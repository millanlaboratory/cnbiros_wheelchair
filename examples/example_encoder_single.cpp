#include "cnbiros_wheelchair/EncoderThread.hpp"
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
void usage( char *x ) {
	printf( "usage: %s /dev/ttyUSB0\n", x );
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
	double counts = 0.0;
	int sequence = 0;
	
	
	printf("CNBI wheel encoder test program\n");
 
	if (argc != 2) {
		printf("please provide 1 aurgument\n");
		usage(argv[0]);
		return EINVAL;
	}

	cnbiros::wheelchair::EncoderThread* encoder;

	try {
		encoder = new cnbiros::wheelchair::EncoderThread(argv[1], true);
	} catch (std::runtime_error& e) {
		fprintf(stderr, "%s\n", e.what());
		exit(EXIT_FAILURE);
	}
	
	while (!kbhit()) { 
		counts = encoder->getDelta(sequence);
		printf("Counts on %s: % 3.0f - Sequence %d\n", argv[1], counts, sequence);
			
		//printf("%4f\n", enc_thread1.getDelta());
			
		usleep(100000);
	}

	delete encoder;
	return 0;
}

