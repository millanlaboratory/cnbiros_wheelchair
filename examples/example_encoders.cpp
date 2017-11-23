/* CNBI Wheelencoder test program
 * 
 * 15.09.2010, tom.carlson@epfl.ch 
 *
 */

//#include "CNBIWheelEncoder.h"
#include "CNBIWheelEncoderThread.h"

int main( int argc, char *argv[] ) 
{
	double dl = 0.0, dr = 0.0;
	
	
	printf("CNBI wheel encoder test program\n");
  
	CNBIWheelEncoderThread encright("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A700eiPG-if00-port0");
	CNBIWheelEncoderThread encleft("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A700eiPs-if00-port0");
	
	for (;;){
 		dl = encleft.getDelta();
 		dr = encright.getDelta();
		printf("Delta (left, right) ( % 3.0f, % 3.0f )\n", dl, dr);
			
		//printf("%4f\n", enc_thread1.getDelta());
			
		usleep(100000);
	}
	
	return 0;
}

