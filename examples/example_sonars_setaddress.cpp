#include "cnbiros_wheelchair/Sonar.hpp"

#include <iostream>

void usage( char *x ) {
	printf( "usage: %s /dev/ttyUSB0\n", x );
}

int main(int argc, char *argv[])
{
	std::string s;
	int retval, revision, err;
	unsigned char current_addr, new_addr;
	char msg[256];
	
	cnbiros::wheelchair::Sonar *sr;

	if ( argc == 2 ) {
		sr = new cnbiros::wheelchair::Sonar(argv[1]);
	} else {
		sr = new cnbiros::wheelchair::Sonar();
	}
	
	
	std::cout << "\nYou must only have ONE sonar connected to the I2C bus.\nEnter its current address: 0x";
	std::cin >> s;
	 
	current_addr = strtoul( s.c_str(), NULL, 16);
	sprintf(msg, "%#x", current_addr);
	//std::cout << msg << std::endl;
	
	err = sr->requestRangeCm(current_addr);
	usleep( 75000 );
	err += sr->requestRegisterValues(current_addr, true);
	revision = sr->getRevision();
	//std::cout << revision << std::endl;
	if (revision == 255 || err != 0){
		std::cout << "ERROR: Could not connect to SRF02 at address 0x" << s << std::endl;
		exit(-1);
	}
	
	std::cout << "Connected to SRF02 at address 0x" << s << ", revision = " << revision << std::endl;
	
	std::cout << "Enter new address : 0x";
	
	s.clear();
	std::cin >> s;
	new_addr = strtoul( s.c_str(), NULL, 16);
	sprintf(msg, "%#x", new_addr);
	sr->changeAddress(current_addr, new_addr);
	
	err = sr->requestRangeCm(new_addr);
	usleep( 75000 );
	err += sr->requestRegisterValues(new_addr, true);
	retval = sr->getRevision();
	
	if (retval == revision && err == 0){
		std::cout << "Successfully changed address of SRF02 to 0x" << s << std::endl;
	} else {
		std::cout << "ERROR: could not change address of SRF02 to 0x" << s << std::endl;
	}

	
	return 0;
	
}
