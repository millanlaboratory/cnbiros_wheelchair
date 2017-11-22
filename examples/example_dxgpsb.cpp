#include <iostream>
#include <fstream>
#include <cmath>

#include "cnbiros_wheelchair/DxgpsbThread.hpp"

using namespace std;

/* Return a single byte representation of a float in the range [-1,1] */
char float2byte(float f){
	int i;
	
	f = min(max(f, (float) -1.0), (float)1.0);
	
	if (f == 1.0){
		i = 127;
	} else {
		i = floor((f * 128.0));
	}
	
	return (char) i;
}


/* Return a float representation of a byte in the range [-1,1] */
float byte2float(char b){
	float f;
	int i = b;
	
	f = i / 127.0;
	
	f = min(max(f, (float) -1.0), (float)1.0);
	
	return f;
	
}

int main(int argc, char* argv[]) {
	std::cout << "Invacare/Dynamics/EPFL dx-gpsb test program, luca.tonin@epfl.ch 2017" << std::endl;
  	
	cnbiros::wheelchair::DxgpsbThread* dxgpsbt;
  	float v, w;
	
	try { 
		if (argc == 2){
  		  dxgpsbt = new cnbiros::wheelchair::DxgpsbThread(argv[1]);
  		} else {
  		  dxgpsbt = new cnbiros::wheelchair::DxgpsbThread();
  		}
	} catch (std::logic_error& e) {
		std::cout<<"error code: "<<e.what()<<std::endl;
	}

  	bool runloop = true;
  	  
  	float RL_Value = 0.0;
  	float FR_Value = 0.0;
  	v = 0;
  	w = 0;
  	
  	char buf[3];
	
	while (runloop) {

		if ( v < -1.0) {
			v = -1.0;
		} else if (v > 1.0){
			v = 1.0;
		}

		if ( w < -1.0) {
			w = -1.0;
		} else if (w > 1.0){
			w = 1.0;
		}
		
		FR_Value = v;
		RL_Value = w;
		
		dxgpsbt->setVelocities(v, w);
		
		std::cout << "Please enter a velocity pair (v w) [-1.0, 1.0]: ";
     	std::cin >> v >> w;
 
	}
  
	delete dxgpsbt;
	
  return 0;
}
