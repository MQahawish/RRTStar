#include <iostream>
#include <math.h>

class Area {
public:
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	bool Free; //flag 
	Area();
	Area(double xmin_, double xmax_, double ymin_, double ymax_, bool Free_); //Area constructor
};