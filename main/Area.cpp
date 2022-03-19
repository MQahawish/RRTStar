#include "Area.h"
Area::Area() {
	Free = true;
	//checks if the area is not occupied
	if (Free)
	{
		xmax = ymax = std::numeric_limits<double>::max();
		xmin = ymin = std::numeric_limits<double>::min();
	}
}
Area::Area(double xmin_, double xmax_, double ymin_, double ymax_, bool Free_)
{
	Free = Free_;
	//checks if the area is not occupied
	if (Free)
	{
		xmax = ymax = std::numeric_limits<double>::max();
		xmin = ymin = std::numeric_limits<double>::min();
	}
	else
	{
		xmin = xmin_;
		xmax = xmax_;
		ymin = ymin_;
		ymax = ymax_;
	}
}