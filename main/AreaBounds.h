#include <iostream>
#include <math.h>

class AreaBounds {
    float xmin;
    float xmax;
    float ymin;
    float ymax;


  public :
   AreaBounds ( const osg :: Vec2f & area ) : xmin ( osg :: Vec2f :: NEGATIVE_INFINITY ), xmax ( osg :: Vec2f :: NEGATIVE_INFINITY ), ymin ( osg :: Vec2f :: NEGATIVE_INFINITY ), ymax ( osg :: Vec2f :: NEGATIVE_INFINITY ) {
    setBound ( osg :: Vec2f ( ), osg :: Vec2f ( ), osg :: Vec2f ( ) );
  }
};