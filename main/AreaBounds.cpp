 #include "AreaBounds.h"
 
  Area::Area(float xmin_,float xmax_,float ymin_,float ymax_,bool Free_)
  {
    Free=Free_;
    if(Free)
    {
      xmax=ymax= std::numeric_limits<float>::max();
      xmin=ymin= std::numeric_limits<float>::min();
    }
    else
    {
      xmin=xmin_;
      xmax=xmax_;
      ymin=ymin_;
      ymax=ymax_;
    }
  }