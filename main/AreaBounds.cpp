 #include "AreaBounds.h"
 
  Area::Area(double xmin_,double xmax_,double ymin_,double ymax_,bool Free_)
  {
    Free=Free_;
    if(Free)
    {
      xmax=ymax= std::numeric_limits<double>::max();
      xmin=ymin= std::numeric_limits<double>::min();
    }
    else
    {
      xmin=xmin_;
      xmax=xmax_;
      ymin=ymin_;
      ymax=ymax_;
    }
  }