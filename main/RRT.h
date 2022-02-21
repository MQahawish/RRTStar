#include <iostream>
#include <math.h>
#include <Node.h>

class RRT : public Node {
    public :
    int* start;
    int* goal;
    int* obstacle_list;
    double rand_area;
    double expand_dis;
    double path_resolution;
    double goal_sample_rate;
    int max_iter;
    int* play_area;
    


   RRT( int*  start, int*  goal,  int*  obstacles, double rand_area, double expand_dis, double path_resolution , double goal_sample_rate, int max_iter, int* play_area ) 
   {
    start=&start;
    goal=&goal;
    obstacles=& obstacles
       expand_dis = 3.0;
       path_resolution = 0.5;
       goal_sample_rate = 5;
       max_iter = 500;
       play_area = NULL;

   }
 
};