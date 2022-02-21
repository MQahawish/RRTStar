#include "RRT.h"

RRTstar::RRTstar(Node start_, Node goal_, Node* obstacles_, double rand_area_, double expand_dis_=3, double path_resolution_=0.5 , double goal_sample_rate_=5, int max_iter_=500, Area play_area_=Area(0,0,0,0,1))
{
start=start_;
goal=goal_;
obstacle_list=obstacles_;
rand_area=rand_area_;
expand_dis=expand_dis_;
path_resolution=path_resolution_;
goal_sample_rate=goal_sample_rate_;
max_iter=max_iter_;
play_Area=play_area_;
}

