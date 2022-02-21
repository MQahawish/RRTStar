#include <iostream>
#include <math.h>
#include "Node.h"
#include "AreaBounds.h"

class RRTstar {
    public :
    Node start;
    Node goal;
    Node* obstacle_list;
    double rand_area;
    double expand_dis;
    double path_resolution;
    double goal_sample_rate;
    int max_iter;
    Area play_Area;
    


   RRTstar( Node  start, Node  goal,  Node*  obstacles, double rand_area, double expand_dis, double path_resolution , double goal_sample_rate, int max_iter, int play_area[4] );
   ~RRTstar();
   void Planning();
   Node steer();
   Node* generate_final_course(int goal_ind);
   float calc_dist_to_goal(int x,int y);
   Node get_random_node();
   int get_nearest_node_index(Node* nodes,Node ran_Node);
   bool check_if_outside_play_area(Area playArea);
   bool check_collision(Node node,Node* obstacle_list);
   float calc_distance_and_angle(Node from,Node to);
   Node* find_near_nodes(Node node);
   void rewire(Node node,Node* nodes);
   int search_best_goal_node();
   void generate_final_course();
   Node choose_parent(Node new_node,Node* near_nodes);
   int calc_new_cost(Node from,Node to);
   void propagate_cost_to_leaves(Node parent);
};