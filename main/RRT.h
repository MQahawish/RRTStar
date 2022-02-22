#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <list>
#include "Node.h"
#include "Obstacle.h"
#include "AreaBounds.h"

using namespace std;

class RRTstar {
    public :
    Node start;
    Node goal;
    vector<Obstacle> obstacle_list;
    vector<Node> node_list;
    double rand_area;
    double expand_dis;
    double path_resolution;
    double goal_sample_rate;
    int max_iter;
    Area play_Area;
    

   RRTstar( Node  start, Node  goal,  vector<Obstacle> obstacles, double rand_area, double expand_dis, double path_resolution , double goal_sample_rate, int max_iter, Area play_area);
   ~RRTstar();
   void Planning();
   Node steer(Node A,Node B);
   vector<Node> generate_final_course(int goal_ind);
   float calc_dist_to_goal(int x,int y);
   Node get_random_node();
   int get_nearest_node_index(vector<Node> nodes,Node ran_Node);
   bool check_if_outside_play_area(Node node,Area playArea);
   bool check_collision(Node node,vector<Obstacle> obstacle_list);
   float calc_distance(Node from,Node to);
   float calc_angle(Node from,Node to);
   Node* find_near_nodes(Node node);
   void rewire(Node node,vector<int> near_nodes_ind);
   int search_best_goal_node();
   void generate_final_course();
   Node choose_parent(Node new_node,Node* near_nodes);
   int calc_new_cost(Node from,Node to);
   void propagate_cost_to_leaves(Node parent);
};