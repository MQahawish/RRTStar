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
    double connect_circle_dist;
    bool search_until_max_iter;
    

   RRTstar( Node  start, Node  goal,  vector<Obstacle> obstacles, double rand_area, double expand_dis, double path_resolution , double goal_sample_rate, int max_iter, Area play_area,double connect_circle_dist,bool search_until_max_iter);
   ~RRTstar();
   void Planning();
Node* steer(Node A,Node B,double extend_length=std::numeric_limits<float>::max()); //+
   vector<Node> generateFinalCourse(int goal_ind);  //+
   float calcDistToGoal(int x,int y);  //+
   Node* getRandomNode();
   int getNearestNodeIndex(vector<Node> nodes,Node ran_Node);  //+
   bool checkIfOutsidePlayArea(Node node,Area playArea);  //+
   bool checkCollision(Node node,vector<Obstacle> obstacle_list);
   float calcDistance(Node from,Node to);  //+
   float calcAngle(Node from,Node to);     //+
   vector<double> findNearNodes(Node node);  //+ 
   void rewire(Node node,vector<int> near_nodes_ind); //+
   int searchBestGoalNode();     //+
   void generateFinalCourse();   //+
   Node* chooseParent(Node new_node,vector<double> near_node_inds);  //+
   int calcNewCost(Node from,Node to);   //+
   void propagateCostToLeaves(Node parent); //+
};