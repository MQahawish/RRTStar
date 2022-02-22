#include "RRT.h"

RRTstar::RRTstar(Node start_, Node goal_, vector<Obstacle> obstacles_, double rand_area_, double expand_dis_=3, double path_resolution_=0.5 , double goal_sample_rate_=5, int max_iter_=500, Area play_area_=Area(0,0,0,0,1))
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

   float RRTstar::calc_distance(Node from,Node to)
   {
      float dx = to.x-from.x;
      float dy = to.y-from.x;
      float d = hypot(dx,dy);
      return d;
   }
   
   float RRTstar::calc_angle(Node from,Node to)
   {
      float dx = to.x-from.x;
      float dy = to.y-from.x;
      float angle = atan2(dx,dy);
      return angle;
   }
   
   //not done
   bool RRTstar::check_collision(Node node,vector<Obstacle> obstacle_list){
        for(int i=0;i<obstacle_list.size();i++)
        {

        }
   }
   
   bool RRTstar::check_if_outside_play_area(Node node,Area playArea){
       if(node.x<playArea.xmin || node.y<playArea.ymin || node.x > playArea.xmax || node.y > playArea.ymax )
           return false;
       else
           return true;
   }
   
   //not done
   Node RRTstar::get_random_node(){

   }
   
   float RRTstar::calc_dist_to_goal(int x,int y)
   {
      float dx= goal.x-x;
      float dy= goal.y-y;
      return hypot(dx,dy);
   }
   
   vector<Node> RRTstar::generate_final_course(int goal_ind){

   }
   
   int RRTstar::get_nearest_node_index(vector<Node> nodes,Node ran_Node){
      float dist=0,index;
      for(int i=0;i<nodes.size();i++)
      {
         if(calc_distance(nodes[i],ran_Node)<dist)
         {
            dist=calc_distance(nodes[i],ran_Node);
            index=i;
         }
      }
      return index;
   }
   
   vector<Node> RRTstar::generate_final_course(int goal_ind){
      vector<Node> path;
      Node node=node_list[goal_ind];
      while(node.m_parent)
      {
         path.push_back(node);
         node=*(node.m_parent);
      }
      path.push_back(node);
      return path;
   }
   
   int RRTstar::calc_new_cost(Node from,Node to){
      float dist=calc_distance(from,to);
      return from.cost+dist;
   }
   
   //not done
   void RRTstar::propagate_cost_to_leaves(Node parent){
      for(int i=0;i<node_list.size();i++)
      {
         if(*node_list[i].m_parent==parent)
         {

         }
      }
   }
   
   void RRTstar::rewire(Node new_node,vector<int> near_nodes_ind){
      for(int index=0;index<near_nodes_ind.size();index++)
      {
         Node near_node = node_list[near_nodes_ind[index]];
         Node edge_node = steer(new_node,near_node);
         edge_node.cost = calc_new_cost(new_node,near_node);
         bool no_collision = check_collision(edge_node,obstacle_list);
         bool improved_cost = near_node.cost > edge_node.cost;
         if(no_collision&&improved_cost)
         {
                near_node.x = edge_node.x;
                near_node.y = edge_node.y;
                near_node.cost = edge_node.cost;
                near_node.m_pathX = edge_node.m_pathX;
                near_node.m_pathY = edge_node.m_pathY;
                near_node.m_parent = edge_node.m_parent;
                propagate_cost_to_leaves(new_node);
         }
      }
   }


