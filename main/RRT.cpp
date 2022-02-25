#include "RRT.h"

RRTstar::RRTstar(Node start_, Node goal_, vector<Obstacle> obstacles_, double rand_area_max_, double rand_area_min_, double expand_dis_ = 3, double path_resolution_ = 0.5, double goal_sample_rate_ = 5, int max_iter_ = 500, Area play_area_ = Area(0, 0, 0, 0, 1), double connect_circle_dist_ = 50.0, bool search_until_max_iter_ = false)
{
   start = start_;
   goal = goal_;
   obstacle_list = obstacles_;
   rand_area_max = rand_area_max_;
   rand_area_min = rand_area_min_;
   expand_dis = expand_dis_;
   path_resolution = path_resolution_;
   goal_sample_rate = goal_sample_rate_;
   max_iter = max_iter_;
   play_Area = play_area_;
   connect_circle_dist = connect_circle_dist_;
   search_until_max_iter = search_until_max_iter_;
}

double RRTstar::calcDistance(Node from, Node to)
{
   double dx = to.x - from.x;
   double dy = to.y - from.y;
   return hypot(dx, dy);
}

double RRTstar::calcAngle(Node from, Node to)
{
   double dx = to.x - from.x;
   double dy = to.y - from.y;
 //  double angle = atan2(dx, dy);
   return atan2(dx, dy);
}

double RRTstar::calcDistToGoal(int x, int y)
{
   double dx = goal.x - x;
   double dy = goal.y - y;
   return hypot(dx, dy);
}

int RRTstar::calcNewCost(Node from, Node to)
{
   return from.cost + calcDistance(from, to);
}

bool RRTstar::checkCollision(Node node)
{
   vector<double> dist;
   vector<double> x_dist;
   vector<double> y_dist;
   for (auto obstacle : obstacle_list)
   {
      for (double x : node.m_pathX)
         x_dist.push_back(obstacle.x - x);
     
      for (double y : node.m_pathY)
         y_dist.push_back(obstacle.y - y);
      
      for (int i = 0; i < node.m_pathX.size(); i++)
         dist.push_back(x_dist[i] * x_dist[i] + y_dist[i] * y_dist[i]);
      
      double min = 0;
      for (double v : dist)
         if (v < min)
            min = v;
   
      if (min < obstacle.size * obstacle.size)
         return false;
      x_dist.clear();
      y_dist.clear();
      dist.clear();
   }
   return true;
}

bool RRTstar::checkIfOutsidePlayArea(Node node, Area playArea)
{
   return !(node.x < playArea.xmin || node.y < playArea.ymin || node.x > playArea.xmax || node.y > playArea.ymax);
}

Node RRTstar::getRandomNode()
{
   srand(time(NULL));
   int rand_num = rand() % 100;
   double big = rand_area_max - rand_area_min + 1;
   Node rand_node(0, 0);
   if (rand_num > goal_sample_rate)
   {
      double x = fmod(rand(),rand_area_max - rand_area_min + 1) + rand_area_min;
      double y = fmod(rand(),rand_area_max - rand_area_min + 1) + rand_area_min;
      rand_node.x = x;
      rand_node.y = y;
   }
   else
   {
      rand_node.x = goal.x;
      rand_node.y = goal.y;
   }
   return rand_node;
}

int RRTstar::getNearestNodeIndex(vector<Node> nodes, Node ran_Node)
{
   double dist = 0, index = -1, cnt = 0;
   for (auto node : nodes)
   {
      double currnetDist = calcDistance(node, ran_Node);
      if (currnetDist < dist)
      {
         dist = currnetDist;
         index = cnt;
      }
      cnt++;
   }
   return index;
}

vector<Node>* RRTstar::generateFinalCourse(int goal_ind)
{
   vector<Node> path;
   Node node = node_list[goal_ind];
   while (node.m_parent)
   {
      path.push_back(node);
      node = *(node.m_parent);
   }
   path.push_back(node);
   vector<Node>* ptrPath = &path;
   return ptrPath;
}

void RRTstar::propagateCostToLeaves(Node parent)
{
   Node *ptrParent = &parent;
   for (auto node : node_list)
   {
      if (node.m_parent == ptrParent)
      {
         node.cost = calcNewCost(parent, node);
         propagateCostToLeaves(node);
      }
   }
}

void RRTstar::rewire(Node new_node, vector<int> near_nodes_ind)
{
   for (int index = 0; index < near_nodes_ind.size(); index++)
   {
      Node near_node = node_list[near_nodes_ind[index]];
      Node *edge_node = steer(new_node, near_node);
      edge_node->cost = calcNewCost(new_node, near_node);
      bool no_collision = checkCollision(*edge_node);
      bool improved_cost = near_node.cost > edge_node->cost;
      if (no_collision && improved_cost)
      {
         near_node.x = edge_node->x;
         near_node.y = edge_node->y;
         near_node.cost = edge_node->cost;
         near_node.m_pathX = edge_node->m_pathX;
         near_node.m_pathY = edge_node->m_pathY;
         near_node.m_parent = edge_node->m_parent;
         propagateCostToLeaves(new_node);
      }
   }
}

int RRTstar::searchBestGoalNode()
{
   vector<double> distToGoalList;
   vector<double> goalInds;
   vector<double> safeGoalInds;
   for (auto node : node_list)
   {
      distToGoalList.push_back(calcDistToGoal(node.x, node.y));
   }
   for (int i = 0; i < distToGoalList.size(); i++)
   {
      if (distToGoalList[i] < expand_dis)
         goalInds.push_back(i);
      
   }
   for (auto goalInd : goalInds)
   {
      Node *t_node = steer(node_list[goalInd], goal);
      if (checkCollision(*t_node))
         safeGoalInds.push_back(goalInd);
   }
   if (safeGoalInds.size() == 0)
      return -1;

   double min_cost = 0;
   for (auto safeInd : safeGoalInds)
   {
      if (node_list[safeInd].cost < min_cost)
         min_cost = node_list[safeInd].cost;
   }
   for (auto safeInd : safeGoalInds)
   {
      if (node_list[safeInd].cost == min_cost)
         return safeInd;
   }
   return -1;
}

Node* RRTstar::chooseParent(Node new_node, vector<double> near_node_inds)
{
   if (near_node_inds.size() == 0)
      return NULL;
   vector<double> costs;
   for (auto ind : near_node_inds)
   {
      Node near_node = node_list[ind];
      Node *t_node = steer(new_node, near_node);
      if (t_node && checkCollision(*t_node))
      {
         costs.push_back(calcNewCost(near_node, *t_node));
      }
      else
      {
         costs.push_back(std::numeric_limits<double>::max());
      }
   }
   double min_cost = 0, index;
   for (int i = 0; i < costs.size(); i++)
      if (costs[i] < min_cost)
      {
         min_cost = costs[i];
         index = i;
      }

   if (min_cost == std::numeric_limits<double>::max())
      return NULL;

   double min_ind = near_node_inds[index];
   Node *new_n = steer(node_list[min_ind], new_node);
   new_n->cost = min_cost;
   return new_n;
}

vector<double> RRTstar::findNearNodes(Node new_node)
{

   double numNodes = node_list.size();
   double r = connect_circle_dist * sqrt((log(numNodes) / numNodes));
   r = r < expand_dis ? r : expand_dis;
   vector<double> distances;
   vector<double> near_inds;
   double index = 0;
   for (auto node : node_list)
      distances.push_back((node.x - new_node.x)*(node.x - new_node.x) + (node.y - new_node.y)*(node.y - new_node.y));
   for (auto distance : distances)
   {
      if (distance < r ^ 2)
      {
         near_inds.push_back(index);
      }
      index++;
   }
   return near_inds;
}

Node* RRTstar::steer(Node from, Node to, double extend_length = std::numeric_limits<double>::max())
{
   Node new_node(from.x, from.y);
   double distance = calcDistance(new_node, to);
   double angle = calcAngle(new_node, to);
   new_node.m_pathX.push_back(new_node.x);
   new_node.m_pathY.push_back(new_node.y);
   extend_length = extend_length > distance ? distance : extend_length;
   double n_expand = floor(extend_length / path_resolution);
   for (int i = 0; i < n_expand; i++)
   {
      new_node.x += path_resolution * cos(angle);
      new_node.y += path_resolution * sin(angle);
      new_node.m_pathX.push_back(new_node.x);
      new_node.m_pathY.push_back(new_node.y);
   }
   distance = calcDistance(new_node, to);
   if (distance < path_resolution)
   {
      new_node.m_pathX.push_back(to.x);
      new_node.m_pathY.push_back(to.y);
      new_node.x = to.x;
      new_node.y = to.y;
   }
   Node *ptrFrom = &from;
   new_node.m_parent = ptrFrom;
   Node *ptrNewNode = &new_node;
   return ptrNewNode;
}
 
vector<Node>* RRTstar::Planning(){
   node_list.push_back(start);
   double i=0;
   while(i<max_iter)
   {
      Node rnd_node=getRandomNode();
      double nearest_ind=getNearestNodeIndex(node_list,rnd_node);
      Node nearest_node=node_list[nearest_ind];
      Node* new_node = steer(nearest_node,rnd_node,expand_dis);
      if(checkCollision(*new_node)&&checkIfOutsidePlayArea(*new_node,play_Area)){
         node_list.push_back(*new_node);
      }
      if(calcDistToGoal(node_list.back().x,node_list.back().y)<=expand_dis)
      {
         Node* final_node=steer(node_list.back(),goal,expand_dis);
         if(checkCollision(*final_node))
         {
            return generateFinalCourse(node_list.size()-1);
         }
      }
      i++;
   }
   return NULL;
}

