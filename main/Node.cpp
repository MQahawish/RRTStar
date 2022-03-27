#include "Node.h"
Node::Node() {
    x = 0;
    y = 0;  
    parent_index = -1;
    cost = 0;
}
Node::Node( double x_, double y_ ) {
    x = x_;
    y = y_;
    pathX.push_back(x);
    pathY.push_back(y);
    parent_index = -1;
    cost=0;
  }

 Node::~Node(){}

 std::ostream& operator<<(std::ostream& os, const Node& node)
 {;
     os << " x,y: " << node.x << "," << node.y << std::endl;
     return os;
 }

 Node& Node::operator= (const Node& node)
 {
     x = node.x;
     y = node.y;
     parent_index = node.parent_index;
     cost = node.cost;
     pathX=node.pathX;
     pathY=node.pathY;
     return *this;
 }

 bool operator==(const Node& left, const Node& right) {
     return (left.cost == right.cost) && (left.parent_index == right.parent_index)
         && left.x == right.x && left.y == right.y;
 }
