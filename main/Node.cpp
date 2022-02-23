#include "Node.h"

Node::Node( double x_, double y_ ) {
    x = x_;
    y = y_;
    m_parent = NULL;
    cost=0;
  }

 Node::~Node(){}