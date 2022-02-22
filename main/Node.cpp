#include "Node.h"

Node::Node( int x_, int y_ ) {
    x = x_;
    y = y_;
    m_pathX = NULL;
    m_pathY = NULL;
    m_parent = NULL;
    cost=0;
  }

 Node::~Node(){}