#include "Node.h"

Node::Node( int x, int y ) {
    m_x = x;
    m_y = y;
    m_pathX = NULL;
    m_pathY = NULL;
    m_parent = NULL;
  }

 Node::~Node(){}