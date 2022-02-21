#include <iostream>
#include <math.h>

class Node
{
    int m_x;
    int m_y;
    int* m_pathX;
    int* m_pathY;
    int* m_parent;

  public : Node ( int x, int y ) {
    m_x = x;
    m_y = y;
    m_pathX = &x;
    m_pathY = &y;
    m_parent = NULL;
  }

    Node(/* args */);
    ~Node();
};

