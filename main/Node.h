#include <iostream>
#include <math.h>

class Node
{
    int m_x;
    int m_y;
    Node* m_pathX;
    Node* m_pathY;
    Node* m_parent;

  public : 
    Node ( int x, int y );
    ~Node();
};

