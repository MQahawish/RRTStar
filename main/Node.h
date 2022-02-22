#include <iostream>
#include <math.h>

class Node
{
    public :
    int x;
    int y;
    Node* m_pathX;
    Node* m_pathY;
    Node* m_parent;
    int cost; 
    Node ( int x_, int y_ );
    ~Node();
};

