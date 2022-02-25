#include <iostream>
#include <math.h>

class Node
{
    public :
    double x;
    double y;
    vector<double> m_pathX;
    vector<double> m_pathY;
    Node* m_parent;
    double cost; 
    Node (double x_,double y_ );
    ~Node();
};

