#include <iostream>
#include <math.h>
#include <vector>
#include <iostream>


class Node
{
    public :
    double x;
    double y;
    double parent_index;
    double cost; 
    Node (double x_,double y_ );
    Node();
    ~Node();
    friend std::ostream& operator<<(std::ostream& os, const Node& node);
    Node& operator= (const Node& node);
    friend bool operator==(const Node& lhs, const Node& rhs);
};


