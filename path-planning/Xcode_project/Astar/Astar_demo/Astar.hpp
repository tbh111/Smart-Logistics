//
//  Astar.hpp
//  Astar
//
//  Created by 童博涵 on 2021/2/21.
//

#ifndef Astar_hpp
#define Astar_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

const int DISTANCE = 10; // distance between two square
const int direction[8][2] = {{-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}}; // direction vector

enum{LENGTH = 51, WIDE = 50}; // map size
enum{ACCESSIBLE, WALL, INOPEN, INCLOSE, STARTPOINT, ENDPOINT}; // node's state

struct Node
{
    int flag; // state
    unsigned int location_x;
    unsigned int location_y;
    unsigned int value_h;
    unsigned int value_g;
    unsigned int value_f;
    Node* parent;
    Node();
};


struct OpenList
{
    Node *opennode;
    OpenList* next;
    OpenList(){next = NULL;};
};

struct CloseList
{
    Node *closenode;
    CloseList* next;
    CloseList(){next = NULL;};
};

void AddNode2Open(OpenList* openlist, Node* node);
void AddNode2Close(CloseList* close, OpenList* &open);

class Astar {
public:
    Astar();
    virtual ~Astar();
    Node m_node[LENGTH][WIDE];
    void initNodeMap(char data[][WIDE], OpenList *open);
    void findDestination(OpenList* open,CloseList* close, char data[][WIDE]);
    OpenList* findMinInOpen(OpenList* open);
    bool insert2OpenList(OpenList*, int x, int y);
    bool isInOpenList(OpenList*, int x, int y);
    bool isInCloseList(OpenList*, int x, int y);
    void isChangeParent(OpenList*, int x, int y);
    bool isAccessible(OpenList*, int x, int y);
    unsigned int distanceManhattan(int d_x, int d_y, int x, int y);
private:
    unsigned int steps;
    int startpoint_x;
    int startpoint_y;
    int endpoint_x;
    int endpoint_y;
};

#endif /* Astar_hpp */




