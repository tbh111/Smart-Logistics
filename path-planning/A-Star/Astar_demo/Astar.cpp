//
//  Astar.cpp
//  Astar
//
//  Created by 童博涵 on 2021/2/21.
//

#include "Astar.hpp"

Node::Node()
{
    flag = 0;
    value_h = 0;
    value_g = 0;
    value_f =  0;
    parent = NULL;
}

void addNode2Open(OpenList* openlist, Node* node)
{
    if(openlist == NULL)
    {
        cout<<"no data in openlist!"<<endl;
        return;
    }
    if(node->flag != STARTPOINT)
    {
        node->flag = INOPEN;
    }
    OpenList* temp = new OpenList;
    temp->next = NULL;
    temp->opennode = node;

//    if(openlist->next==NULL)
//    {openlist->next = temp;return;}

    while(openlist->next != NULL)
    {
        if(node->value_f < openlist->next->opennode->value_f)
        {
            OpenList* tempadd = openlist->next;
            temp->next = tempadd;
            openlist->next = temp;
            break;
        }
        else
            openlist = openlist->next;
    }
    openlist->next = temp;
}

void addNode2Close(CloseList* close, OpenList* &open)
{
    if(open == NULL)
    {
        cout<<"no data in openlist!"<<endl;
        return;
    }
    if(open->opennode->flag != STARTPOINT)
        open->opennode->flag = INCLOSE;

    if(close->closenode == NULL)
    {
        close->closenode = open->opennode;
        OpenList* tempopen = open;
        open = open->next;
        //open->opennode=NULL;
    //    open->next=NULL;
        delete tempopen;
        return;
    }
    while(close->next != NULL)
        close = close->next;

    CloseList* temp = new CloseList;
    temp->closenode = open->opennode;
    temp->next = NULL;
    close->next = temp;

    OpenList* tempopen = open;
    open = open->next;
    delete tempopen;
}



Astar::Astar() {
    steps=0;
    startpoint_x = -1;
    startpoint_y = -1;
    endpoint_y = -1;
    endpoint_x = -1;
}

Astar::~Astar() {
    
}

void Astar::findDestination(OpenList* open,CloseList* close, char data[][WIDE])
{
    insert2OpenList(open,startpoint_x,startpoint_y);// 起点
    addNode2Close(close,open);// 起点放到 close中
    //OpenList* temp=findMinInOpen(open);
    while(!insert2OpenList(open, open->opennode->location_x, open->opennode->location_y))
    {
        addNode2Close(close,open);
        if(open == NULL)
        {
            cout<<"unable to locate exit"<<endl;
            return;
        }
    }
    Node *tempnode = &m_node[endpoint_x][endpoint_y];
    while(tempnode->parent->flag != STARTPOINT)
    {
        tempnode = tempnode->parent;
        data[tempnode->location_x][tempnode->location_y] = '@';
    }
//    m_node;
}
// 在openlist中找到最小的 f值  节点
OpenList* Astar::findMinInOpen(OpenList* open)
{
    return open;
}
//////////////////////////////////////////////////////////////////////////
//  将临近的节点加入 openlist中
//                0      1      2
//                3      S      4
//                5      6      7
/////////////////////////////////////////////////////////////////////////////
bool Astar::insert2OpenList(OpenList* open,int center_x, int center_y)
{
    for(int i = 0; i < 8 ; i++)
    {
        int new_x = center_x + direction[i][0];
        int new_y = center_y+ direction[i][1];

        if(new_x >= 0 && new_y >= 0 && new_x<LENGTH && new_y<WIDE && isAccessible(open, new_x,new_y))// 0
        {
            if (m_node[new_x][new_y].flag == ENDPOINT)
            {
                m_node[new_x][new_y].parent = &m_node[center_x][center_y];
                return true;
            }
            m_node[new_x][new_y].flag = INOPEN;
            m_node[new_x][new_y].parent = &m_node[center_x][center_y];
            m_node[new_x][new_y].value_h = distanceManhattan(endpoint_x, endpoint_y, new_x,new_y);//曼哈顿距离

            if(0==i || 2==i||5==i||7==i)
                m_node[new_x][new_y].value_g = m_node[center_x][center_y].value_g+14;
            else
                m_node[new_x][new_y].value_g = m_node[center_x][center_y].value_g+10;

            m_node[new_x][new_y].value_f = m_node[new_x][new_y].value_g+m_node[new_x][new_y].value_h;

            addNode2Open(open, &m_node[new_x][new_y]);// 加入到 openlist中
        }
    }
    isChangeParent(open, center_x,  center_y);
    //if(counts>1000)
    //    return true;
    //else
    return false;
}
// 是否有更好的路径
void Astar::isChangeParent(OpenList* open,int center_x, int center_y)
{
    for(int i = 0; i < 8 ; i++)
    {
        int new_x = center_x + direction[i][0];
        int new_y = center_y + direction[i][1];
        if(new_x>=0 && new_y>=0 && new_x<LENGTH && new_y<WIDE && isInOpenList(open, new_x, new_y))// 0
        {
            if(0 == i || 2 == i || 5 == i || 7 == i)
            {
                if(m_node[new_x][new_y].value_g > m_node[center_x][center_y].value_g+14)
                {
                    m_node[new_x][new_y].parent = &m_node[center_x][center_y];
                    m_node[new_x][new_y].value_g = m_node[center_x][center_y].value_g+14;
                }
            }
            else
            {
                if(m_node[new_x][new_y].value_g > m_node[center_x][center_y].value_g+10)
                {
                    m_node[new_x][new_y].parent = &m_node[center_x][center_y];
                    m_node[new_x][new_y].value_g = m_node[center_x][center_y].value_g+10;
                }
            }
        }
    }
}

bool Astar::isAccessible(OpenList* open, int x, int y)
{
    if(isInOpenList(open, x, y))
        return false;
    if(isInCloseList(open, x, y))
        return false;
    if(m_node[x][y].flag == WALL )
        return false;
    else
        return true;
}
bool Astar::isInOpenList(OpenList* openlist, int x, int y)
{
    if(m_node[x][y].flag == INOPEN)
        return true;
    else
        return false;
}

bool Astar::isInCloseList(OpenList* openlist, int x, int y)
{
    if(m_node[x][y].flag == INCLOSE || m_node[x][y].flag==STARTPOINT)
        return true;
    else
        return false;
}
//显示地图
void displayMap(char data[][WIDE])
{
    for(int i=0; i< LENGTH ;i++)
    {
        for(int j=0; j<WIDE; j++)
            cout<<data[i][j];
        cout<<endl;
    }
}
unsigned int Astar::distanceManhattan(int d_x, int d_y, int x, int y)
{
    unsigned int temp = (abs(d_x - x) + abs(d_y-y))*DISTANCE;
    return temp;
}
//初始化 node
void Astar::initNodeMap(char data[][WIDE], OpenList * openlist)
{
    ifstream fin;
    fin.open("starmap.txt");
    if(!fin.is_open())
    {
        cout << "Failed to open file!" << endl;
        return ;
    }
    for(int i = 0; i < LENGTH; i++)
    {
        for(int j = 0; j < WIDE; j++)
        {
            fin >> data[i][j];
            m_node[i][j].location_x = i;
            m_node[i][j].location_y = j;
            m_node[i][j].parent = NULL;
            switch(data[i][j])
            {
            case '.':
                m_node[i][j].flag = ACCESSIBLE;
                break;
            case 'x':
                m_node[i][j].flag = WALL;
                break;
            case 's':
                m_node[i][j].flag = STARTPOINT;
                openlist->next = NULL;
                openlist->opennode = &m_node[i][j];//  将起点放到 OPenList中
                startpoint_x = i;
                startpoint_y = j;
                break;
            case 'd':
                m_node[i][j].flag = ENDPOINT;
                endpoint_x = i;
                endpoint_y = j;
                break;
            }
        }
        fin.get();
    }
    //cout<<data[startpoint_x][startpoint_y]<<endl;
    fin.close();
}
