//
//  dijkstra.hpp
//  path-planning
//
//  Created by 童博涵 on 2021/1/21.
//

#ifndef dijkstra_hpp
#define dijkstra_hpp

#include <stdio.h>
#include <algorithm>
#endif /* dijkstra_hpp */
const int MAXV = 510;               //最大顶点数
const int INF = 1000000000;         //无穷大

class Graph_DG{
private:
    int n, m, st, G[MAXV][MAXV];    //n为顶点数，m为边数，st为起点，ed为终点
    int d[MAXV];                    //起点到达各点的最短路径长度
    bool vis[MAXV] = {false};       //标记数组，vis[i]==true表示已访问，初值为false
    int pre[MAXV];                  //表示从起点到顶点v的最短路径上v的前一个顶点
public:
    Graph_DG(int n, int m, int st);
    ~Graph_DG();
    void initGraph();
    void Dijkstra();
    void printDis();
    void DFS(int v, int count);
};

