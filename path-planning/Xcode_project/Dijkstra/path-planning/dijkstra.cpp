//
//  dijkstra.cpp
//  path-planning
//
//  Created by 童博涵 on 2021/1/21.
//

#include "dijkstra.hpp"
using namespace std;
Graph_DG::Graph_DG(int n, int m, int st){
    this->n = n;
    this->m = m;
    this->st = st;
    fill(this->G[0], this->G[0]+MAXV*MAXV, INF);    //初始化图G
}
Graph_DG::~Graph_DG(){
    //printf("Destructor called\n");
}
void Graph_DG::initGraph(){
    int u,v,w;
    printf("input src, dst, dis\n");
    for (int i = 0; i < m; i++) {
        scanf("%d%d%d", &u, &v, &w);
        G[u][v] = w;
        //G[v][u] = w;                              //无向图
    }
}
void Graph_DG::Dijkstra(){
    fill(d, d + MAXV, INF);                         //将整个d数组赋为INF
    for(int i = 0; i < n; i++){
        pre[i] = i;
    }
    d[st] = 0;                                      //起点到自身的距离为0
    for (int i = 0; i < n; i++) {
        int u = -1, MIN = INF;                      //使d[u]最小，MIN存放最小的d[u]
        for(int j = 0;j < n; j++){                  //找到未访问的顶点中d[]最小的
            if(vis[j] == false && d[j] < MIN){
                u = j;
                MIN = d[j];
            }
        }
        if(u == -1) break;                      //找不到小于INF的d[u]，剩下的顶点和起点不连通
        vis[u] = true;                              //标记u已访问
        for(int v = 0; v < n; v++){                 //v未访问&&u能到达v&&以u为中介点使d[v]更优
            if(vis[v] == false && G[u][v] != INF && d[u] + G[u][v] < d[v]){
                d[v] = d[u] + G[u][v];              //优化d[v]
                pre[v] = u;                         //记录v的前驱顶点是u
            }
        }
    }
    
}
void Graph_DG::printDis(){                          //输出所有顶点的最短距离
    for (int i = 0; i < n; i++) {
        printf("the path from %d to %d is:", st, i);
        DFS(i,0);
        if(d[i] == INF){
            printf(" distance = INF");
        }
        else{
            printf(" distance = %d", d[i]);
        }
        printf("\n");
    }
}
void Graph_DG::DFS(int v, int count){               //s为起点编号，v为当前访问的顶点编号（从终点开始递归）
    if(v == st){                                    //到达起点，输出并返回
        printf("%d ", st);
        return;
    }
    else if(d[v] == INF){
        printf("no path");
        return;
    }

    count++;
    DFS(pre[v], count);                             //递归访问v的前驱顶点
    printf("%d ", v);                               //输出每一层顶点
}
