//
//  main.cpp
//  path-planning
//
//  Created by 童博涵 on 2021/1/21.
//

#include <iostream>
#include "dijkstra.hpp"
int main(int argc, const char * argv[]) {

    int n,m,st,ed;
    printf("input n m st ed\n");
    scanf("%d%d%d%d",&n, &m, &st, &ed);
    Graph_DG graph(n, m, st, ed);
    graph.initGraph();
    int dis = graph.Dijkstra();
    printf("the path is:");
    graph.DFS(ed,0);
    printf("\nmin distance is:%d\n", dis);
    
    
    return 0;
}
