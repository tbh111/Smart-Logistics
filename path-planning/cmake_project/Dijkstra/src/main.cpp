//
//  main.cpp
//  path-planning
//
//  Created by 童博涵 on 2021/1/21.
//

#include <iostream>
#include "dijkstra.hpp"
int main(int argc, const char * argv[]) {

    int n,m,st;
    printf("input n m st\n");
    scanf("%d%d%d",&n, &m, &st);
    Graph_DG graph(n, m, st);
    graph.initGraph();
    graph.Dijkstra();
    graph.printDis();

    return 0;
}

