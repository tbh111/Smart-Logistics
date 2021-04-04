//
//  main.cpp
//  Astar
//
//  Created by 童博涵 on 2021/2/21.
//

#include <iostream>
#include "Astar.hpp"
using namespace std;
int main(int argc, const char * argv[]) {
    Astar findpath;
    OpenList* openlist = new OpenList;
    CloseList* closelist = new CloseList;
    //closelist=NULL;
//    const OpenList* HEADOPEN = openlist;
//    const CloseList* HEADCLOSE = closelist;
    closelist->closenode = NULL;

    char data[LENGTH][WIDE] = {0};

    findpath.initNodeMap(data, openlist);
    findpath.findDestination(openlist, closelist, data);

    for(int i = 0; i < LENGTH; i++)
    {
        for(int j = 0; j < WIDE; j++)
        {
            cout << data[i][j];
        }
        cout << endl;
    }
    delete openlist;
    delete closelist;
    return 0;
}
