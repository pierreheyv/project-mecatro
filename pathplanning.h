#ifndef PATHPLANNING_H_INCLUDED
#define PATHPLANNING_H_INCLUDED

#include<stdio.h>
#include <stdlib.h>
#include "struct.h"

#define N 18 //number of node

typedef struct Path{
    int nbNodeNotVisited;
    int visited[N];//1 if the node number i has been visited, 0 if not
    int obj[N];//list of node to follow from the actual nodes towards node 0
    int totalPoints;//number of point expected with this path
    int timeleft;//[s]
    int actual_node;//where you are (physically or in simulation calculation)
    int objnb;//number of obj already passed (or added to the path for calculation)
} Path;


typedef struct Map //contient la carte + le path
{
    int dist[N][N];//tableau des dist entre noeuds !!! il faut que tous les noeuds puissent retourner à la base de façon rectiligne (à priori pas de problème vu la map), base = node 0
    int node[N][5];//[0]: position x, [1]: position y, [2]: needed orientation (infinity if no orientation needed), [3]: nb de points qu'il y moy de se faire à ce noeudnoeud, [4]: cout de l'action (tps)
    Path* mypath;
} Map;

Path* initpath (Map* mymap);
Path* adaptpath(Map* mymap, Path* mypath);
Path* pathplanning(Map* mymap, Path* mypath, Path* best_path);
Map* initmap();



#endif // PATHPLANNING_H_INCLUDED
