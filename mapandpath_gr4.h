#ifndef MAPANDPATH_H_INCLUDED
#define MAPANDPATH_H_INCLUDED

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cmath>

#define BASENODE 1
#define NNODE 10//number of node in the map
#define NBOBST 27
#define GAMETIME 100

NAMESPACE_INIT(ctrlGr4);

typedef struct Path{
    int nbNodeNotVisited;
    int visited[NNODE];//1 if the node number i has been visited, 0 if not
    int obj[NNODE];//list of node to follow (dest list)
    double totalPoints;//number of point expected with this path
    double timeleft;//[s]
    int actual_node;//where you are (physically or in simulation calculation)
    int objnb;//number of obj in the path
    int nextNode;//number of the next node on the path
    int nextNodelnb;//number of the next node on the destlist
    int nbciblenode;//number of nodes with target added to the dest list
} Path;

typedef struct Obstacles{
    double posxy[2];
    double width;//width of the obstacle (including margin)
} Obstacles;


typedef struct Map //contient la carte + le most strategic path
{
    double dist[NNODE][NNODE];//tableau des dist entre noeuds (en temps) !!! node 0 = dépot
    double node[NNODE][5];//[0]: position x, [1]: position y, [2]: needed orientation (neg if no orientation needed), [3]: nb de points qu'il y moy de se faire à ce noeudnoeud, [4]: cout de l'action (tps)
    int obstaclesnb;
    double** obstacles;//
    Path* mypath;
} Map;

void initpath (Map* mymap);
void newpath(Map *mymap, double timepassed);
void update_destlist(Map* mymap, Path* mypath);
Map* initmap();
void readmap(Map* mymap);
void load_obstacles(Map* mymap);

NAMESPACE_CLOSE();

#endif // MAPANDPATH_H_INCLUDED
