#ifndef PATHPLANNING_H_INCLUDED
#define PATHPLANNING_H_INCLUDED

#include<stdio.h>
#include <stdlib.h>
#include "struct.h"

#define N 7 //number of node
#define maxnbobstacles 10 //maximum number of obstacles

typedef struct Path{
    int nbNodeNotVisited;
    int visited[N];//1 if the node number i has been visited, 0 if not
    int obj[N];//list of node to follow from the actual nodes towards node 0
    double totalPoints;//number of point expected with this path
    double timeleft;//[s]
    int actual_node;//where you are (physically or in simulation calculation)
    int objnb;//number of obj already passed (or added to the path for calculation)
} Path;

typedef struct Obstacles{
    double posxy[2];
    double width; //width of the obstacle (including margin)
} Obstacles;


typedef struct Map //contient la carte + le path
{
    double dist[N][N];//tableau des dist entre noeuds (en temps) !!! il faut que tous les noeuds puissent retourner à la base de façon rectiligne (à priori pas de problème vu la map), base = node 0
    double node[N][5];//[0]: position x, [1]: position y, [2]: needed orientation (infinity if no orientation needed), [3]: nb de points qu'il y moy de se faire à ce noeudnoeud, [4]: cout de l'action (tps)
    int step;
    int obstaclesnb;
    Obstacles obstacles[maxnbobstacles];//better ways to do it but flemme
    Path* mypath;
} Map;

void initpath (Map* mymap);
void adaptpath(Map* mymap);
void pathplanning(Map* mymap, Path* mypath);
Map* initmap();
void readmap(Map* mymap);


#endif // PATHPLANNING_H_INCLUDED
