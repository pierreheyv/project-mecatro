#include<stdio.h>
#define N 3 //number of node

class Map
{
    int dist[N][N];//tableau des dist entre noeuds !!! il faut que tous les noeuds puissent retourner à la base de façon rectiligne (à priori pas de problème vu la map), base = node 0
    int node[N][5];//[0]: position x, [1]: position y, [2]: needed orientation (infinity if no orientation needed), [3]: gain/noeud, [4]: cout de l'action
};

class Path
{
    int nbNodeNotVisited;
    int visited[N];//1 if the node number i has been visited, 0 if not
    int obj[N];//list of node to follow from the actual nodes towards node 0
    int totalPpoints;//number of point expected with this path
    int timeleft;//[s]
    int actual_node;//where you are (physically or in simulation calculation)
    int objnb;//number of obj already passed (or added to the path for calculation)
};

Map mymap;//map to be initialized with nodes
//mymap->dist = ...;
//mymap->node = ...;
//difficulté : pour les manche à air je ne sais pas trop comment faire : mettre un seul noeud mais difficile ou mettre 2 noeuds mais on fait l'action entre les deux

Path mypath;

Path *initpath (Map mymap)//initialize the path (for the beginning when we are on the base node 0)
{
    //initialization
    mypath->timeleft = 100;//to be adapted
    mypath->totalPpoints = 0;
    mypath->nbNodeNotVisited = N-1;//base node visited
    mypath->objnb = 0;

    for(i=1; i< N; i++)
    {
        mypath->visited[i] = 0;
    }
    mypath->actual_node = 0;
    mypath->visited[0] = 1;

    return mypath;
}

Path* adaptpath(Map mymap, Path mypath)//function to call if need a new path (at the beginning or when a problem occurs)
{
    //reset of values that will be updated (needed for good working of pathplanning function)
    mypath->totalPpoints =0;
    mypath->objnb = 0;
    //mypath->timeleft = 100 - timepassed  //time left actualisation
    Path newpath;
    newpath->totalPpoints =0;

    //computation
    newpath = pathplanning(mymap, mypath, newpath);

    //adaptation
    mypath->obj = newpath->obj;
    mypath->totalPpoints = newpath->totalPpoints;
    mypath->timeleft = newpath->timeleft;
    return mypath;
}

Path* pathplanning(Map mymap, Path mypath, Path best_path)//return an object Path with the best path and total expected points in it (most of the rest variable are unusable/false)
{
    if (best_path->totalPpoints < mypath->totalPpoints || (best_path->totalPpoints == mypath->totalPpoints && best_path->timeleft < mypath->timeleft))
        best_path = mypath;
    //simulate a robot going trough all possibilities
    for(i=1; i< mypath->nbNodeNotVisited; i++)//test all nodes
    {
        mypath->visited[i] = 1;//set analysing node as visited
        if (mypath->visited[i] == 0 && (mymap->dist[mypath->actual_node][i] + mymap->dist[0][i]+ mymap->node[i][1]) < mypath->timeleft)//si le noeud n'a pas encore été visité et est atteignable à temps => si tps d'aller au noeud + tps d'action au noeud + temps de retour à la base est < temps restant
        {
            mypath->timeleft = mypath->timeleft - (dist[0][i]+ mymap->node[i][4]); //timeleft update
            mypath->totalPoints = mypath->totalPpoints + mymap->node[i][3]; //point number update
            mypath->actual_node = i;//actual node for calculation
            mypath->obj[newpath->objnb] = i;//node added to the path obj list
            mypath->objnb++;
            mypath->nbNodeNotVisited--;
            if (mypath->nbNodeNotVisited>1)
                mypath = pathplanning(mymap, mypath, best_path); //next step
        }
    }
    return best_path;
}
