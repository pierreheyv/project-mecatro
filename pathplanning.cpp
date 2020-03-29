#include "pathplanning.h"

Path* initpath(Map *mymap){
	Path* mypath = (Path*) malloc(sizeof(Path));
    //initialization
    mypath->timeleft = 100;//to be adapted
    mypath->totalPoints = 0;
    mypath->nbNodeNotVisited = N-1;//base node visited
    mypath->objnb = 0;

    for(int i=1; i< N; i++)
    {
        mypath->visited[i] = 0;
    }
    mypath->actual_node = 0;
    mypath->visited[0] = 1;

    mypath = adaptpath(mymap, mypath);

    return mypath;
}

Map mymap;//map to be initialized with nodes
//mymap->dist = ...;
//mymap->node = ...;
//difficulté : pour les manche à air je ne sais pas trop comment faire : mettre un seul noeud mais difficile ou mettre 2 noeuds mais on fait l'action entre les deux

Path* adaptpath(Map* mymap, Path* mypath)//function to call if need a new path (at the beginning or when a problem occurs)
{
    //reset of values that will be updated (needed for good working of pathplanning function)
    mypath->totalPoints =0;
    mypath->objnb = 0;
    //mypath->timeleft = 100 - timepassed  //time left actualisation

    //computation
    Path* newpath;
    newpath = pathplanning(mymap, mypath, mypath);

    //adaptation
    *(mypath->obj) = *(newpath->obj);
    mypath->totalPoints = newpath->totalPoints;
    mypath->timeleft = newpath->timeleft;

    return mypath;
}

Path* pathplanning(Map* mymap, Path* mypath, Path* best_path)//return an object Path with the best path and total expected points in it (most of the rest variable are unusable/false)
{
    if (best_path->totalPoints < mypath->totalPoints || (best_path->totalPoints == mypath->totalPoints && best_path->timeleft < mypath->timeleft)) //to be changed
        best_path = mypath;
    //simulate a robot going trough all possibilities
    for(int i=1; i< N; i++)//test all nodes (could be smarter !!!)
    {
        if (mypath->visited[i] == 0 && (mymap->dist[mypath->actual_node][i] + mymap->dist[0][i]+ mymap->node[i][4]) < mypath->timeleft)//si le noeud n'a pas encore été visité et est atteignable à temps => si tps d'aller au noeud + tps d'action au noeud + temps de retour à la base est < temps restant
        {
            mypath->timeleft = mypath->timeleft - (mymap->dist[0][i]+ mymap->node[i][4]); //timeleft update
            mypath->totalPoints = mypath->totalPoints + mymap->node[i][3]; //point number update
            mypath->actual_node = i;//actual node for calculation
            mypath->visited[i] = 1;//set analysing node as visited
            mypath->obj[mypath->objnb] = i;//node added to the path obj list
            mypath->objnb++;
            mypath->nbNodeNotVisited--;
            if (mypath->nbNodeNotVisited>1)//to be tested
                mypath = pathplanning(mymap, mypath, best_path); //next step
        }
    }
    return best_path;
}

//map
Map* initmap(){
	Map* mymap = (Map*) malloc(sizeof(Map));
    //initialization des noeuds
    //... (Pierre-Yves)
    mymap->mypath = initpath(mymap);
    return mymap;
}

void readmap(Map* mymap) //function created to test but probably unuseful
{
    FILE *file;

    // use appropriate location if you are using MacOS or Linux
    file = fopen("map.txt","r");

    if(file == NULL)
    {
        printf("map.txt not openable");
        exit(1);
    }
    else
    {
        for(int i=0; i< N; i++)
        {
            fscanf(file, "%d %d %d %d %d", &mymap->node[i][0], &mymap->node[i][1], &mymap->node[i][2], &mymap->node[i][3], &mymap->node[i][4]);
            if (mymap->node[i][0] == NULL || mymap->node[i][1] == NULL || mymap->node[i][2] == NULL || mymap->node[i][3] == NULL || mymap->node[i][4] == NULL)
                printf("error reading line %d", i);
        }
    }

    fclose(file);
}
