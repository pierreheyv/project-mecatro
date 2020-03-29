#include "pathplanning.h"

void initpath(Map *mymap)
{
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

    mymap->mypath = mypath;
    adaptpath(mymap);

    printf("-----------best path found : -------------\n");
}

Map mymap;//map to be initialized with nodes
//mymap->dist = ...;
//mymap->node = ...;
//difficulté : pour les manche à air je ne sais pas trop comment faire : mettre un seul noeud mais difficile ou mettre 2 noeuds mais on fait l'action entre les deux

void adaptpath(Map* mymap)//function to call if need a new path (at the beginning or when a problem occurs)
{
    //reset of values that will be updated (needed for good working of pathplanning function)
    mymap->mypath->totalPoints = 0;
    mymap->mypath->objnb = 0;
    //mypath->timeleft = 100 - timepassed  //time left actualisation
    mymap->step = 0;
    //printf("\n\ntotal points before begining %f\n\n\n", mymap->mypath->totalPoints);

    pathplanning(mymap, mymap->mypath);
}

void pathplanning(Map* mymap, Path* mypath)//return an object Path with the best path and total expected points in it (most of the rest variable are unusable/false)
{
    //printf("begin pathplanning with nb of node not visited = %d\n", mypath->nbNodeNotVisited);
    if (mypath->nbNodeNotVisited >= 1)
    {
        Path savepath = *mypath;//créer nouveau path avec même intérieur que mypath
        Path* psavepath = &savepath;//adresse de newpath

        for(int i=1; i< N; i++)//test all nodes (could be smarter !!!)
        {
            //printf("from node %d test to go to node %d\n", mypath->actual_node, i);

            if (mypath->visited[i] == 0)//si le noeud n'a pas encore été visité et est atteignable à temps => si tps d'aller au noeud + tps d'action au noeud + temps de retour à la base est < temps restant
            {
                Path newpath = *psavepath;//créer nouveau path avec même intérieur que mypath
                Path* pnewpath = &newpath;//adresse de newpath

                if ((mymap->dist[mypath->actual_node][i] + mymap->dist[0][i]+ mymap->node[i][4]) < mypath->timeleft)
                {
                    //printf("time cost %f \n", (mymap->dist[mypath->actual_node][i] + mymap->dist[0][i]+ mymap->node[i][4]));
                    //printf("time left %f \n", mypath->timeleft);
                    //printf("path taken to node %d \n", i);
                    newpath.timeleft = pnewpath->timeleft - (mymap->dist[0][i]+ mymap->node[i][4]); //timeleft update
                    newpath.totalPoints = (pnewpath->totalPoints + mymap->node[i][3]); //point number update
                    newpath.actual_node = i;//actual node for simulation
                    newpath.visited[i] = 1;//set analysing node as visited
                    newpath.obj[pnewpath->objnb] = i;//node added to the path obj list
                    newpath.objnb++;
                    newpath.nbNodeNotVisited--;

                    //printf("total points end %f\n", pnewpath->totalPoints);

                    pathplanning(mymap, pnewpath);
                }
                else
                {
                    //printf("path to node %d not taken but set as visited \n", i);
                    newpath.visited[i] = 1;//set analysing node as visited
                    newpath.nbNodeNotVisited--;
                    pathplanning(mymap, pnewpath);
                }
            }
        }
    }
    else
    {
        //printf("end of line obj list : \n");

        //for(int i = 0; i < N-1; i++)
            //printf("%d ", mypath->obj[i]);
        //printf("\n");

        //printf("timeleft : %f\n", mypath->timeleft);
        //printf("total points : %f\n", mypath->totalPoints);

        //printf("path number : %d\n", mymap->step);
        mymap->step++;

        if (mymap->mypath->totalPoints < mypath->totalPoints || (mymap->mypath->totalPoints == mypath->totalPoints && mymap->mypath->timeleft < mypath->timeleft)) //to be changed
        {
            mymap->mypath->objnb = mypath->objnb;
            mymap->mypath->totalPoints = mypath->totalPoints;
            mymap->mypath->timeleft = mypath->timeleft;
            for(int i = 0; i < N-1; i++)
                mymap->mypath->obj[i] = mypath->obj[i];
            //printf("path updated !!!!!\n\n");
        }
        //else
            //printf("no need to update\n\n");
    }
}

//map
Map* initmap()
{
    printf("map init\n");
    Map* mymap = (Map*) malloc(sizeof(Map));
    //initialization des noeuds
    readmap(mymap);
    //... (Pierre-Yves)
    mymap->step = 0;
    initpath(mymap);
    return mymap;
}

void readmap(Map* mymap) //function created to test but probably unuseful
{
    FILE *file;

    // use appropriate location if you are using MacOS or Linux
    file = fopen("nodesB.txt","r");

    if(file == NULL)
    {
        printf("node.txt not openable\n");
        //exit(1);
    }
    else
    {
        for(int i=0; i< N; i++)
        {
            fscanf(file, "%lf %lf %lf %lf %lf", &mymap->node[i][0], &mymap->node[i][1], &mymap->node[i][2], &mymap->node[i][3], &mymap->node[i][4]);
            printf("node [%d] : posx = %f ; posy= %f, points = %f \n", i, mymap->node[i][0], mymap->node[i][1], mymap->node[i][3]);
        }
        printf("node.txt finished \n");
    }

    fclose(file);

    file = fopen("distB.txt","r");

    if(file == NULL)
    {
        printf("dist.txt not openable\n");
        //exit(1);
    }
    else
    {
        for(int i=0; i< N; i++)
        {
            for(int j=0; j< N; j++)
            {
                fscanf(file, "%lf", &mymap->dist[i][j]);
                printf("dist [%d][%d] = %f \n", i, j, mymap->dist[i][j]);
            }
        }
        printf("distB.txt finished \n\n\n");
    }

    fclose(file);

}
