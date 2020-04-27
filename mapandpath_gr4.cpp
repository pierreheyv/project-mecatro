#include "CtrlStruct_gr4.h"
#include "namespace_ctrl.h"
#include "mapandpath_gr4.h"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>

//////////////////////////////////////////////////////////////////////////////////
//////////////       Itinerary : strategic path planning       ///////////////////
//////////////////////////////////////////////////////////////////////////////////c
NAMESPACE_INIT(ctrlGr4);

void update_destlist(Map* mymap, Path* simupath, int mode)//adapth mymap with the best path and total expected points in it variable have to be initialized before mode = 1 if agressive, 0 if not
{
    if (simupath->nbNodeNotVisited >= 1)//adapted to robotics
    {
        Path savepath = *simupath;//créer nouveau path (avec les même carac)
        Path* psavepath = &savepath;//adresse de newpath

        for(int i=0; i < NNODE; i++)
        {
            if (psavepath->visited[i] == 0)//si le noeud n'a pas encore été visité et est atteignable à temps => si tps d'aller au noeud + tps d'action au noeud + temps de retour à la base est < temps restant
            {
                Path newpath = *psavepath;//créer nouveau path avec même intérieur que le savepath
                Path* pnewpath = &newpath;//adresse de newpath

                if ((mymap->dist[simupath->actual_node][i] + mymap->dist[0][i]+ mymap->node[i][4]) < simupath->timeleft)
                {
                    newpath.timeleft = pnewpath->timeleft - (mymap->dist[newpath.actual_node][i]+ mymap->node[i][4]); //timeleft update
                    newpath.totalPoints = (pnewpath->totalPoints + mymap->node[i][3]); //point number update
                    newpath.actual_node = i;//actual node for simulation
                    newpath.visited[i] = 1;//set analysing node as visited
                    newpath.obj[pnewpath->objnb] = i;//node added to the path obj list
                    newpath.objnb++;
                    newpath.nbNodeNotVisited--;

                    update_destlist(mymap, pnewpath, mode);
                }
                else
                {
                    newpath.visited[i] = 1;//set analysing node as visited
                    newpath.nbNodeNotVisited--;
                    update_destlist(mymap, pnewpath, mode);
                }
            }
        }
    }
    else
    {
        if (mymap->mypath->totalPoints < simupath->totalPoints || (mymap->mypath->totalPoints == simupath->totalPoints && mymap->mypath->timeleft < simupath->timeleft))
        {
            mymap->mypath->objnb = simupath->objnb;
            mymap->mypath->totalPoints = simupath->totalPoints;
            mymap->mypath->timeleft = simupath->timeleft;
            for(int i = mode; i < simupath->objnb; i++)//fill begining from node "mode" => if mode == 1 (agresive) first node is always opponent
                mymap->mypath->obj[i] = simupath->obj[i];
            if (simupath->timeleft > mymap->dist[OPPONENTNODE][0] + mymap->dist[simupath->actual_node][0])//if time is sufficienrt at the end, go to opponent side to annoy him (will do at the begining only if agressive mode is on)
            {
                mymap->mypath->timeleft = 0;
                mymap->mypath->obj[simupath->objnb] = OPPONENTNODE;
                mymap->mypath->objnb++;
            }
        }
    }
}

void newpath(Map *mymap, double timepassed)//function to call if need a new path (at the beginning or when a problem occurs)
{
    //reset of values that will be updated (needed for good working of update_destlist function)
    mymap->mypath->totalPoints = 0;
    mymap->mypath->timeleft = GAMETIME -  timepassed; //time left actualisation
    mymap->mypath->nextNodelnb = 0;
    int saveactualnode = mymap->mypath->actual_node;

    int mode = 0;
    if (mymap->mypath->agressivmode && (mymap->mypath->timeleft/10 > mymap->dist[mymap->mypath->actual_node][OPPONENTNODE] + mymap->dist[OPPONENTNODE][mymap->mypath->actual_node])) //if mode = agresive + if the time needed to go to opponent and go back cost less than 10% of the available time then first go ti the opponent side to annoy him
    {
        mode = 1;
        mymap->mypath->obj[0] = OPPONENTNODE;
        mymap->mypath->objnb = 1;
        mymap->mypath->timeleft -= mymap->dist[mymap->mypath->actual_node][OPPONENTNODE];
        mymap->mypath->actual_node = OPPONENTNODE;//for simulation
    }
    else
        mymap->mypath->objnb = 0;

    update_destlist(mymap, mymap->mypath, mode);
    mymap->mypath->nextNode = mymap->mypath->obj[0];
    mymap->mypath->actual_node = saveactualnode;
}

void initpath(Map *mymap)//init path + fill with a new best path
{
    Path* mypath = (Path*) malloc(sizeof(Path));
    mymap->mypath = mypath;
    newpath(mymap, 0);
}

Map* initmap()
{
    Map* mymap = (Map*) malloc(sizeof(Map));
    readmap(mymap);
    initpath(mymap);
    mymap->mypath->actual_node = 0;
    mymap->lastnode = NORTHNODE; //till we know, return base = North (updated when we get the image result)

    load_obstacles(mymap);

    return mymap;
}

void load_obstacles(Map* mymap)
{
    mymap->obstaclesnb = 27;

    double **pobst;

    pobst = new double*[mymap->obstaclesnb]; // dynamic `array (size 5) of pointers to int`

    for (int i = 0; i < mymap->obstaclesnb; ++i)
    {
        pobst[i] = new double[3];
    }

    double **obstacles = pobst;

    //recifs
    obstacles[0][0]  = 1    ;
    obstacles[0][1]  = -0.6   ;
    obstacles[0][2]  = 0.33;
    obstacles[1][0]  = 0.8  ;
    obstacles[1][1]  = -0.6   ;
    obstacles[1][2]  = 0.23;
    obstacles[2][0]  = 0.7  ;
    obstacles[2][1]  = -0.6   ;
    obstacles[2][2]  = 0.19;

    obstacles[3][0]  = 1    ;
    obstacles[3][1]  = 0      ;
    obstacles[3][2]  = 0.33;
    obstacles[4][0]  = 0.8  ;
    obstacles[4][1]  = 0      ;
    obstacles[4][2]  = 0.23;
    obstacles[5][0]  = 0.7  ;
    obstacles[5][1]  = 0      ;
    obstacles[5][2]  = 0.19;

    obstacles[6][0]  = 1    ;
    obstacles[6][1]  = 0.6    ;
    obstacles[6][2]  = 0.33;
    obstacles[7][0]  = 0.8  ;
    obstacles[7][1]  = 0.6    ;
    obstacles[7][2]  = 0.23;
    obstacles[8][0]  = 0.7  ;
    obstacles[8][1]  = 0.6    ;
    obstacles[8][2]  = 0.19;

    //bancs de sable longs
    obstacles[9][0]  = -1   ;
    obstacles[9][1]  = -0.3   ;
    obstacles[9][2]  = 0.25;
    obstacles[10][0] = -0.8 ;
    obstacles[10][1] = -0.3   ;
    obstacles[10][2] = 0.15;
    obstacles[11][0] = -1   ;
    obstacles[11][1] = -0.5   ;
    obstacles[11][2] = 0.33;
    obstacles[12][0] = -1   ;
    obstacles[12][1] = -0.7   ;
    obstacles[12][2] = 0.33;
    obstacles[13][0] = -1   ;
    obstacles[13][1] = -0.9   ;
    obstacles[13][2] = 0.33;
    obstacles[14][0] = -1   ;
    obstacles[14][1] = -1.1   ;
    obstacles[14][2] = 0.33;
    obstacles[15][0] = -1   ;
    obstacles[15][1] = -1.3   ;
    obstacles[15][2] = 0.33;

    obstacles[16][0] = -1   ;
    obstacles[16][1] = 0.3    ;
    obstacles[16][2] = 0.25;
    obstacles[17][0] = -0.8 ;
    obstacles[17][1] = 0.3    ;
    obstacles[17][2] = 0.15;
    obstacles[18][0] = -1   ;
    obstacles[18][1] = 0.5    ;
    obstacles[18][2] = 0.33;
    obstacles[19][0] = -1   ;
    obstacles[19][1] = 0.7    ;
    obstacles[19][2] = 0.33;
    obstacles[20][0] = -1   ;
    obstacles[20][1] = 0.9    ;
    obstacles[20][2] = 0.33;
    obstacles[21][0] = -1   ;
    obstacles[21][1] = 1.1    ;
    obstacles[21][2] = 0.33;
    obstacles[22][0] = -1   ;
    obstacles[22][1] = 1.3    ;
    obstacles[22][2] = 0.33;

    //bancs de sable coins
    obstacles[23][0] = 0.8  ;
    obstacles[23][1] = -1.5   ;
    obstacles[23][2] = 0.43;
    obstacles[24][0] = 0.8  ;
    obstacles[24][1] = 1.5    ;
    obstacles[24][2] = 0.43;

    //pitits ilôts de sable
    obstacles[25][0]=-0.661 ;
    obstacles[25][1] = -0.308 ;
    obstacles[25][2] = 0.18;
    obstacles[26][0]=-0.661 ;
    obstacles[26][1] = 0.308  ;
    obstacles[26][2] = 0.18;

    mymap->obstacles = obstacles;
}

void readmap(Map* mymap) //function
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
        for(int i=0; i< NNODE; i++)
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
        for(int i=0; i< NNODE; i++)
        {
            for(int j=0; j< NNODE; j++)
            {
                fscanf(file, "%lf", &mymap->dist[i][j]);
                printf("dist [%d][%d] = %f \n", i, j, mymap->dist[i][j]);
            }
        }
        printf("distB.txt finished \n\n\n");
    }

    fclose(file);

}

NAMESPACE_CLOSE();
