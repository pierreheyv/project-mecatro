#include <cstdio>
#include <stdio.h>
#include <stdlib.h>

//#include <wiringPiSPI.h>  // Ne pas oublier de (dé)commenter wiringspi pour compilation
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"
#include <math.h>


#include "struct.h"
#include "positionctrl.h"
#include "ia.h"
#include "speedctrl.h"
#include "pathplanning.h"


#define CAN_BR 125e3

//Pour analyser ce code commencer ici à la fonction main et suivre avec fichier structure code

//plus propre si on mettait ça dans un fichier à part avec toutes les fonctions "supplémentaires"
void writeDataToFile (double data)  // by example "DataPath" = "C:\\program.txt"
{

    FILE *fptr;

    // use appropriate location if you are using MacOS or Linux
    fptr = fopen("error.txt","a");

    if(fptr == NULL)
    {
        printf("Error!");
        exit(1);
    }

    fprintf(fptr,"%f \n",data);
    fclose(fptr);

    //return 0;
}

int main()
{
    printf("hello world\n");
    printf("##############################################################################################################\n");
    printf("\t\t\t Titatronic ready !!!)");
    printf("##############################################################################################################\n");


    CAN *can;
    can = new CAN(CAN_BR);
    can->configure();

    SPI_DE0 *spi;
    spi = new SPI_DE0(0,500000);

    CtrlStruct* structure = init();

    can->ctrl_motor(1);
    can->push_TowDC(0);
    can->push_PropDC(0, 0);

    //path and map initialization (see pathplanning algorithm.c to understand how it works)
    Map* mymap = initmap();

    while(1)
    {
        updateinfo(structure, spi);//(cfr posctrl) update info from laser tower and from encoder, speed, avancement, dist from beam

        computecmd(structure, mymap);//(cfr ia) update the wanted commands

        //action : create sub function if need
        run_speed_controller(structure, can);//(cfr speedctrl )apply the speed (low level controller)

        //other : create sub function if need
        writeDataToFile (structure->theUserStruct->Error_omega_l);
    }
}
