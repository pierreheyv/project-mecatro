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


#define CAN_BR 125e3

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

    can->ctrl_motor(1);

    SPI_DE0 *spi;
    spi = new SPI_DE0(0,500000);

    CtrlStruct* structure = init();

    can->push_TowDC(0);

    can->push_PropDC(0, 0);

    while(1)
    {
        updateinfo(structure, spi);  //update info from laser tower and from encoder (cfr posctrl)
        //speed, avancement, dist from beam

        computecmd(structure);//compute cmd to give to the motors

        can->push_PropDC(structure->theCtrlOut->wheel_commands[0], structure->theCtrlOut->wheel_commands[1]);

        writeDataToFile (structure->theUserStruct->Error_omega_l);
    }
    can->push_PropDC(0, 0);
}
