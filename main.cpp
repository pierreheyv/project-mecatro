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

//void writeDataToFile (double data, string "DataPath"){ // by example "DataPath" = "C:\\program.txt"
/*
void initfile(){ // by example "DataPath" = "C:\\program.txt"

   FILE *fptr;
   fptr = fopen("/home/pi/Desktop/error.txt","w");
   fclose(fptr);
}
*/
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

void writeDataToFile2 (double data)  // by example "DataPath" = "C:\\program.txt"
{

    FILE *fptr;

    // use appropriate location if you are using MacOS or Linux
    fptr = fopen("tension.txt","a");

    if(fptr == NULL)
    {
        printf("Error!");
        exit(1);
    }

    fprintf(fptr,"%f \n",data);
    fclose(fptr);
}

int main()
{
    printf("hello world\n");
    printf("##############################################################################################################\n");
    printf("\t\t\tWelcome to the Minibot project of the ELEME2002 class :)");
    printf("##############################################################################################################\n");
    printf("\t\t I'm Miss Sunshine, please take care of me !\n");
    printf("\t\t Please do not interchange the chips on my tower/motor PWM boards !\n");
    printf("\t\t Try to respect the C-file interface when programming me because\n \t\t it will be the same in the robotic project (Q2) !\n");


    CAN *can;
    can = new CAN(CAN_BR);
    can->configure();

    can->ctrl_motor(1);

    SPI_DE0 *spi;
    spi = new SPI_DE0(0,500000);

    CtrlStruct* structure = init();
    int initfilebool = remove("error.txt");
    int initfilebool2 = remove("tension.txt");

    can->push_TowDC(0);

    can->push_PropDC(0, 0);

    while(1)
    {
        //dégeulasse de faire ça ici mais bug avec le fait de passer un SPI flemme de gérer ça
        int lenc1 = (spi->read(0x02))>>16;
        int lenc2 = (spi->read(0x03))>>16;
        int beforeTreatmentWrs = (spi->read(0x00));
        int beforeTreatmentWls = (spi->read(0x01));

        int maskWrs;
        int maskWls;
        if(beforeTreatmentWrs >= 0)
        {
            maskWrs = 0b00000000;
        }
        else
        {
            maskWrs = 0b11111111;
        }

        if(beforeTreatmentWls >= 0)
        {
            maskWls = 0b00000000;
        }
        else
        {
            maskWls = 0b11111111;
        }

        int wrs = maskWrs <<24 | (spi->read(0x00))>>8;
        int wls = maskWls <<24 | (spi->read(0x01))>>8;

        updateinfo(structure, lenc1, lenc2, wls, wrs);  //update info from laser tower and from encoder (cfr posctrl)
        //speed, avancement, dist from beam

        //computecmd(structure);//calculate the desired distance, angle for the following computations (cfr ia)

        run_position(structure);//calculate what tension to give to the wheels (cfr posctrl)

        can->push_PropDC(structure->theCtrlOut->wheel_commands[0], structure->theCtrlOut->wheel_commands[1]);

        writeDataToFile (structure->theUserStruct->Error_omega_l);
        writeDataToFile2 (structure->theCtrlOut->wheel_commands[0]);

    }
    can->push_PropDC(0, 0);
}
