#include "ia.h"


void computecmd(CtrlStruct *theCtrlStruct){

    cmdtype(theCtrlStruct);//chooses de cmd type

    int type = theCtrlStruct->theUserStruct->cmdtype;
    //calculate the distance left, angle left
    if (type == 2){ //mode manuel pure

        //take the instruction
        theCtrlStruct->theUserStruct->wanteddist = 5;
        theCtrlStruct->theUserStruct->wantedangle = 0;

        //calculates what's left (je sais pas comment faire niveau avancement distance linéaire et de rotation vs avancement)
        theCtrlStruct->theUserStruct->distleft = theCtrlStruct->theUserStruct->wanteddist - theCtrlStruct->theUserStruct->avancement[0];
        theCtrlStruct->theUserStruct->angleleft = theCtrlStruct->theUserStruct->wantedangle - theCtrlStruct->theUserStruct->avancement[0];
    }

    else if (type == 1){//mode liste d'instructions
        //calculates what's left
        theCtrlStruct->theUserStruct->distleft = theCtrlStruct->theUserStruct->wanteddist - theCtrlStruct->theUserStruct->avancement[0];
        double dleft = theCtrlStruct->theUserStruct->distleft;
        double aleft = theCtrlStruct->theUserStruct->angleleft;
        if (dleft == 0 && aleft == 0){
            takenext_instru(theCtrlStruct);
            //reset the avancements
            theCtrlStruct->theUserStruct->avancement[0] = 0;
            theCtrlStruct->theUserStruct->avancement[1] = 0;
        }
    }
}

void cmdtype(CtrlStruct *theCtrlStruct){
    theCtrlStruct->theUserStruct->cmdtype = 2; //puts the cmdtype to 2 (= manual test)
}

void takenext_instru(CtrlStruct *theCtrlStruct){
    //to be implemented ...
    //read from txt
    double dist_inst = 5;//to be searched in the file
    double angle_inst = 0;//to be searched in the file
    //apply
    theCtrlStruct->theUserStruct->wanteddist = dist_inst;
    theCtrlStruct->theUserStruct->wantedangle = angle_inst;
}
