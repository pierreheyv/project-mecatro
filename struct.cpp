
#include "struct.h"
#include <stdlib.h>



CtrlStruct* init(){
	CtrlStruct* structure = (CtrlStruct*) malloc(sizeof(CtrlStruct));
	structure->theUserStruct = (UserStruct *) malloc(sizeof(UserStruct));
	structure->theCtrlIn = (CtrlIn *) malloc(sizeof(CtrlIn));
	structure->theCtrlOut = (CtrlOut *) malloc(sizeof(CtrlOut));
	structure->theUserStruct->wantedspeedl = 0;
	structure->theUserStruct->wantedspeedr = 0;
	structure->theUserStruct->windup = 1;
	structure->theUserStruct->cmdtype = 0;
    structure->theUserStruct->wanteddist = 0;
	structure->theUserStruct->wantedangle = 0;
	structure->theUserStruct->avancement[0] = 0;
	structure->theUserStruct->avancement[1] = 0;
	structure->theUserStruct->angleleft = 0;
	structure->theUserStruct->distleft =0;
	structure->theUserStruct->period = 0.003;
	return structure;
}

void freeStruct(CtrlStruct *structure){
	free(structure->theUserStruct);
	free(structure);
}
