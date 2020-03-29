
#include "struct.h"



CtrlStruct* init(){
	CtrlStruct* structure = (CtrlStruct*) malloc(sizeof(CtrlStruct));
	structure->theUserStruct = (UserStruct *) malloc(sizeof(UserStruct));
	structure->theCtrlIn = (CtrlIn *) malloc(sizeof(CtrlIn));
	structure->theCtrlOut = (CtrlOut *) malloc(sizeof(CtrlOut));
	structure->theUserStruct->windup = 1;
	structure->theUserStruct->cmdtype = 0;
	structure->theUserStruct->period = 0.003;
	structure->theUserStruct->stime = 0;

	structure->theUserStruct->action_state = 0;
	structure->theUserStruct->side = 1; //1=right -1=left
	return structure;
}

void freeStruct(CtrlStruct *structure){
	free(structure->theUserStruct);
	free(structure);
}
