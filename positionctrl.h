#ifndef POSITIONCTRL_H_INCLUDED
#define POSITIONCTRL_H_INCLUDED

#define K 0.005//à modif !!!

//info
void updateinfo(CtrlStruct *theCtrlStruct, int a, int b, int c, int d);
void computeinfopos(CtrlStruct *theCtrlStruct);
void computeinfospeed(CtrlStruct *theCtrlStruct, int wel, int wer);
void computedisttobeam(CtrlStruct *theCtrlStruct, int lastRisingTick, int lastFallingTick);

//controller
void run_position(CtrlStruct *theCtrlStruct);
#endif // POSITIONCTRL_H_INCLUDED
