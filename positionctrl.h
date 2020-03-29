#ifndef POSITIONCTRL_H_INCLUDED
#define POSITIONCTRL_H_INCLUDED

#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"

#include "speedctrl.h"

#define K 0.005//à modif !!!
#define DISTTOAVOIDR 0.5//à définir !!!

//info
void updateinfo(CtrlStruct *theCtrlStruct, SPI_DE0 *spi );
void computeinfopos(CtrlStruct *theCtrlStruct, SPI_DE0 *spi);
void motorspeed(CtrlStruct *theCtrlStruct, SPI_DE0 *spi);
void computedisttobeam(CtrlStruct *theCtrlStruct, SPI_DE0 *spi);
int adjusttbit (int beforeTreatment); //to treat the information comming from the FPGA (avoid the first dirty byte)
void needtoavoid(CtrlStruct *theCtrlStruct);
void proberror(CtrlStruct *theCtrlStruct);
#endif // POSITIONCTRL_H_INCLUDED
