/*!
 * \file ctrl_main_gr4.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr4.h"
#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"

NAMESPACE_INIT(ctrlGr4);


//////////////////////////////////////////////////////////////////////////////////
/////////////////   Initialization necessary functions       /////////////////////
//////////////////////////////////////////////////////////////////////////////////

/*! \brief initialize controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
    setStartingSide(cvs);
    cvs->mymap = initmap();//cfr map and path
}

/*! \brief updte the starting side variable
 *
 * -1 if right side / 1 if left side
 * \param[in] cvs controller main structure
 */
void setStartingSide(CtrlStruct *cvs) // Later I will choose the constant position depending of the robot ID maybe
{
    if(cvs->inputs->robot_id < 2)
    {
        cvs->startingSide = -1; // robot on the right/yellow side
    }
    else
    {
        cvs->startingSide = 1; //robot on the left/blue side
    }
}

//////////////////////////////////////////////////////////////////////////////////
/////////////////   Selection of the desired action/state    /////////////////////
////////////////////          MAIN FUNCTION LOOP       ///////////////////////////
//////////////////////////////////////////////////////////////////////////////////


/*! \brief controller loop (called every timestep)
 *
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{
    //test if it's time to end and goback to base for the end of the game
    cvs->tactual = cvs->inputs->t;

    cvs->mymap->mypath->timeleft = cvs->mymap->mypath->timeleft - cvs->tactual;
    if ((cvs->mymap->dist[cvs->mymap->mypath->actual_node][BASENODE] + TIMEMARGIN < cvs->mymap->mypath->timeleft))//if time to go home go home
    {
        cvs->stateGlobal = 1;
        cvs->mymap->mypath->nextNode = BASENODE;
    }

    //getting information
    odometrie(cvs);//cfr odometrie.c

    //state ?
    switch(cvs->stateGlobal)
    {
    case 0  :
        calibration(cvs);//cfr calibration.c
        break;

    case 1  :
        navigation(cvs);//cfr navigation.c
        break;

    case 2  :
        performActions(cvs);//cfr performAction.c
        break;

    case 3  :  //test state
        cvs->wheel_demands[0] = 20;
        cvs->wheel_demands[1] = -20;
        break;

    // default position is set to stop
    default : // Stop all actions //cfr performAction.c
        stopActions(cvs);
        printf("in case stop \n");
    }
    controller_speed_loop(cvs);
}

//////////////////////////////////////////////////////////////////////////////////
/////////////////   Ending the game : stopping the robot    //////////////////////
//////////////////////////////////////////////////////////////////////////////////


/*! \brief last controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{
    cvs->outputs->wheel_commands[0] = 0;
    cvs->outputs->wheel_commands[1] = 0;
}

NAMESPACE_CLOSE();
