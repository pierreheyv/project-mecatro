/*!
 * \file performActions_gr4.cc
 * \brief
 */

#include "namespace_ctrl.h"

#include "performActions_gr4.h"
#include "CtrlStruct_gr4.h"

NAMESPACE_INIT(ctrlGr4);

//////////////////////////////////////////////////////////////////////////////////
///////////////////////       Set actions to perform        //////////////////////
//////////////////////////////////////////////////////////////////////////////////

/*! \brief launch action in function of the actual node associated action
 *
 * \param[in] cvs controller main structure
 */
void performActions(CtrlStruct *cvs)
{
    switch(cvs->mymap->mypath->actual_node) //different action depending on the node
    {
    case 0  : //test
    //...

    case 1  : //example
        printf("Turning to celebrate \n");
        if ((fabs(cvs->startingActiontime - cvs->inputs->t)) <= 2.1)
        {
            cvs->wheel_demands[0] = 20*(cvs->startingSide);
            cvs->wheel_demands[1] = -20*(cvs->startingSide);
        }
        else
        {
            nextobj(cvs->mymap->mypath);
            cvs->stateGlobal = 1;
        }
        break;
    case 3 : //manche à air
        action_manche(cvs);
        break;
    case 4 : //girouette
        //...
        break;
    case 5 : //phare
        //...
        break;

    default : //no action defined with this node => go to next
        nextobj(cvs->mymap->mypath);
        cvs->stateGlobal = 1;
    }
}


/*! \brief update the destination with the destination list
 *
 * \param[in] path (destination list + informations)
 */
void nextobj(Path* mypath)
{
    if (mypath->nextNodelnb == mypath->objnb)//c'était le dernier obj
        mypath->nextNode = DEPONODE;//
    else
    {
        mypath->actual_node = mypath->nextNode;
        mypath->visited[mypath->actual_node] = 1;
        mypath->nbNodeNotVisited--;
        mypath->nextNodelnb++;
        mypath->nextNode = mypath->obj[mypath->nextNodelnb];
    }
}

//--------------------------------MA action--------------------------------------------

void action_manche(CtrlStruct *cvs)
{
    switch(cvs->stateAction) //action state of the MA (=Manche à Aire) action
    {
    case 0  : //robot in front of first MA
        if (cvs->piston_out)//piston is released
        {
            pushMA(cvs, 0);
        }
        else
            simu_pneuma_piston(1, cvs);//here in RTOS : send message to CAN -> electrovanne
        break;
    case 1  : //in front of MA 2
        pushMA(cvs, 1);
        break;
    case 2:
        if (!cvs->piston_in)
            simu_pneuma_piston(2, cvs);
        else
        {
            cvs->stateGlobal = 1;
            nextobj(cvs->mymap->mypath);
        }
    }
}

void simu_pneuma_piston(int cmd, CtrlStruct *cvs)
{
    switch(cmd)
    {
    case 1:
        printf("pneuma piston cmd 01 => piston being released\n");
        if (cvs->piston_state < 100)
            cvs->piston_state += 10;
        break;
    case 2:
        printf("pneuma piston cmd 10 => piston being going back\n");
        if (cvs->piston_state > 0)
            cvs->piston_state -= 10;
        break;
    }
    printf("piston deployed at %d percent \n", cvs->piston_state);
    if (cvs->piston_state == 100)
    {
        cvs->piston_out =1;
        cvs->piston_in =0;
    }
    else
    {
        cvs->piston_out = 0;
        if (cvs->piston_state == 0)
            cvs->piston_in = 1;
    }
}

void pushMA(CtrlStruct *cvs, int MAnb)
{
    if (cvs->navigmode == 1)
    {
        printf("robot detected in the zone mission aborted\n");
        cvs->stateGlobal = 1;
        nextobj(cvs->mymap->mypath);
    }

    double diffangle = cvs->position_xyt[2] - M_PI_2;
    if (diffangle>0.1)
        rot(cvs, diffangle);
    else
    {
        if (MAnb == 1)
        {
            if (simu_middle_controller(cvs, 0, 20))//(MAnb && middle_controller(cvs, 0, 20))
                cvs->stateAction = 2;
        }
        else if (simu_middle_controller(cvs, 0, 10))//(middle_controller(cvs, 0, 10))
            cvs->stateAction = 1;
    }
}
//////////////////////////////////////////////////////////////////////////////////
///////////////////////       desired speeds put to 0      ///////////////////////
//////////////////////////////////////////////////////////////////////////////////

/*! \brief stop the robot using speed controller
 *
 * \param[in] main control structure
 */
void stopActions(CtrlStruct *cvs)
{
    cvs->wheel_demands[0] = 0;
    cvs->wheel_demands[1] = 0;
}

NAMESPACE_CLOSE();
