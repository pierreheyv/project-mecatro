/*!
 * \file performActions_gr4.cc
 * \brief
 */

#include "namespace_ctrl.h"

#include "performActions_gr4.h"
#include <CtrlStruct_gr4.h>

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
    //during this function actual node = nextnode (because if this function is launched it's because we arrived on the destination "nextnode"
    if (cvs->mymap->mypath->nextNode == DEPONODE)//if we are in the depot
    {
        newpath(cvs->mymap, cvs->tactual);
        cvs->outputs->flag_release = 1;
        if (cvs->inputs->nb_targets == 0)
        {
            cvs->nbtargets = 0;
            cvs->stateGlobal = 1;
        }
    }
    else if (cvs->mymap->node[0][3] == 0)//no action to be done
        nextobj(cvs->mymap->mypath);
    else//if we are on target
    {
        cvs->outputs->flag_release = 0;
        switch(cvs->stateAction) //different action depending on the node (mostly for mecatronic project => here blocked on 0, case 1 is an example of other action)
        {
        case 0  :
            if (cvs->inputs->target_detected == 1)
            {
                printf("Waiting to grab the disk");
                if ((fabs(cvs->startingActiontime - cvs->inputs->t)) <= 3 && (cvs->inputs->nb_targets == cvs->nbtargets))
                {
                    cvs->wheel_demands[0] = 0;
                    cvs->wheel_demands[1] = 0;
                }
                else
                {
                    cvs->stateGlobal = 1;
                }
                break;
            }
            else
            {
                printf("should be on target node but no target detected");
                cvs->stateGlobal = 1;
            }

        case 1  :
            printf("Turning to celebrate \n");
            if ((fabs(cvs->startingActiontime - cvs->inputs->t)) <= 2.1)
            {
                cvs->wheel_demands[0] = 20*(cvs->startingSide);
                cvs->wheel_demands[1] = -20*(cvs->startingSide);
            }
            else
            {
                cvs->stateGlobal = 1;
            }
            break;

        // default position is set to stop
        default : // Stop all actions
            stopActions(cvs);
        }
    }
}

/*! \brief update the destination with the destination list
 *
 * \param[in] path (destination list + informations)
 */
void nextobj(Path* mypath)
{
    if (mypath->nextNodelnb == mypath->objnb)//c'était le dernier obj
        mypath->nextNode = DEPONODE;
    else
    {
        mypath->actual_node = mypath->nextNode;
        mypath->visited[mypath->actual_node] = 1;
        mypath->nbNodeNotVisited--;
        mypath->nextNodelnb++;
        mypath->nextNode = mypath->obj[mypath->nextNodelnb];
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
