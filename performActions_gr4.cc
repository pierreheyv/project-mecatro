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
        }
        break;
    case 3 : //manche à air
        //...
        break;
    case 4 : //girouette
        //...
        break;
    case 5 : //phare
        //...
        break;

    default : //no action defined with this node => go to next
        nextobj(cvs->mymap->mypath);
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
    cvs->stateGlobal = 1;
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
