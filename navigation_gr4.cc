/*!
 * \file navigation_gr4.cc
 * \brief
 */

#include "navigation_gr4.h"
#include "namespace_ctrl.h"
#include "performActions_gr4.h"
#include "CtrlStruct_gr4.h"

NAMESPACE_INIT(ctrlGr4);
//////////////////////////////////////////////////////////////////////////////////
///////////////////////       Navigation commands        /////////////////////////
//////////////////////////////////////////////////////////////////////////////////

/*! \brief launch the right navigation mode with destination right destination
 *
 * \param[in] cvs controller main structure
 */
void navigation(CtrlStruct *cvs)
{
    if (cvs->navigmode == 0)
        middle_controller(cvs, cvs->mymap->node[cvs->mymap->mypath->nextNode][0], cvs->mymap->node[cvs->mymap->mypath->nextNode][1]);
    else potential_field(cvs, cvs->mymap);
}

/*! \brief classical rotate/go-forward navigation mode
 *
 * \param[in] cvs controller main structure, destination positions
 */
void middle_controller(CtrlStruct *cvs, double objposx, double objposy)
{
    double deltax = cvs->position_xyt[0] - objposx;
    double deltay = cvs->position_xyt[1] - objposy;
    double dist = sqrt(pow(deltax, 2)+pow(deltay,2));

    // Checking if arrived at destination
    if (fabs(dist) < 0.01)
    {
        cvs->stateNavigation = 0;
        cvs->mymap->mypath->actual_node = cvs->mymap->mypath->nextNode;//update actual node
        cvs->stateGlobal = 2;//do action related to that node
        cvs->startingActiontime = cvs->inputs->t;//record time from action state launch
        printf("on target");
    }
    else
    {
        double diagangle = atan2(deltay,deltax); //atan(dy/dx)
        if (diagangle<0)
        {
            diagangle = diagangle + M_PI;
        }
        printf("diagangle : %f \n", diagangle);

        double diffAngle = diagangle - (cvs->position_xyt[2]);

        int sign = ((diffAngle>=0) - (diffAngle<=0));

        if (fabs(diffAngle)> M_PI)
        {
            sign = -sign;
        }

        if (fabs(diffAngle) > 0.1*M_PI) //Checking if need to be reoriented
        {
            cvs->stateNavigation = 1;
        }

        // If moving forward is not the right choice
        if (cvs->stateNavigation == 1)// Correcting the angle
        {
            if (fabs(diffAngle) > 0.01*M_PI) //orientation almost perfect
            {
                rot(cvs, diffAngle);
            }
            else
            {
                cvs->stateNavigation = 0;   // moving forward is the right choice
            }
        }
        else
        {
            double alpha = (M_PI_2 - diffAngle);
            double curv_radius = dist/(cos(alpha)*2);
            double centerangle = M_PI - alpha*2;
            cvs->wheel_demands[0] = KL*centerangle*(curv_radius+(sign*WIDTH));// formula for right wheel
            cvs->wheel_demands[1] = KL*centerangle*(curv_radius+(-sign*WIDTH)); // formula for left wheel
        }
    }
}

/*! \brief demands on wheels speed to rotate counter clockwise with a speed proportional to angle
 *
 * \param[in] cvs controller main structure, angle left to rotate
 */
void rot(CtrlStruct *cvs, double angle) //
{
    printf("pure rotation, diff angle = %f \n", angle);
    cvs->wheel_demands[0] = KR*angle;//to be tested signs
    cvs->wheel_demands[1] = -KR*angle;
}

/*! \brief potential field navigation mode: repulsive force of static and moving obstacles, attraction point is the destination node (stored in the destination list "mypath")
 * demands speed in function of the computed fictive resultant force of the repulsive/attractive points
 * \param[in] cvs controller main structure, destination positions
 */
void potential_field(CtrlStruct *structure, Map *mymap)
{
    double deltax;//deltapos robot vs obst/attraction point
    double deltay;

    double fangle;//angle of the att/rep force in the x,y repere
    double diffangle;//angle between the force and the robot's theta

    double dist;
    double fint;//force intensity

    double resfp = 0;//resultant force in the robot frame perpendicular to the robot
    double resff = 0;//resultant force in the robot frame toward the forward movement of the robot

    //add the point of attraction force
    deltax = structure->position_xyt[0] - mymap->node[mymap->mypath->obj[mymap->mypath->nextNode]][0];
    deltay = structure->position_xyt[1] - mymap->node[mymap->mypath->obj[mymap->mypath->nextNode]][1];

    dist = sqrt(pow(deltax, 2)+pow(deltay,2));
    fint = KFA*dist; //force in proportional to dist

    if (fabs(dist) < 0.01) //if we are on target go to action state
    {
        structure->stateNavigation = 0;
        structure->stateGlobal = 2;//passer en mode stop
        structure->startingActiontime = structure->inputs->t;
        printf("on target");
        return;
    }

    fangle = atan((deltay)/(deltax));//vérif angle < 0
    diffangle = structure->position_xyt[2] - fangle;

    resfp = resfp + cos(diffangle)*fint;
    resff = resff + sin(diffangle)*fint;

    //calculate the resultant of the obstacle forces in the robot system for all the fixed obstacles
    for(int i=0; i<mymap->obstaclesnb; i++)
    {
        deltax = structure->position_xyt[0] - mymap->obstacles[i][0];
        deltay = structure->position_xyt[1] - mymap->obstacles[i][1];

        dist = sqrt(pow(deltax, 2)+pow(deltay,2));
        fint = - KFA/(dist - mymap->obstacles[i][2]); //force in 1/x

        fangle = atan((deltay)/(deltax));//vérif angle < 0
        diffangle = structure->position_xyt[2] - fangle;

        resfp = resfp + cos(diffangle)*fint;
        resff = resff + sin(diffangle)*fint;
    }

    //add the forces due to the oponnents
    for(int i=0; i< structure->inputs->nb_opponents; i++)
    {
        dist = structure->pos_beacon_disdirray[0][i];
        double dir = structure->pos_beacon_disdirray[1][i];
        double width = structure->pos_beacon_disdirray[2][i];

        fint = - KFA/(dist - width - ROBOTRADIUS);//force in 1/x

        resfp = resfp + cos(dir)*fint;
        resff = resff + sin(dir)*fint;
    }

    //convert this into desired speed on wheels (applying the force on a decaled point (DECFP) and calculating the couples see report)
    if (resff>0)
    {
        structure->wheel_demands[0] = resff + DECFP*resfp/WIDTH*KRA;//not bounded (but is it a problem ?)
        structure->wheel_demands[1] = resff - DECFP*resfp/WIDTH*KRA;
    }
    else
    {
        structure->wheel_demands[0] = resff - DECFP*resfp/WIDTH*KRA;//not bounded (but is it a problem ?)
        structure->wheel_demands[1] = resff + DECFP*resfp/WIDTH*KRA;
    }
}

NAMESPACE_CLOSE();
