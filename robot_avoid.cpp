#define WIDTH 0.5 //width between two wheels
#define DECFP 0.5 //distance between de center of mobility and the point of application of the potential field force (scheme)
#define KRA 0.5 //coef of rotation for robot avoid (see in code)
#define KFA 0.5 //coef of force for robot avoid (see in code)

//returns the wanted speed of each wheels after computing the force exerted on the robot by some obstacles
void avoid(CtrlStruct *structure, Map *mymap)
{
    //rmq :
    //obstacles have to be continuously updated with nb and pos
    //for now only point obstacles are treated

    double deltax;//deltapos robot vs obst/attraction point
    double deltay;

    double fangle;//angle of the att/rep force in the x,y repere
    double diffangle;//angle between the force and the robot's theta

    double dist;
    double fint;//force intensity

    double resfp;//resultant force in the robot frame perpendicular to the robot
    double resff;//resultant force in the robot frame toward the forward movement of the robot

    for(int i=0; i<mymap->obstaclesnb; i++) //calculate the resultant of the obstacle forces in the robot system
    {
        deltax = structure->theUserStruct->posxyt[0] - mymap->obstacles[i]->posxy[0];
        deltay = structure->theUserStruct->posxyt[1] - mymap->obstacles[i]->posxy[1];

        dist = sqrt(pow(deltax, 2)+pow(deltay,2));
        fint = - KFA/(dist - mymap->obstacles[i]->width); //force in 1/x

        fangle = atan((deltay)/(deltax));//vérif angle < 0
        diffangle = structure->theUserStruct->posxyt[2] - fangle;

        resfp = resfp + cos(diffangle)*fint;
        resff = resff + sin(diffangle)*fint;
    }

    //add the point of attraction force
    deltax = structure->theUserStruct->posxyt[0] - mymap->node[mymap->mypath->obj[mymap->mypath->objnb]][0];
    deltay = structure->theUserStruct->posxyt[1] - mymap->node[mymap->mypath->obj[mymap->mypath->objnb]][1];

    dist = sqrt(pow(deltax, 2)+pow(deltay,2));
    fint = KFA/dist; //force in 1/x

    fangle = atan((deltay)/(deltax));//vérif angle < 0
    diffangle = structure->theUserStruct->posxyt[2] - fangle;

    resfp = resfp + cos(diffangle)*fint;
    resff = resff + sin(diffangle)*fint;

    //convert this into forces on wheels (applying the force on a decaled point (DECFP) and calculating the couples)
    //hard to explain just ask for explication...
    if (resff>0)
    {
        structure->theUserStruct->wantedspeedl = resff + DECFP*resfp/WIDTH*KRA;//not bounded (but is it a problem ?)
        structure->theUserStruct->wantedspeedr = resff - DECFP*resfp/WIDTH*KRA;
    }
    else
    {
        structure->theUserStruct->wantedspeedl = resff - DECFP*resfp/WIDTH*KRA;//not bounded (but is it a problem ?)
        structure->theUserStruct->wantedspeedr = resff + DECFP*resfp/WIDTH*KRA;
    }
}
