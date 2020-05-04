#include <iostream>
#include "ctrl_main_gr4.h"

#include <time.h>
#include <math.h>
#include <windows.h>
#include <unistd.h>

using namespace std;

int main()
{

    Map* mymap = initmap();

    printf("best path found \n");

    printf("%d number in the path \n", mymap->mypath->objnb);

    for(int i=0; i < mymap->mypath->objnb; i++)
    {
        printf("%d ", mymap->mypath->obj[i]);
    }

    return 0;
}
