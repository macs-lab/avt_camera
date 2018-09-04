/*=========================================================
Created by Hui Xiao - University of Connecticuit - 2018
hui.xiao@uconn.edu
===========================================================*/

#ifndef CAMPARAM
#define CAMPARAM

#include <string>
//#include "VimbaCPP/Include/VimbaCPP.h"

struct CameraParam
{
    std::string cam_IP;
    int image_height;
    int image_width;
    int exposure_in_us;    
};


#endif