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
    std::string trigger_source;
    double frame_rate;
    // fram_rate could be: FixedRate, Software, FreeRun
    bool balance_white_auto;
    bool exposure_auto;
};


#endif