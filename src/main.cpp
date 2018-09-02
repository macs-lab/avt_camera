/*=============================================================================
  Copyright (C) 2013 - 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        program.cpp

  Description: Implementation of main entry point of AsynchronousGrabConsole
               example of VimbaCPP.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

/*=========================================================
Modified by Hui Xiao - University of Connecticuit - 2018
hui.xiao@uconn.edu
===========================================================*/

#include <string>
#include <cstring>
#include <iostream>

#include "VimbaCPP/Include/VimbaCPP.h"
#include "avt_camera_streaming/ApiController.h"
#include "ros/ros.h"

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "avt_camera");

    VmbErrorType err = VmbErrorSuccess;
    AVT::VmbAPI::Examples::ApiController apiController;
    
    // Print out version of Vimba
    std::cout<<"Vimba C++ API Version "<<apiController.GetVersion()<<"\n";
    
    // Startup Vimba
    err = apiController.StartUp();        
    if ( VmbErrorSuccess == err )
    {
        err = apiController.StartContinuousImageAcquisition();

        if ( VmbErrorSuccess == err )
        {
            std::cout<< "Press <enter> to stop acquisition...\n" ;
            getchar();

            apiController.StopContinuousImageAcquisition();
        }
        apiController.ShutDown();
    }

    if ( VmbErrorSuccess == err )
    {
        std::cout<<"\nAcquisition stopped.\n" ;
    }
    else
    {
        std::string strError = apiController.ErrorCodeToMessage( err );
        std::cout<<"\nAn error occurred: " << strError << "\n";
    }
    return err;
}
