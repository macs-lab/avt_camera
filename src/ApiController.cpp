/*=============================================================================
  Copyright (C) 2013 - 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ApiController.cpp

  Description: Implementation file for the ApiController helper class that
               demonstrates how to implement an asynchronous, continuous image
               acquisition with VimbaCPP.

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

#include <sstream>
#include <iostream>

#include "avt_camera_streaming/ApiController.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

#define NUM_FRAMES 3
#define CAM_IP "169.254.75.133"


ApiController::ApiController(CameraParam cp)
    : m_system ( VimbaSystem::GetInstance() )
    , cam_param( cp )
{}


ApiController::~ApiController()
{
}

//
// Translates Vimba error codes to readable error messages
//
// Parameters:
//  [in]    eErr        The error code to be converted to string
//
// Returns:
//  A descriptive string representation of the error code
//
std::string ApiController::ErrorCodeToMessage( VmbErrorType eErr ) const
{
    return AVT::VmbAPI::Examples::ErrorCodeToMessage( eErr );
}

//
// Starts the Vimba API and loads all transport layers
//
// Returns:
//  An API status code
VmbErrorType ApiController::StartUp()
{
    return m_system.Startup();
}

//
// Shuts down the API
//
void ApiController::ShutDown()
{
    // Release Vimba
    m_system.Shutdown();
}

//
// Opens the given camera
// Sets the maximum possible Ethernet packet size
// Adjusts the image format
// Sets up the observer that will be notified on every incoming frame
// Calls the API convenience function to start image acquisition
// Closes the camera in case of failure
//
// Parameters:
//  [in]    Config      A configuration struct including the camera ID and other settings
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::StartContinuousImageAcquisition()
{
    // Open the desired camera by its ID
    VmbErrorType res = m_system.OpenCameraByID( cam_param.cam_IP.c_str(), VmbAccessModeFull, m_pCamera );
    if ( VmbErrorSuccess == res )
    {
        // Set the GeV packet size to the highest possible value
        // (In this example we do not test whether this cam actually is a GigE cam)
        FeaturePtr pCommandFeature;
        if ( VmbErrorSuccess == m_pCamera->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ))
        {
            if ( VmbErrorSuccess == pCommandFeature->RunCommand() )
            {
                bool bIsCommandDone = false;
                do
                {
                    if ( VmbErrorSuccess != pCommandFeature->IsCommandDone( bIsCommandDone ))
                    {
                        break;
                    }
                } while ( false == bIsCommandDone );
            }
        }

        if ( VmbErrorSuccess == res )
        {
            // set camera so that transform algorithms will never fail
            res = PrepareCamera();
            if ( VmbErrorSuccess == res )
            {
                // Create a frame observer for this camera (This will be wrapped in a shared_ptr so we don't delete it)
                m_pFrameObserver = new FrameObserver( m_pCamera, cam_param) ;
                // Start streaming
                res = m_pCamera->StartContinuousImageAcquisition( NUM_FRAMES, IFrameObserverPtr( m_pFrameObserver ));
            }
        }
        if ( VmbErrorSuccess != res )
        {
            // If anything fails after opening the camera we close it
            m_pCamera->Close();
        }
    }
    else
    {
        std::cout << "failed to open camera with IP: " << CAM_IP << std::endl;
    }

    return res;
}

/**setting a feature to maximum value that is a multiple of 2*/
VmbErrorType SetIntFeatureValueModulo2( const CameraPtr &pCamera, const char* const& Name )
{
    VmbErrorType    result;
    FeaturePtr      feature;
    VmbInt64_t      value_min,value_max;
    result = SP_ACCESS( pCamera )->GetFeatureByName( Name, feature );
    if( VmbErrorSuccess != result )
    {
        return result;
    }
    result = SP_ACCESS( feature )->GetRange( value_min, value_max );
    if( VmbErrorSuccess != result )
    {
        return result;
    }
    value_max =( value_max>>1 )<<1;
    result = SP_ACCESS( feature )->SetValue ( value_max );
    return result;
}
/**prepare camera so that the delivered image will not fail in image transform*/
VmbErrorType ApiController::PrepareCamera()
{
    VmbErrorType result;
    result = SetIntFeatureValueModulo2( m_pCamera, "Width" );
    if( VmbErrorSuccess != result )
    {
        return result;
    }
    result = SetIntFeatureValueModulo2( m_pCamera, "Height" );
    if( VmbErrorSuccess != result )
    {
        return result;
    }
    return result;
}

//
// Calls the API convenience function to stop image acquisition
// Closes the camera
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::StopContinuousImageAcquisition()
{
    // Stop streaming
    m_pCamera->StopContinuousImageAcquisition();

    // Close camera
    return  m_pCamera->Close();
}

//
// Gets all cameras known to Vimba
//
// Returns:
//  A vector of camera shared pointers
//
CameraPtrVector ApiController::GetCameraList() const
{
    CameraPtrVector cameras;
    // Get all known cameras
    if ( VmbErrorSuccess == m_system.GetCameras( cameras ))
    {
        // And return them
        return cameras;
    }
    return CameraPtrVector();
}

//
// Gets the version of the Vimba API
//
// Returns:
//  The version as string
//
std::string ApiController::GetVersion() const
{
    std::ostringstream  os;
    os<<m_system;
    return os.str();
}
}}} // namespace AVT::VmbAPI::Examples
