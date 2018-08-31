/*=============================================================================
  Copyright (C) 2013 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ApiController.h

  Description: Header file for the ApiController helper class that demonstrates
               how to implement an asynchronous, continuous image acquisition
               with VimbaCPP.

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

#ifndef AVT_VMBAPI_EXAMPLES_APICONTROLLER
#define AVT_VMBAPI_EXAMPLES_APICONTROLLER

#include <string>

#include "VimbaCPP/Include/VimbaCPP.h"

#include "avt_camera_streaming/FrameObserver.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

class ApiController
{
  public:
    ApiController();
    ~ApiController();

    //
    // Starts the Vimba API and loads all transport layers
    //
    // Returns:
    //  An API status code
    //
    VmbErrorType        StartUp();    
    
    //
    // Shuts down the API
    //
    void                ShutDown();
    
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
    VmbErrorType        StartContinuousImageAcquisition( const ProgramConfig & );    
    
    //
    // Calls the API convenience function to stop image acquisition
    // Closes the camera
    //
    // Returns:
    //  An API status code
    //
    VmbErrorType        StopContinuousImageAcquisition();

    //
    // Gets all cameras known to Vimba
    //
    // Returns:
    //  A vector of camera shared pointers
    //
    CameraPtrVector     GetCameraList() const;
    
    //
    // Translates Vimba error codes to readable error messages
    //
    // Parameters:
    //  [in]    eErr        The error code to be converted to string
    //
    // Returns:
    //  A descriptive string representation of the error code
    //
    std::string         ErrorCodeToMessage( VmbErrorType eErr ) const;
    
    //
    // Gets the version of the Vimba API
    //
    // Returns:
    //  The version as string
    //
    std::string         GetVersion() const;

  private:
    VmbErrorType        PrepareCamera();
    VimbaSystem &       m_system;                   // A reference to our Vimba singleton
    CameraPtr           m_pCamera;                  // The currently streaming camera
    FrameObserver*      m_pFrameObserver;           // Every camera has its own frame observer
};

}}} // namespace AVT::VmbAPI::Examples

#endif
