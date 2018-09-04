/*=============================================================================
  Copyright (C) 2013 - 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FrameObserver.h

  Description: The frame observer that is used for notifications from VimbaCPP
               regarding the arrival of a newly acquired frame.

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

#ifndef AVT_VMBAPI_EXAMPLES_FRAMEOBSERVER
#define AVT_VMBAPI_EXAMPLES_FRAMEOBSERVER

#include <queue>
#include "VimbaCPP/Include/VimbaCPP.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc.hpp>

#include "avt_camera_streaming/MessagePublisher.h"
#include "avt_camera_streaming/CamParam.h"

#include "ros/console.h"

#define IMAGE_SIZE_WIDTH 1600
#define IMAGE_SIZE_HEIGHT 1200

namespace AVT {
namespace VmbAPI {
namespace Examples {


class FrameObserver : virtual public IFrameObserver
{
public:
    //
    // We pass the camera that will deliver the frames to the constructor
    //
    // Parameters:
    //  [in]    pCamera             The camera the frame was queued at
    //  [in]    eFrameInfos         Indicates how the frame will be displayed
    //  [in]    eColorProcessing    Indicates how color processing is applied
    //
    FrameObserver( CameraPtr pCamera, CameraParam cp);
    
    //
    // This is our callback routine that will be executed on every received frame.
    // Triggered by the API.
    //
    // Parameters:
    //  [in]    pFrame          The frame returned from the API
    //
    virtual void FrameReceived( const FramePtr pFrame );

private:
    void ShowFrameInfos( const FramePtr & );
    double GetTime();
    void SetCameraImageSize(CameraPtr m_pCamera, const VmbInt64_t& width, const VmbInt64_t& height);
	void SetExposureTime(const VmbInt64_t& time_in_us);
    template <typename T>
    class ValueWithState
    {
    private:
        T m_Value;
        bool m_State;
    public:
        ValueWithState()
            : m_State( false )
        {}
        ValueWithState( T &value )
            : m_Value ( value )
            , m_State( true )
        {}
        const T& operator()() const
        {
            return m_Value;
        }
        void operator()( const T &value )
        {
            m_Value = value;
            m_State = true;
        }
        bool IsValid() const
        {
            return m_State;
        }
        void Invalidate()
        {
            m_State = false;
        }
    };
    VmbInt64_t					width=1600;					//width and height of the image.
	VmbInt64_t					height=1200;
    ValueWithState<double>      m_FrameTime;
    ValueWithState<VmbUint64_t> m_FrameID;

    MessagePublisher mp;
    CameraParam cam_param;
};

}}} // namespace AVT::VmbAPI::Examples

#endif
