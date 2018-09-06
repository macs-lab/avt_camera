/*=============================================================================
  Copyright (C) 2013 - 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FrameObserver.cpp

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

#include <iostream>
#include <iomanip>
#ifdef WIN32
#include <Windows.h>
#else
#include <time.h>
#endif //WIN32

#include "avt_camera_streaming/FrameObserver.h"

#define SHOW_FRAME_INFO 0

namespace AVT {
namespace VmbAPI {
namespace Examples {

//
// We pass the camera that will deliver the frames to the constructor
//
// Parameters:
//  [in]    pCamera             The camera the frame was queued at
//  [in]    eFrameInfos         Indicates how the frame will be displayed
//  [in]    eColorProcessing    Indicates how color processing is applied
//
FrameObserver::FrameObserver( CameraPtr pCamera, CameraParam cp)
    :   IFrameObserver( pCamera )
    ,   cam_param(cp)
{
    // Set the width and height of the camera image
	VmbInt64_t width_s = cam_param.image_width;
	VmbInt64_t height_s = cam_param.image_height;
	SetCameraImageSize(pCamera, width_s, height_s);

	// Set the exposure time
	VmbInt64_t expTime = cam_param.exposure_in_us;
	SetExposureTime(expTime);

	// Read the width and height of the camera image
	FeaturePtr feature;
	if (VmbErrorSuccess == pCamera->GetFeatureByName("Width", feature))
	{
		if (VmbErrorSuccess == feature->GetValue(width))
		{
			std::cout << "The Image width is " << width << std::endl;
		}
	}

	if (VmbErrorSuccess == pCamera->GetFeatureByName("Height", feature))
	{
		if (VmbErrorSuccess == feature->GetValue(height))
		{
			std::cout << "The Image height is " << height << std::endl;
		}
	}
}


void FrameObserver::SetCameraImageSize(CameraPtr m_pCamera, const VmbInt64_t& width, const VmbInt64_t& height)
{
	// set the offset value to centralize the image.
	VmbInt64_t offset_x = int((1600 - width) / 2);
	VmbInt64_t offset_y = int((1200 - height) / 2);
	FeaturePtr pCommandFeature;
	VmbErrorType err;
	err = m_pCamera->GetFeatureByName("Height", pCommandFeature);
	if (err == VmbErrorSuccess)
	{
		err = pCommandFeature->SetValue(height);
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pCommandFeature->IsCommandDone(bIsCommandDone))
				{
					break;
				}
			} while (false == bIsCommandDone);
		}
		else
		{
            ROS_ERROR("failed to set height.");
		}
	}

	err = m_pCamera->GetFeatureByName("Width", pCommandFeature);
	if (err == VmbErrorSuccess)
	{
		err = pCommandFeature->SetValue(width);
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pCommandFeature->IsCommandDone(bIsCommandDone))
				{
					break;
				}
			} while (false == bIsCommandDone);
		}
		else
		{
            ROS_ERROR("failed to set width");
		}
	}

	
	err = m_pCamera->GetFeatureByName("OffsetX", pCommandFeature);
	if (err == VmbErrorSuccess)
	{
		err = pCommandFeature->SetValue(offset_x);
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pCommandFeature->IsCommandDone(bIsCommandDone))
				{
					break;
				}
			} while (false == bIsCommandDone);
		}
		else
		{
            ROS_ERROR("failed to set OffsetX");
		}
	}

	err = m_pCamera->GetFeatureByName("OffsetY", pCommandFeature);
	if (err == VmbErrorSuccess)
	{
		err = pCommandFeature->SetValue(offset_y);
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pCommandFeature->IsCommandDone(bIsCommandDone))
				{
					break;
				}
			} while (false == bIsCommandDone);
		}
		else
		{
            ROS_ERROR("failed to set OffsetY");
		}
	}
	
}

void FrameObserver::SetExposureTime(const VmbInt64_t & time_in_us)
{
	FeaturePtr pCommandFeature;
	VmbErrorType err;
	err = m_pCamera->GetFeatureByName("ExposureTimeAbs", pCommandFeature);
	if (err == VmbErrorSuccess)
	{
		err = pCommandFeature->SetValue((double)time_in_us); 
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pCommandFeature->IsCommandDone(bIsCommandDone))
				{
					break;
				}
			} while (false == bIsCommandDone);
		}
		else
		{
			std::cout << "failed to set ExposureTimeAbs." << std::endl;
			getchar();
		}
	}
}

//
// Gets the current timestamp for interval measurement
//
double FrameObserver::GetTime()
{
    double dTime = 0.0;

#ifdef WIN32
    LARGE_INTEGER nCounter;
    QueryPerformanceCounter( &nCounter );
    dTime = ( (double)nCounter.QuadPart ) / m_dFrequency;
#else
    //clock_t nTime = times(NULL);
    //dTime = ((double)(nTime) * 10000.0) / ((double)CLOCKS_PER_SEC);
    struct timespec now;
    clock_gettime( CLOCK_REALTIME, &now );
    dTime = ( (double)now.tv_sec ) + ( (double)now.tv_nsec ) / 1000000000.0;
#endif //WIN32

    return dTime;
}

//
// Prints out frame parameters such as 
// - width
// - height
// - pixel format
//
// Parameters:
//  [in]    pFrame          The frame to work on
//
void PrintFrameInfo( const FramePtr &pFrame )
{
    std::cout<<" Size:";
    VmbUint32_t     nWidth = 0;
    VmbErrorType    res;
    res = pFrame->GetWidth(nWidth);
    if( VmbErrorSuccess == res )
    {
        std::cout<<nWidth;
    }
    else
    {
        std::cout<<"?";
    }

    std::cout<<"x";
    VmbUint32_t nHeight = 0;
    res = pFrame->GetHeight(nHeight);
    if( VmbErrorSuccess == res )
    {
        std::cout<< nHeight;
    }
    else
    {
        std::cout<<"?";
    }

    std::cout<<" Format:";
    VmbPixelFormatType ePixelFormat = VmbPixelFormatMono8;
    res = pFrame->GetPixelFormat( ePixelFormat );
    if( VmbErrorSuccess == res )
    {
        std::cout<<"0x"<<std::hex<<ePixelFormat<<std::dec;
    }
    else
    {
        std::cout<<"?";
    }
}

//
// Prints out frame status codes as readable status messages
//
// Parameters:
//  [in]    eFrameStatus    The error code to be converted and printed out
//
void PrintFrameStatus( VmbFrameStatusType eFrameStatus )
{
    switch( eFrameStatus )
    {
    case VmbFrameStatusComplete:
        std::cout<<"Complete";
        break;

    case VmbFrameStatusIncomplete:
        std::cout<<"Incomplete";
        break;

    case VmbFrameStatusTooSmall:
        std::cout<<"Too small";
        break;

    case VmbFrameStatusInvalid:
        std::cout<<"Invalid";
        break;

    default:
        std::cout<<"unknown frame status";
        break;
    }
}

//
// Prints out details of a frame such as
// - ID
// - receive status
// - width, height, pixel format
// - current frames per second
//
// Parameters:
//  [in]    pFrame          The frame to work on
//
void FrameObserver::ShowFrameInfos( const FramePtr &pFrame ) 
{
    bool                bShowFrameInfos     = false;
    VmbUint64_t         nFrameID            = 0;
    bool                bFrameIDValid       = false;
    VmbFrameStatusType  eFrameStatus        = VmbFrameStatusComplete;
    bool                bFrameStatusValid   = false;
    VmbErrorType        res                 = VmbErrorSuccess;
    double              dFPS                = 0.0;
    bool                bFPSValid           = false;
    VmbUint64_t         nFramesMissing      = 0;

    res = pFrame->GetFrameID( nFrameID );
    if( VmbErrorSuccess == res )
    {
        bFrameIDValid = true;

        if( m_FrameID.IsValid() )
        {
            if( nFrameID != ( m_FrameID() + 1 ) )
            {
                nFramesMissing = nFrameID - m_FrameID() - 1;
                if( 1 == nFramesMissing )
                {
                    std::cout<<"1 missing frame detected\n";
                }
                else
                {
                    std::cout<<nFramesMissing<<"missing frames detected\n";
                }
            }
        }

        m_FrameID( nFrameID );
        double dFrameTime = GetTime();
        if(     ( m_FrameTime.IsValid() )
            &&  ( 0 == nFramesMissing ) )
        {
            double dTimeDiff = dFrameTime - m_FrameTime();
            if( dTimeDiff > 0.0 )
            {
                dFPS = 1.0 / dTimeDiff;
                bFPSValid = true;
            }
            else
            {
                bShowFrameInfos = true;
            }
        }

        m_FrameTime( dFrameTime );
    }
    else
    {
        bShowFrameInfos = true;
        m_FrameID.Invalidate();
        m_FrameTime.Invalidate();
    }

    res = pFrame->GetReceiveStatus( eFrameStatus );
    if( VmbErrorSuccess == res )
    {
        bFrameStatusValid = true;

        if( VmbFrameStatusComplete != eFrameStatus )
        {
            bShowFrameInfos = true;
        }
    }
    else
    {
        bShowFrameInfos = true;
    }
    if( 1 )
    {
        std::cout<<"Frame ID:";
        if( bFrameIDValid )
        {
            std::cout<<nFrameID;
        }
        else
        {
            std::cout<<"?";
        }

        std::cout<<" Status:";
        if( bFrameStatusValid )
        {
            PrintFrameStatus( eFrameStatus);
        }
        else
        {
            std::cout<<"?";
        }
        PrintFrameInfo( pFrame );
        
        std::cout<<" FPS:";
        if( bFPSValid )
        {
            std::streamsize s = std::cout.precision();
            std::cout<<std::fixed<<std::setprecision(2)<<dFPS<<std::setprecision(s);
        }
        else
        {
            std::cout<<"?";
        }

        std::cout<<"\n";
    }
    else
    {
        std::cout<<".";
    }
}

//
// This is our callback routine that will be executed on every received frame.
// Triggered by the API.
//
// Parameters:
//  [in]    pFrame          The frame returned from the API
//
void FrameObserver::FrameReceived( const FramePtr pFrame )
{
    VmbUchar_t *pImage = NULL; // frame data will be put here to be converted to cv::Mat
    if(! SP_ISNULL( pFrame ) )
    {
        if( cam_param.show_frame_info )
        {
            ShowFrameInfos( pFrame);
        }
        VmbFrameStatusType status;
        VmbErrorType Result;
        Result = SP_ACCESS( pFrame)->GetReceiveStatus( status);
        if( VmbErrorSuccess == Result && VmbFrameStatusComplete == status)
        {
            if (VmbErrorSuccess == pFrame->GetImage(pImage))
			{
				cv::Mat image = cv::Mat(height, width, CV_8UC1, pImage);
				cv::cvtColor(image, image, cv::COLOR_BayerBG2RGB);
				m_pCamera->QueueFrame(pFrame);   // I can queue frame here because image is already transformed.
                //cv::imshow("image", image);
                //cv::waitKey(1);
                mp.PublishImage(image);		
			}
        }
        else
        {
            std::cout<<"frame incomplete\n";
        }
    }
    else
    {
        std::cout <<" frame pointer NULL\n";
    }

    m_pCamera->QueueFrame( pFrame );
}
}}} // namespace AVT::VmbAPI::Examples
