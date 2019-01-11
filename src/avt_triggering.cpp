/*=========================================================
Created by Hui Xiao - University of Connecticuit - 2019
hui.xiao@uconn.edu
===========================================================*/

#include <iostream>
#include <sstream>
#include <cstring>
#include "VimbaCPP/Include/VimbaCPP.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "string.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"
#include "avt_camera_streaming/CamParam.h"
#include "avt_camera_streaming/MessagePublisher.h"
#include "std_msgs/String.h"

/*From Vimba C++ manual: To assure correct continuous image capture, use at least two or three frames. The appropriate number of frames to be queued in your application depends on the frames per second the camera 
delivers and on the speed with which you are able to re-queue frames (also taking into consideration the 
operating system load). The image frames are filled in the same order in which they were queued.*/
#define NUM_OF_FRAMES 3

//define observer that reacts on new frames
class FrameObserver : public AVT::VmbAPI::IFrameObserver
{
public:
    // In contructor call the constructor of the base class
    // and pass a camera object
    FrameObserver( AVT::VmbAPI::CameraPtr pCamera, MessagePublisher& imgPublisher) : IFrameObserver( pCamera ), pImagePublisher(&imgPublisher)
    {
    // Put your initialization code here
    }
    void FrameReceived( const AVT::VmbAPI::FramePtr pFrame )
    {
        VmbUchar_t *pImage = NULL; // frame data will be put here to be converted to cv::Mat
        VmbFrameStatusType eReceiveStatus ;
        if( VmbErrorSuccess == pFrame->GetReceiveStatus( eReceiveStatus ) )
        {
            if ( VmbFrameStatusComplete == eReceiveStatus )
            {
                // Put code here to react on a successfully received frame               
                if (VmbErrorSuccess == pFrame->GetImage(pImage))
			    {
                    VmbUint32_t width=1600;	
                    VmbUint32_t height=1200;
                    pFrame->GetHeight(height);
                    pFrame->GetWidth(width);
                    //ROS_INFO("received an image");
                    cv::Mat image = cv::Mat(height,width, CV_8UC1, pImage);
                    cv::cvtColor(image, image, cv::COLOR_BayerBG2RGB);
                    m_pCamera->QueueFrame(pFrame);   // I can queue frame here because image is already transformed.
                    pImagePublisher->PublishImage(image);
                }
            }
            else
            {
                // Put code here to react on an unsuccessfully received frame
                ROS_INFO("receiving frame failed.");
            }
        }
        // When you are finished copying the frame , re - queue it
        m_pCamera->QueueFrame( pFrame );
    }
private:
    MessagePublisher *pImagePublisher;  // class pointer, will point to the MessagePublisher when initializing
};

class AVTCamera
{
public:
    AVTCamera() : sys(AVT::VmbAPI::VimbaSystem::GetInstance()), frames(NUM_OF_FRAMES), n("~")
    {
        
        getParams(n, cam_param);
        sub = nn.subscribe("trigger", 1, &AVTCamera::triggerCb, this);
    }

    void StartAcquisition();
    void StopAcquisition();
    void SetCameraFeature();
    //call this function triggers an image
    void TriggerImage(); 
private:
    // camera trigger call back
    void triggerCb(const std_msgs::String::ConstPtr& msg);
    // this function fetch parameters from ROS server
    void getParams(ros::NodeHandle &n, CameraParam &cp);
    void SetCameraImageSize(const VmbInt64_t& width, const VmbInt64_t& height);
    void SetExposureTime(const VmbInt64_t & time_in_us); 

    CameraParam cam_param;
    VmbInt64_t nPLS; // Payload size value
    AVT::VmbAPI::FeaturePtr pFeature; // Generic feature pointer
    AVT::VmbAPI::VimbaSystem &sys;
    AVT::VmbAPI::CameraPtr camera;
    AVT::VmbAPI::FramePtrVector frames; // Frame array
    ros::NodeHandle n;   // this will be initialized as n("~") for accessing private parameters
    ros::NodeHandle nn;  // initialized without namespace. 
    ros::Subscriber sub; // subscriber to camera trigger signal
    MessagePublisher image_pub;  // image publisher class. Using image_transport api.
};


void AVTCamera::triggerCb(const std_msgs::String::ConstPtr& msg)
{
    TriggerImage();
}

void AVTCamera::getParams(ros::NodeHandle &n, CameraParam &cam_param)
{
    int height,width,exposure;
    if(n.getParam("cam_IP", cam_param.cam_IP))
    {
        ROS_INFO("Got camera IP %s", cam_param.cam_IP.c_str());
    }
    else
    {
        cam_param.cam_IP = "169.254.49.41";   // dafault IP
        ROS_ERROR("failed to get param 'cam_IP' ");
    }

    if(n.getParam("image_height", height))
    {
        ROS_INFO("Got image_height %i", height);
    }
    else
    {
        height = 1200;
        ROS_ERROR("failed to get param 'image_height' ");
    }

    if(n.getParam("image_width", width))
    {
        ROS_INFO("Got image_width %i", width);
    }
    else
    {
        width = 1600;
        ROS_ERROR("failed to get param 'image_width' ");
    }

    if(n.getParam("exposure_in_us", exposure))
    {
        ROS_INFO("Got exposure_in_us %i", exposure);
    }
    else
    {
        exposure = 10000;
        ROS_ERROR("failed to get param 'exposure_in_us' ");
    }

    if(n.getParam("trigger", cam_param.trigger))
    {
        ROS_INFO("trigger %s", cam_param.trigger ? "enabled" : "disabled");
    }
    else
    {
        cam_param.trigger = false;
        ROS_ERROR("failed to get param 'trigger' ");
    }
    cam_param.image_height = height;
    cam_param.image_width = width;
    cam_param.exposure_in_us = exposure;
}

void AVTCamera::StartAcquisition()
{
    sys.Startup();    
    VmbErrorType res = sys.OpenCameraByID( cam_param.cam_IP.c_str(), VmbAccessModeFull, camera );
    if (VmbErrorSuccess != res)
    {
        ROS_ERROR("failed to open the camera");
    }
    else
    {
        SetCameraFeature();
        camera->GetFeatureByName("PayloadSize", pFeature );
        pFeature->GetValue(nPLS );
        
        for( AVT::VmbAPI::FramePtrVector::iterator iter= frames.begin(); frames.end()!= iter; ++iter)
        {
            (*iter).reset(new AVT::VmbAPI::Frame(nPLS ));
            (*iter)->RegisterObserver(AVT::VmbAPI::IFrameObserverPtr(new FrameObserver(camera,image_pub)));
            camera->AnnounceFrame(*iter );
        }
        
        // Start the capture engine (API)
        camera->StartCapture();
        for( AVT::VmbAPI::FramePtrVector::iterator iter= frames.begin(); frames.end()!=iter; ++iter)
        {
            // Put frame into the frame queue
            camera->QueueFrame(*iter );
        }
        // Start the acquisition engine ( camera )
        camera->GetFeatureByName("AcquisitionStart", pFeature );
        pFeature->RunCommand();
    }
}

void AVTCamera::StopAcquisition()
{
    camera->GetFeatureByName("AcquisitionStop", pFeature );
    pFeature->RunCommand();
    // Stop the capture engine (API)
    // Flush the frame queue
    // Revoke all frames from the API
    camera->EndCapture();
    camera->FlushQueue();
    camera->RevokeAllFrames();
    for( AVT::VmbAPI::FramePtrVector::iterator iter=frames.begin(); frames.end()!=iter; ++iter)
    {
        // Unregister the frame observer / callback
        (*iter)-> UnregisterObserver();
    }
    sys.Shutdown();
}

void AVTCamera::SetCameraImageSize(const VmbInt64_t& width, const VmbInt64_t& height)
{
	// set the offset value to centralize the image.
	VmbInt64_t offset_x = int((1600 - width) / 2);
	VmbInt64_t offset_y = int((1200 - height) / 2);
	VmbErrorType err;
	err = camera->GetFeatureByName("Height", pFeature);
	if (err == VmbErrorSuccess)
	{
		err = pFeature->SetValue(height);
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pFeature->IsCommandDone(bIsCommandDone))
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

	err = camera->GetFeatureByName("Width", pFeature);
	if (err == VmbErrorSuccess)
	{
		err = pFeature->SetValue(width);
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pFeature->IsCommandDone(bIsCommandDone))
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

	
	err = camera->GetFeatureByName("OffsetX", pFeature);
	if (err == VmbErrorSuccess)
	{
		err = pFeature->SetValue(offset_x);
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pFeature->IsCommandDone(bIsCommandDone))
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

	err = camera->GetFeatureByName("OffsetY", pFeature);
	if (err == VmbErrorSuccess)
	{
		err = pFeature->SetValue(offset_y);
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pFeature->IsCommandDone(bIsCommandDone))
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

void AVTCamera::SetExposureTime(const VmbInt64_t & time_in_us)
{
	VmbErrorType err;
	err = camera->GetFeatureByName("ExposureTimeAbs", pFeature);
	if (err == VmbErrorSuccess)
	{
		err = pFeature->SetValue((double)time_in_us); 
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pFeature->IsCommandDone(bIsCommandDone))
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

void AVTCamera::TriggerImage()
{
    VmbErrorType err;
	err = camera->GetFeatureByName("TriggerSoftware", pFeature);
	if (err == VmbErrorSuccess)
	{
		err = pFeature->RunCommand();
		if (VmbErrorSuccess == err)
		{
			bool bIsCommandDone = false;
			do
			{
				if (VmbErrorSuccess != pFeature->IsCommandDone(bIsCommandDone))
				{
					break;
				}
			} while (false == bIsCommandDone);
		}
	}
}

// This must be called after opening the camera.
void AVTCamera::SetCameraFeature()
{
    VmbErrorType err;
    SetExposureTime(cam_param.exposure_in_us);
    SetCameraImageSize(cam_param.image_width, cam_param.image_height);
    // Set acquisition mode
    camera->GetFeatureByName("AcquisitionMode", pFeature);
    err = pFeature->SetValue("Continuous");
    if (VmbErrorSuccess == err)
    {
        bool bIsCommandDone = false;
        do
        {
            if (VmbErrorSuccess != pFeature->IsCommandDone(bIsCommandDone))
            {
                break;
            }
        } while (false == bIsCommandDone);
    }

    // Set Trigger source
    camera->GetFeatureByName("TriggerSource", pFeature);
    if(cam_param.trigger)
    {
        err = pFeature->SetValue("Software");
    }
    else
    {
        err = pFeature->SetValue("Freerun");
    }
    
    if (VmbErrorSuccess == err)
    {
        bool bIsCommandDone = false;
        do
        {
            if (VmbErrorSuccess != pFeature->IsCommandDone(bIsCommandDone))
            {
                break;
            }
        } while (false == bIsCommandDone);
    }
}

int main( int argc, char* argv[])
{
    ros::init(argc, argv, "triggered_avt_camera", ros::init_options::AnonymousName);
    AVTCamera avt_cam;
    avt_cam.StartAcquisition();
    while(ros::ok())
    {
        ros::spinOnce();
    }
    avt_cam.StopAcquisition();
}