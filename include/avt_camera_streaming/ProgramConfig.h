/*=============================================================================
  Copyright (C) 2014 - 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ProgrammConfig.h

  Description:

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

#ifndef PROGRAM_CONFIG_H_
#define PROGRAM_CONFIG_H_

#include <cstring>

#include "avt_camera_streaming/BaseException.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

class ProgramConfigException: public BaseException
{
public:
    ProgramConfigException( const char* const &fun, const char* const& msg, VmbErrorType result = VmbErrorOther )
        : BaseException( fun, msg, result )
    {
    }
    ~ProgramConfigException() throw()
    {}
};

enum FrameInfos
{
    FrameInfos_Off,
    FrameInfos_Show,
    FrameInfos_Automatic
};

enum ColorProcessing
{
    ColorProcessing_Off,
    ColorProcessing_Matrix,
};

struct ProgramConfig
{
    FrameInfos          m_FrameInfos;
    bool                m_RGBValue;
    ColorProcessing     m_ColorProcessing;
    std::string         m_CameraID;
    bool                m_PrintHelp;
public:
    ProgramConfig()
        : m_FrameInfos(  AVT::VmbAPI::Examples::FrameInfos_Off )
        , m_RGBValue( false )
        , m_ColorProcessing( ColorProcessing_Off )
        , m_PrintHelp( false )
    {
    }
    VmbErrorType ParseCommandline( int argc, char* argv[] )
    {
        VmbErrorType    Result          = VmbErrorSuccess;
        char *          pParameter      = NULL;                         // The command line parameter
        for( int i = 1; i < argc; ++i )
        {
            pParameter = argv[i];
            if( 0 >= std::strlen( pParameter ))
            {
                return VmbErrorBadParameter;
            }

            if( '/' == pParameter[0] )
            {
                if( 0 == std::strcmp( pParameter, "/i" ))
                {
                    if(    ( FrameInfos_Off != getFrameInfos() )
                        || ( getPrintHelp() ))
                    {
                        return  VmbErrorBadParameter;
                    }

                    setFrameInfos( FrameInfos_Show );
                }
                else if( 0 == std::strcmp( pParameter, "/a" ))
                {
                    if(     ( FrameInfos_Off != getFrameInfos() )
                        ||  ( getPrintHelp() ))
                    {
                        return  VmbErrorBadParameter;
                    }

                    setFrameInfos( FrameInfos_Automatic );
                }
                else if( 0 == std::strcmp( pParameter, "/h" ))
                {
                    if(     ( ! getCameraID().empty() )
                        ||  ( getPrintHelp() )
                        ||  ( AVT::VmbAPI::Examples::FrameInfos_Off != getFrameInfos() ))
                    {
                        return  VmbErrorBadParameter;
                    }

                    setPrintHelp( true );
                }
                else if( 0 == std::strcmp( pParameter, "/r" ))
                {
                    if( getPrintHelp() )
                    {
                        return  VmbErrorBadParameter;
                    }

                    setRGBValue( true );
                }
                else if( 0 == std::strcmp( pParameter, "/c" ))
                {
                    if(     ( ColorProcessing_Off != getColorProcessing() )
                        ||  ( getPrintHelp() ))
                    {
                        return  VmbErrorBadParameter;
                    }

                    setColorProcessing( ColorProcessing_Matrix );
                    setRGBValue( true );
                }
                else
                {
                    return  VmbErrorBadParameter;
                }
            }
            else
            {
                if( !getCameraID().empty() )
                {
                    return  VmbErrorBadParameter;
                }

                setCameraID( pParameter );
            }
        }
        return Result;
    }
    FrameInfos getFrameInfos() const
    {
        return m_FrameInfos;
    }
    void setFrameInfos( FrameInfos infos )
    {
        m_FrameInfos = infos;
    }
    bool getRGBValue() const
    {
        return m_RGBValue;
    }
    void setRGBValue( bool RGBValue )
    {
        m_RGBValue = RGBValue;
    }
    ColorProcessing  getColorProcessing() const
    {
        return m_ColorProcessing;
    }
    void setColorProcessing( ColorProcessing processing )
    {
        m_ColorProcessing = processing;
    }
    const std::string& getCameraID() const
    {
        return m_CameraID;
    }
    void setCameraID( const std::string &name )
    {
        m_CameraID = name;
    }
    void setCameraID( const char* const&name )
    {
        if( NULL != name )
        {
            m_CameraID = std::string( name );
        }
        else
        {
            throw ProgramConfigException(__FUNCTION__,"null pointer in name parameter", VmbErrorBadParameter);
        }
    }
    bool getPrintHelp() const
    {
        return m_PrintHelp;
    }
    void setPrintHelp( bool state )
    {
        m_PrintHelp = state;
    }
    template <typename STREAM_TYPE>
    static STREAM_TYPE& PrintHelp( STREAM_TYPE &s ) 
    {
        s<<"Usage: AsynchronousGrab [CameraID] [/i] [/h]\n";
        s<<"Parameters: CameraID    ID of the camera to use (using first camera if not specified)\n";
        s<<"            /i          Show frame infos\n";
        s<<"            /a          Automatically only show frame infos of corrupt frames\n";
        s<<"            /h          Print out help\n";
        s<<"            /r          Convert to RGB and show RGB values\n";
        s<<"            /c          Color correction (includes /r)\n";
        return s;
    }
};

}}}
#endif