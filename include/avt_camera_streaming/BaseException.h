/*=============================================================================
  Copyright (C) 2014 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        BaseException.h

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

#ifndef BASE_EXCEPTION_H_
#define BASE_EXCEPTION_H_

#include <exception>
#include <string>

#include "VimbaCPP/Include/VimbaCPP.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

class BaseException: public std::exception
{
    std::string     m_Function;
    std::string     m_Message;
    VmbErrorType    m_Result;
public:
    BaseException( const char* const &fun, const char* const& msg, VmbErrorType result )
        : m_Result ( result )
    {
        try
        {
            if( NULL != fun)
            {
                m_Function = std::string( fun );
            }
        }
        catch(...) {}
        try
        {
            if( NULL != msg)
            {
                m_Message = std::string( msg );
            }
        }
        catch(...) {}
    }
    ~BaseException() throw()
    {}
    const std::string& Function() const
    {
        return m_Function;
    }
    const std::string& Message() const
    {
        return m_Message;
    }
    VmbErrorType Result() const
    {
        return m_Result;
    }
};

}}}

#endif