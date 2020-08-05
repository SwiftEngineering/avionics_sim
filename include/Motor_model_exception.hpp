/**
 * @brief       Motor Model Exception class
 * @file        Motor_model_exception.hpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "AvionicsSimException.hpp"
using namespace std;

class Motor_model_exception: public AvionicsSimException
{
    public:
        
        // Constructor (C string).   
        ///
        /// \brief      Constructor accepting a C String as input for exception message. 
        ///
        /// \details   The string contents are copied upon construction. Hence, responsibility for deleting the char* lies with the caller. 
        /// \param[in]  message     C-style string error message
        /// \return      Instance of exception class
        /// 
        explicit Motor_model_exception(const char* message, bool _isCritical=false):AvionicsSimException(message, _isCritical)
        {
                msg_=message;
                critical=_isCritical;
        }

        // Constructor (STL C++ string).   
        ///
        /// \brief      Constructor accepting a STL C++ string as input for exception message. 
        ///
        /// \details    N/A 
        /// \param[in]  message     STL C++ string error message
        /// \return      Instance of exception class
        /// 
        explicit Motor_model_exception(const std::string& message, bool _isCritical=false):AvionicsSimException(message, _isCritical)
        {
                msg_=message;
                critical=_isCritical;
        }

        // Destructor.  
        ///
        /// \brief      Destructor
        ///
        /// \details    Made virtual to allow for subclass destructor handling. 
        /// \param[in]  N/A
        /// \return     N/A
        virtual ~Motor_model_exception() throw (){}

    protected:
};