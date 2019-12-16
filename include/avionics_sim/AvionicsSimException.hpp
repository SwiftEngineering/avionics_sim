/**
 * @brief       Generic Avionics SIM Exception class
 * @file        AvionicsSimException.hpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 * @detail      Adapted from https://stackoverflow.com/questions/8152720/correct-way-to-inherit-from-stdexception. Parameterized try-catch method adapted from 
 * https://stackoverflow.com/questions/3561659/how-can-i-abstract-out-a-repeating-try-catch-pattern-in-c
 */

#include <exception>
#include <sstream>
#include <string>
using namespace std;

class AvionicsSimException: public std::exception
{
    public:
        
        // Constructor (C string).   
        ///
        /// \brief      Constructor accepting a C String as input for exception message. 
        ///
        /// \details   The string contents are copied upon construction. Hence, responsibility for deleting the char* lies with the caller. The explicit keyword here is used to prevent implicit conversion of single argument.
        /// \param[in]  message     C-style string error message
        /// \return      Instance of exception class
        /// 
        explicit AvionicsSimException(const char* message, bool _isCritical=false)
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
        ///             isCritical  boolean indicating criticality of exception.
        /// \return     Instance of exception class
        /// 
        explicit AvionicsSimException(const std::string& message, bool _isCritical=false)
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
        virtual ~AvionicsSimException() throw (){}

        // Returns a pointer to the (constant) error description.  
        ///
        /// \brief      Returns a pointer to the (constant) error description.
        ///
        /// \details    Returns error message as const char*. 
        /// \param[in]  N/A
        /// \return     A pointer to a const char*. The underlying memory
        ///             is in posession of the Exception object. Callers must
        ///             not attempt to free the memory.
        virtual const char* what() const throw (){
            return msg_.c_str();
        }

        // Returns a boolean indicating whether the exception is critical or not. 
        ///
        /// \brief      Returns a boolean indicating whether the exception is critical or not.
        ///
        /// \details    N/A. 
        /// \param[in]  N/A
        /// \return     Boolean indicating whether exception is critical or not.
        virtual const bool isCritical() const throw (){
            return critical;
        }

    protected:
        
        /// \brief Error message
        std::string msg_;

        /// \brief Flag indicating whether or not exception is a critical one.
        bool critical;
};

// Class for templatized try catch calls.
//TODO: Had issues having templatized member object, so must always pass in as first parameter to try-catch. In the future, consider a workaround.
class AvionicsSimTryCatchBlock
{
    public:
        AvionicsSimTryCatchBlock() : 
        exceptionMsg(""), 
        exceptionOccurred(false),
        isCritical(false)
        {}

        // Templatized try catch function. 
        ///
        /// \brief      Templatizes try-catch statements. 
        ///
        /// \details    Should be used in cases when there is nothing specific to do in catch statement other than get error message and criticality. Currently only accepts methods with no parameters. Designed to be used in a method that calls several functions that require being called in a try-catch.
        /// \param[in]  obj     Pointer to class object containing method that requires a try-catch
        /// \param[in]  ReturnT(ClassT::*func)  method requiring try-catch method.
        /// \param[in]  exceptionMsg  Reference to variable that will hold exception message generated in catch clause.
        /// \param[in]  exceptionOccurred  Reference to variable that will hold result of try-catch (true if exception occurred, false otherwise)
        /// \param[in]  isCritical  Reference to variable that will hold criticality of exception.
        /// \return     Result of function passed in as second parameter.
        template <typename ReturnT, typename ClassT>
        ReturnT tryCatch(ClassT* obj, ReturnT(ClassT::*func)())
        {
            std::stringstream strStream;
            strStream << exceptionMsg << std::endl;
            try {
                return (obj->*func)();
            }
            catch (const AvionicsSimException& ex) {
                strStream << ex.what();
                exceptionMsg=exceptionMsg+strStream.str();
                isCritical=isCritical|ex.isCritical();
            }
            catch (...) {
                strStream << "Unknown exception occurred.";
                exceptionMsg=strStream.str();
                exceptionMsg=exceptionMsg+strStream.str();
                isCritical=isCritical|true;
            }
            return ReturnT();
        }

        // Sets initial exception message. 
        ///
        /// \brief      N/A
        ///
        /// \details    Sets initial exception message. Good for preparing initial data regarding try catch block (such as name of function calling the blocks)
        /// \param[in]  std::string     string containing initial exception message.
        /// \return     N/A
        void setExceptionMessage(std::string msg) {exceptionMsg=msg;};

        // Returns a string containing all the exception messages captured. 
        ///
        /// \brief      N/A
        ///
        /// \details    N/A
        /// \param[in]  N/A
        /// \return     String containing all the exception messages captured. 
        std::string getExceptionMessage() {return exceptionMsg;};

        // Returns a boolean indicating whether an exception has occurred in the try catch blocks. 
        ///
        /// \brief      N/A
        ///
        /// \details    N/A
        /// \param[in]  N/A
        /// \return     Boolean indicating whether an exception has occurred in the try catch blocks.
        bool exceptionHasOccurred() {return exceptionOccurred;};

        // Returns a boolean indicating whether a critical exception has occurred in the try catch blocks. 
        ///
        /// \brief      N/A
        ///
        /// \details    N/A
        /// \param[in]  N/A
        /// \return     Boolean indicating whether a critical exception has occurred in the try catch blocks. 
        bool exceptionIsCritical() {return isCritical;};
    
    private:

        //Exception messages collected
        std::string exceptionMsg;

        //Flag indicating whether or not an exception has occured.
        bool exceptionOccurred;

        //Flag indicating whether or not exception is a critical one.
        bool isCritical;
};
