/**
 * @brief       Lift Drag Model Exception class
 * @file        LiftDragModelException.hpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include "AvionicsSimException.hpp"

namespace std {

class LiftDragModelException: public AvionicsSimException {
  public:
    // Constructor (C string).
    ///
    /// \brief      Constructor accepting a C String as input for exception message.
    ///
    /// \details    The string contents are copied upon construction. Hence, responsibility for deleting the char* lies
    ///             with the caller.
    /// \param[in]  message     C-style string error message
    /// \return     Instance of exception class
    ///
    explicit LiftDragModelException(const char *message, bool _isCritical = false): AvionicsSimException(message) {
        msg_ = message;
        critical = _isCritical;
    }

    // Constructor (STL C++ string).
    ///
    /// \brief      Constructor accepting a STL C++ string as input for exception message.
    ///
    /// \details    N/A
    /// \param[in]  message     STL C++ string error message
    /// \return      Instance of exception class
    ///
    explicit LiftDragModelException(const std::string &message, bool _isCritical = false):
        AvionicsSimException(message) {
        msg_ = message;
        critical = _isCritical;
    }

    // Destructor.
    ///
    /// \brief      Destructor
    ///
    /// \details    Made virtual to allow for subclass destructor handling.
    /// \param[in]  N/A
    /// \return     N/A
    virtual ~LiftDragModelException() throw() {}

  protected:
};

}  // namespace std
