/**
 * @brief       Utility functions for integration tests
 * @file        utilities.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include <stdio.h>
#include <vector>
#include <string>

// Function to retrieve string within vector of strings as a vector.  
///
/// \brief      Each node contains a comma delimited list of bracketed strings. This method extracts one of those bracketed
///  strings out as a vector rather than just a string. 
///
/// \details    N/A
/// \param[in]  src (Vector of strings containing the bracketed strings)
/// \param[out]  dest (Vector holding extracted string as a vector)
/// \return     N/A
///
void getDataVector(unsigned int idx, std::vector<std::string> &src, std::vector<std::string> &dest);

// Function to retrieve string within vector of strings as a float. 
///
/// \brief      N/A 
///
/// \details    N/A
/// \param[in]  src (Vector of strings containing the float value as a string)
/// \param[out]  dest (Vector holding extracted string as a vector)
/// \return     N/A
///
void getDataFloat(unsigned int idx, std::vector<std::string> &src, float &dest);

// Function to retrieve string within vector of strings as an integer. 
///
/// \brief      N/A 
///
/// \details    N/A
/// \param[in]  src (Vector of strings containing the integer value as a string)
/// \param[out]  dest (Vector holding extracted string as a vector)
/// \return     N/A
///
void getDataInt(unsigned int idx, std::vector<std::string> &src, int &dest);

// Function to retrieve string within vector of strings as an string. 
///
/// \brief      N/A 
///
/// \details    N/A
/// \param[in]  src (Vector of strings containing the string)
/// \param[out]  dest (Vector holding extracted string as a vector)
/// \return     N/A
///
void getDataString(unsigned int idx, std::vector<std::string> &src, std::string &dest);
