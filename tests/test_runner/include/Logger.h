/**
 * @brief       Header defining Logger class, a class used for generating log files. 
 * @file        Logger.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @Reference   Shores, K. (2017) Logger source code 
 * (https://codereview.stackexchange.com/questions/147882/simple-log-writer-class)
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <string>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <exception>
#include <boost/filesystem.hpp>
#include <vector>

class Logger
{
    public:
        Logger(); ///< Default constructor
        virtual ~Logger(); ///< Destructor

        // Function to open file for writing.  
        ///
        /// \brief      N/A
        ///
        /// \details    N/A
        /// \param[in]  N/A
        /// \return     Boolean containing true on a successful open or true if the logger is already open.
        /// Contains false otherwise.
        ///
        bool open();

        // Function to open file for writing.  
        ///
        /// \brief      N/A
        ///
        /// \details    N/A
        /// \param[in]  pPath    log filename
        /// \return     Boolean containing true on a successful open or true if the logger is already open.
        /// Contains false otherwise.
        ///
        bool open(std::string pPath);

        // Function to open file for writing.  
        ///
        /// \brief      N/A
        ///
        /// \details    N/A
        /// \param[in]  pPath    filesystem path object for log file.
        /// \return     Boolean containing true on a successful open or true if the logger is already open.
        /// Contains false otherwise.
        ///
        bool open(const boost::filesystem::path pPath);

        // Function to write string to log file.  
        ///
        /// \brief      N/A
        ///
        /// \details    N/A
        /// \param[in]  pMessage    String to write to log file.
        /// \return     N/A
        ///
        void writeLog(const std::string& pMessage);

        // Function to write vector of strings to log file.  
        ///
        /// \brief      N/A
        ///
        /// \details    N/A
        /// \param[in]  pMessages    Vector of strings to write to log file.
        /// \return     N/A
        ///
        void writeLog(const std::vector<std::string>& pMessages);

        // Function to close log file.  
        ///
        /// \brief      N/A
        ///
        /// \details    N/A
        /// \param[in]  N/A
        /// \return     Boolean representing success or failure in closing log file.
        ///
        bool close();

    private:
        bool createAndOpen();

    private:
        boost::filesystem::path mDir;
        boost::filesystem::path mFile;
        boost::filesystem::path mFull_path;
        std::ofstream mStream;

        // default values
        std::string mDefault_dir = "./";
        std::string mDefault_file = "log.log";
};
#endif