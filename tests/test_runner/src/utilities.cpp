/**
 * @brief       Utility functions for integration tests
 * @file        utilities.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "utilities.h"
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

void getDataVector(unsigned int idx, std::vector<std::string> &src, std::vector<std::string> &dest)
{
	std::string tmp=src.at(idx);
	boost::char_separator<char> sep(",");
	boost::tokenizer<boost::char_separator<char>> tokens(tmp, sep);
	for (const auto& t : tokens) {
		dest.push_back(t);
	}
}

void getDataFloat(unsigned int idx, std::vector<std::string> &src, float &dest)
{
	float tmp=std::stof(src.at(idx));
	dest=tmp;
}

//Included for completeness.
void getDataInt(unsigned int idx, std::vector<std::string> &src, int &dest)
{
	int tmp=std::stoi(src.at(idx));
	dest=tmp;
}

void getDataString(unsigned int idx, std::vector<std::string> &src, std::string &dest)
{
	std::string tmp=src.at(idx);
	dest=tmp;
}
