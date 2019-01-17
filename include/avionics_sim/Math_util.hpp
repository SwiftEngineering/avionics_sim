/**
 * @brief       Math_util
 * @file        Math_util.hpp
 * @author      Jacob Schloss <jschloss@swiftengineering.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <ctgmath>
#include <numeric>

namespace avionics_sim
{

class Math_util
{
public:

	///
	/// linear_map
	///
	/// Map a value linearly from one range to another range, handling offset and scale.
	/// Input and output ranges may be postive or negative, and may be in any order to allow reflecting a range
	///
	/// \param [in] x			The input number
	/// \param [in] in_min		The start of the input range
	/// \param [in] in_max		The end of the input range
	/// \param [in] out_min		The start of the output range
	/// \param [in] out_max		The end of the output range
	/// \return					The corresponding value in the output range
	///
	template<typename T>
	static T linear_map(const T& x, const T& in_min, const T& in_max, const T& out_min, const T& out_max)
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	///
	/// pow_integer
	///
	/// Computes x^n, where n is a non-negative integer and ^ is the exponent operator
	/// More correct than std::pow if x is also an integer, especially if x or n is a large integer
	/// Possibly faster than std::pow when n is a non-negative integer
	///
	/// \param [in] x	The base number
	/// \param [in] n	The exponent
	/// \return			x^n
	///
	template<typename T>
	static T pow_integer(const T& x, const size_t n)
	{
		if(n == 0)
		{
			return T(1);
		}

		T accum = x;
		for(size_t i = 1; i < n; i++)
		{
			accum *= x;
		}

		return accum;
	}

	///
	/// calculate_rms
	///
	/// Compute the RMS value of a given array
	///
	/// \param [in] first	First in sequence
	/// \param [in] last	last in sequence
	/// \param [in] count	number of elements
	/// \return			sqrt(sum(x_i^2) / N)
	///
	template< class InputIt, class AccumType = typename std::iterator_traits<InputIt>::value_type, class ResultType = AccumType >
	static ResultType calculate_rms(InputIt first, InputIt last, const size_t count)
	{
		const AccumType sum_sq = std::accumulate<InputIt, AccumType>(first, last, AccumType(0), [](const AccumType& a, const AccumType& b){return a + b*b;});

		return sqrt(ResultType(sum_sq) / ResultType(count));
	}

	///
	/// calculate_rms
	///
	/// Compute the RMS value of a given array. Number of elements determined by std::distance.
	///
	/// \param [in] first	First in sequence
	/// \param [in] last	last in sequence
	/// \return			sqrt(sum(x_i^2) / N)
	///
	template< class InputIt, class AccumType = typename std::iterator_traits<InputIt>::value_type, class ResultType = AccumType >
	static ResultType calculate_rms(InputIt first, InputIt last)
	{
		const size_t count = std::distance(first, last);

		return calculate_rms<InputIt, AccumType, ResultType>(first, last, count);
	}

	///
	/// calculate_mean
	///
	/// Compute the mean value of a given array
	///
	/// \param [in] first	First in sequence
	/// \param [in] last	last in sequence
	/// \param [in] count	number of elements
	/// \return				sum(x_i) / N
	///
	template< class InputIt, class AccumType = typename std::iterator_traits<InputIt>::value_type, class ResultType = AccumType >
	static ResultType calculate_mean(InputIt first, InputIt last, const size_t count)
	{
		const AccumType sum = std::accumulate<InputIt, AccumType>(first, last, AccumType(0), [](const AccumType& a, const AccumType& b){return a + b;});

		return ResultType(sum) / ResultType(count);
	}

	///
	/// calculate_mean
	///
	/// Compute the mean value of a given array. Number of elements determined by std::distance.
	///
	/// \param [in] first	First in sequence
	/// \param [in] last	last in sequence
	/// \return				sum(x_i) / N
	///
	template< class InputIt, class AccumType = typename std::iterator_traits<InputIt>::value_type, class ResultType = AccumType >
	static ResultType calculate_mean(InputIt first, InputIt last)
	{
		const size_t count = std::distance(first, last);

		return calculate_mean<InputIt, AccumType, ResultType>(first, last, count);
	}

protected:

};

}