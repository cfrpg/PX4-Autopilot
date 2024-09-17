/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file MathHelper.hpp
 *
 * collection of more mathematical functions that get used over and over again
 */

#pragma once

#include "Limits.hpp"

#include <px4_platform_common/defines.h>
#include <matrix/matrix/math.hpp>

namespace math
{

const float PIf = 3.1415926f;
const float TwoPIf = 2 * PIf;
const double PI = 3.1415629535897932;


/**
 * Angular distance between two normalized angle a and b (a-b)
 *
 * @param a first angle
 * @param b second angle
 * @return distance of two angle
 */
inline float AngleSub(float a, float b)
{
	if (a > b)
	{
		return a - b;
	}

	return a + math::TwoPIf - b;
}

/**
 * Normalize an angle to [-PI,PI]
 *
 * @param a Original angle
 * @return Normalized angle
 */
inline float AngleNormalize(float a)
{
	if (a > math::PIf)
	{
		return a - math::TwoPIf;
	}

	if (a < -math::PIf)
	{
		return a + math::TwoPIf;
	}

	return a;
}

/**
 * Normalize an angle to [0,2*PI]
 *
 * @param a Original angle
 * @return Normalized angle
 */
inline float AngleToPostitve(float a)
{
	if (a < 0)
	{
		return a + math::TwoPIf;
	}

	return a;
}

const static double BinomialCoefficients[][11] =
{
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0},
	{1, 3, 3, 1, 0, 0, 0, 0, 0, 0, 0},
	{1, 4, 6, 4, 1, 0, 0, 0, 0, 0, 0},
	{1, 5, 10, 10, 5, 1, 0, 0, 0, 0, 0},
	{1, 6, 15, 20, 15, 6, 1, 0, 0, 0, 0},
	{1, 7, 21, 35, 35, 21, 7, 1, 0, 0, 0},
	{1, 8, 28, 56, 70, 56, 28, 8, 1, 0, 0},
	{1, 9, 36, 84, 126, 126, 84, 36, 9, 1, 0},
	{1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1}
};

template<typename T>
void Binomial(int32_t n, int8_t sign, T res[])
{
	for (int8_t i = 0; i < n + 1; i++)
	{
		res[i] = (T)math::BinomialCoefficients[n][i];
	}

	if (sign < 0)
	{
		for (int8_t i = 0; i < n + 1; i += 2)
		{
			res[i] *= -1;
		}
	}
}

template<typename T>
void PolynomialMul(const T p1[], const T p2[], uint8_t n1, uint8_t n2, T res[])
{
	uint8_t i, j;

	for (i = 0; i < n1 + n2 - 1; i++)
	{
		res[i] = 0;
	}

	for (i = 0; i < n1; i++)
	{
		for (j = 0; j < n2; j++)
		{
			res[i + j] += p1[i] * p2[j];
		}
	}
}

template<typename T = float>
class Derivator
{
public:
	Derivator() = default;
	Derivator(T _dt, T _hpf)
	{
		dt = _dt;
		hpf = _hpf;
		val = 0;
		lastData = 0;
	}

	~Derivator() = default;

	T Process(T v)
	{
		val = hpf * (v - lastData) + val;
		val /= (1 + hpf);
		lastData = v;
		return val;
	}

	T Value()
	{
		return val;
	}

private:
	T val;
	T lastData;
	T dt;
	T hpf;
};


static constexpr double butterPb[10][10] =
{
	{1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{1.0, 1.4142136, 0, 0, 0, 0, 0, 0, 0, 0},
	{1.0, 2.0, 2.0, 0, 0, 0, 0, 0, 0, 0},
	{1.0, 2.6131259, 3.4142136, 2.6131259, 0, 0, 0, 0, 0, 0},
	{1.0, 3.236068, 5.236068, 5.236068, 3.236068, 0, 0, 0, 0, 0},
	{1.0, 3.8637033, 7.4641016, 9.1416202, 7.4641016, 3.8637033, 0, 0, 0, 0},
	{1.0, 4.4939592, 10.0978347, 14.5917939, 14.5917939, 10.0978347, 4.4939592, 0, 0, 0},
	{1.0, 5.1258309, 13.1370712, 21.8461510, 25.6883559, 21.8461510, 13.1370712, 5.1258309, 0, 0},
	{1.0, 5.7587705, 16.5817187, 31.1634375, 41.9863857, 41.9863857, 31.1634375, 16.5817187, 5.7587705, 0},
	{1.0, 6.3924532, 20.4317291, 42.8020611, 64.8823963, 74.2334292, 64.8823963, 42.8020611, 20.4317291, 6.3924532}
};

} /* namespace math */
