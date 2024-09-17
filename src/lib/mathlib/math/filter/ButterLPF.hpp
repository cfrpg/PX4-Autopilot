/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file ButterLPF.hpp
 *
 * @brief Butterworth low pass filter.
 *
 * @author cfrpg <cfrpg@hotmail.com>
 */

#pragma once

#include <float.h>
#include <cmath>
#include <mathlib/math/Functions.hpp>
#include "FilterBase.hpp"
#include "FilterQueue.hpp"
#include <mathlib/math/MathHelper.hpp>
#include <px4_platform_common/log.h>

using namespace math;

template <typename T>
class ButterLPF : public FilterBase<T>
{
public:
	ButterLPF() = default;
	ButterLPF(int32_t _order, double fc, double fs) : xs(), ys()
	{
		int8_t i, j;
		int32_t N = _order;
		order = _order;
		cutoff = fc;
		simplingFreq = fs;

		T b[12] = {T()};
		T tmp1[12] = {T()};
		T tmp2[12] = {T()};
		T tmp3[12] = {T()};

		for (i = 0; i < 12; i++)
		{
			b[i] = 0;
			tmp1[i] = 0;
			tmp2[i] = 0;
			tmp3[i] = 0;
			num[i] = 0;
			den[i] = 0;
		}

		wc = fabs(tan(math::PI * cutoff / simplingFreq));

		for (i = 0; i < N; i++)
		{
			b[i] = (T)(math::butterPb[N - 1][i] * pow(wc, N - i));
		}

		b[N] = (T)1;

		for (i = 0; i <= N; i++)
		{
			math::Binomial<T>(i, -1, tmp1);
			math::Binomial<T>(N - i, 1, tmp2);
			math::PolynomialMul(tmp1, tmp2, i + 1, N - i + 1, tmp3);

			for (j = 0; j < N + 1; j++)
			{
				den[j] += b[i] * tmp3[j];

				if (i == 0)
				{
					num[j] = b[0] * tmp2[j];
				}
			}
		}

		for (i = N; i >= 0; i--)
		{
			num[i] /= den[0];
			den[i] /= den[0];
		}

		xs = FilterQueue<T>();
		ys = FilterQueue<T>();

	}

	~ButterLPF() = default;

	T Process(T val) override
	{
		T res;
		uint8_t i;

		if (ys.Count() < order)
		{
			xs.Enqueue(val);
			res = num[0] * xs[xs.Count() - 1];

			for (i = 1; i < xs.Count() - 1; i++)
			{
				res += num[i] * xs[xs.Count() - 1 - i];
				res -= den[i] * ys[ys.Count() - i];
			}

			ys.Enqueue(res);
			value = val;
			return val;
		}

		xs.Enqueue(val);
		res = num[0] * xs[order];

		for (i = 1; i <= order; i++)
		{
			res += num[i] * xs[xs.Count() - i];
			res -= den[i] * ys[ys.Count() - i];
		}

		res /= den[0];
		xs.Dequeue();
		ys.Dequeue();
		ys.Enqueue(res);
		value = res;
		return res;
	}

	T Value(void)
	{
		return value;
	}
protected:


private:
	int32_t order;
	double cutoff;
	double simplingFreq;
	double wc;

	T num[16];
	T den[16];
	T value;

	FilterQueue<T> xs;
	FilterQueue<T> ys;
};
