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
 * @file FilterQueue.hpp
 *
 * @brief Data queue for high order filter.
 *
 * @author cfrpg <cfrpg@hotmail.com>
 */

#pragma once

#include <float.h>

using namespace math;

template <typename T=float>
class FilterQueue
{
public:
	FilterQueue()
	{
		Clear();
	}


	~FilterQueue() = default;

	void Clear(void)
	{
		count = 0;
		head = 0;
		tail = 0;
		for(int i=0;i<16;i++)
			val[i]=0;
	}

	void Enqueue(T v)
	{
		if (((tail + 1) & 0x0F) == head)
		{
			return;
		}

		val[tail] = v;
		tail = (tail + 1) & 0x0F;
		count++;
	}

	T Dequeue(void)
	{
		if (tail == head)
		{
			return 0;
		}

		float r = val[head];
		head = (head + 1) & 0x0F;
		count--;
		return r;
	}

	const T operator[](int32_t i) const
	{
		if (i >= count)
		{
			return 0;
		}

		return val[(head + i) & 0x0F];
	}

	int16_t Count()
	{
		return count;
	}

private:
	T val[16];

	int16_t count;
	int16_t head;
	int16_t tail;
};
