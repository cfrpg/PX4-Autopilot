/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/phase_sensor.h>
#include "lib/mathlib/filters.h"

using namespace time_literals;

class PhaseSensor : public ModuleBase<PhaseSensor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	PhaseSensor();
	~PhaseSensor() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Publications
	uORB::Publication<phase_sensor_s> _phase_sensor_pub{ORB_ID(phase_sensor)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PS_SIN_CHANNEL>) _param_ps_sin_ch,
		(ParamInt<px4::params::PS_COS_CHANNEL>) _param_ps_cos_ch,
		(ParamFloat<px4::params::PS_SIN_OFF>) 	_param_ps_sin_off,
		(ParamFloat<px4::params::PS_SIN_SCL>) 	_param_ps_sin_scl,
		(ParamFloat<px4::params::PS_COS_OFF>) 	_param_ps_cos_off,
		(ParamFloat<px4::params::PS_COS_SCL>) 	_param_ps_cos_scl,
		(ParamFloat<px4::params::PS_ANG_OFF>) 	_param_ps_ang_off,
		(ParamFloat<px4::params::PS_UP_ANG>) 	_param_ps_up_ang,
		(ParamFloat<px4::params::PS_DOWN_ANG>) 	_param_ps_down_ang
	)


	bool _armed{false};
	float _sinmax{0}, _sinmin{10};
	float _cosmax{0}, _cosmin{10};
	float _sin{0}, _cos{0};
	uint64_t lastTime{0};
	float timeInterval{0};

	phase_sensor_s ps = phase_sensor_s();

	Derivator<float> dsin;
	Derivator<float> dcos;
	ButterLPF<float> wlpf;
};
