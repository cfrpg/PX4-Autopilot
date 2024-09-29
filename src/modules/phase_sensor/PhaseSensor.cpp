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

#include "PhaseSensor.hpp"
#include <px4_platform_common/getopt.h>


PhaseSensor::PhaseSensor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	dsin = Derivator<float>(0, 0.3f);
	dcos = Derivator<float>(0, 0.3f);
	wlpf = ButterLPF<float>(4, 10, 500);
	ps.direction = PhaseSensor::UnknowStroke;
}

PhaseSensor::~PhaseSensor()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool PhaseSensor::init()
{
	// execute Run() on every adc_report publication
	if (!_adc_report_sub.registerCallback())
	{
		PX4_ERR("callback registration failed");
		return false;
	}


	PX4_INFO("Phase sensor started.");
	return true;
}

void PhaseSensor::Run()
{
	if (should_exit())
	{
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated())
	{
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	//  grab latest adc data
	if (_adc_report_sub.updated())
	{
		adc_report_s adc;

		if (_adc_report_sub.copy(&adc))
		{
			timeInterval = (float)(adc.timestamp - lastTime) * 1E-6f;
			// get adc reading
			float adc_k = adc.v_ref / adc.resolution;
			ps.adc_timestamp = adc.timestamp;
			ps.sin_adc_raw = adc.raw_data[_param_ps_sin_ch.get()];
			ps.cos_adc_raw = adc.raw_data[_param_ps_cos_ch.get()];
			ps.sin_raw = adc_k * ps.sin_adc_raw;
			ps.cos_raw = adc_k * ps.cos_adc_raw;
			ps.hall_sin = (ps.sin_raw - _param_ps_sin_off.get()) * _param_ps_sin_scl.get();
			ps.hall_cos = (ps.cos_raw - _param_ps_cos_off.get()) * _param_ps_cos_scl.get();

			// calculate angle
			ps.angle_raw = atan2f(ps.hall_sin, ps.hall_cos);
			ps.flapping_phase = -math::AngleSub(ps.angle_raw, _param_ps_ang_off.get());
			ps.flapping_phase = math::AngleNormalize(ps.flapping_phase);
			ps.flapping_angle = ps.flapping_phase;

			// calculate angular speed
			dsin.Process(ps.hall_sin);
			dcos.Process(ps.hall_cos);
			ps.angular_speed = dsin.Value() * ps.hall_cos - dcos.Value() * ps.hall_sin;
			ps.angular_speed = -wlpf.Process(ps.angular_speed) / timeInterval;

			// update flapping direction
			if (ps.direction == UnknowStroke)
			{
				if (fabsf(ps.flapping_phase) < math::PIf / 2)
				{
					ps.direction = DownStroke;
				}

				else
				{
					ps.direction = UpStroke;
				}
			}

			else if (ps.direction == DownStroke)
			{
				if (fabsf(ps.flapping_phase) > math::PIf / 2)
				{
					ps.direction = UpStroke;
				}
			}

			else if (ps.direction == UpStroke)
			{
				if (fabsf(ps.flapping_phase) < math::PIf / 2)
				{
					ps.direction = DownStroke;
				}
			}

			// update sensor reading range
			_sinmax = math::max(_sinmax, ps.sin_raw);
			_sinmin = math::min(_sinmin, ps.sin_raw);
			_cosmax = math::max(_cosmax, ps.cos_raw);
			_cosmin = math::min(_cosmin, ps.cos_raw);

			_sin = ps.sin_raw;
			_cos = ps.cos_raw;

			lastTime = adc.timestamp;
		}
	}

	ps.timestamp = hrt_absolute_time();
	_phase_sensor_pub.publish(ps);
	perf_end(_loop_perf);
}

int PhaseSensor::task_spawn(int argc, char *argv[])
{
	PhaseSensor *instance = new PhaseSensor();

	if (instance)
	{
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init())
		{
			return PX4_OK;
		}

	}

	else
	{
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int PhaseSensor::print_status()
{

	PX4_INFO_RAW("Phase sensor status:\n");
	PX4_INFO_RAW("  Sin      : %f, Raw: %f - %f - %f\n", (double)ps.hall_sin, (double)_sinmin, (double)ps.sin_raw,
		     (double)_sinmax);
	PX4_INFO_RAW("  Cos      : %f, Raw: %f - %f - %f\n", (double)ps.hall_cos, (double)_cosmin, (double)ps.cos_raw,
		     (double)_cosmax);
	PX4_INFO_RAW("  Angle    : %f (%f deg)\n", (double)ps.angle_raw, (double)math::degrees(ps.angle_raw));
	PX4_INFO_RAW("  Flapping : %f (%f deg)\n", (double)ps.flapping_angle, (double)math::degrees(ps.flapping_angle));
	PX4_INFO_RAW("  Phase    : %f (%f deg)\n", (double)ps.flapping_phase, (double)math::degrees(ps.flapping_phase));
	PX4_INFO_RAW("  Speed    : %f (%f deg)\n", (double)ps.angular_speed, (double)math::degrees(ps.angular_speed));
	PX4_INFO_RAW("  Direction: %s\n",ps.direction==UnknowStroke?"Unknow":(ps.direction==UpStroke?"Up stroke":"Down stroke"));

	PX4_INFO_RAW("Update inteval: %f\n", (double)timeInterval);
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	return 0;
}

int PhaseSensor::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "test"))
	{
		PX4_INFO_RAW("Hello PSP.\n");

		if (is_running())
		{
			PX4_INFO_RAW("Runing.\n");
		}

		else
		{
			PX4_INFO_RAW("Not runing.\n");
		}

		return 0;
	}

	if (!strcmp(verb, "cali"))
	{
		int myoptind = 1;
		int ch;
		int rflag = 0, cflag = 0;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "arc", &myoptind, &myoptarg)) != EOF)
		{
			switch (ch)
			{
			case 'a':
				rflag = 1;
				cflag = 1;
				break;

			case 'r':
				rflag = 1;
				break;

			case 'c':
				cflag = 1;
				break;

			case '?':
				PX4_INFO_RAW("Unknow flag.\n");
				break;

			default:
				PX4_INFO_RAW("Unrecognized flag\n");
				break;
			}
		}

		PhaseSensor *self = _object.load();

		if (rflag)
		{
			float scl, off;

			PX4_INFO_RAW("Calibrate range:\n");
			scl = 2.0f / (self->_sinmax - self->_sinmin);
			off = (self->_sinmax + self->_sinmin) / 2.0f;
			PX4_INFO_RAW("  sin: scale: %f, offset: %f\n", (double)scl, (double)off);
			self->_param_ps_sin_scl.set(scl);
			self->_param_ps_sin_scl.commit();
			self->_param_ps_sin_off.set(off);
			self->_param_ps_sin_off.commit();

			scl = 2.0f / (self->_cosmax - self->_cosmin);
			off = (self->_cosmax + self->_cosmin) / 2.0f;
			PX4_INFO_RAW("  cos: scale: %f, offset: %f\n", (double)scl, (double)off);
			self->_param_ps_cos_scl.set(scl);
			self->_param_ps_cos_scl.commit();
			self->_param_ps_cos_off.set(off);
			self->_param_ps_cos_off.commit();
		}

		if (cflag)
		{
			PX4_INFO_RAW("Calibrate offset:\n");
			PX4_INFO_RAW("  Phase offset: %f (%f deg)\n", (double)self->ps.angle_raw, (double)math::degrees(self->ps.angle_raw));
			self->_param_ps_ang_off.set(self->ps.angle_raw);
			self->_param_ps_ang_off.commit();
		}

		return 0;
	}

	return print_usage("unknown command");
}

int PhaseSensor::print_usage(const char *reason)
{
	if (reason)
	{
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Phase sensor driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("phase_sensor", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("cali","Calibrate phase sensor");
	PRINT_MODULE_USAGE_PARAM_FLAG('a',"Fully calibration. (Default)",true);
	PRINT_MODULE_USAGE_PARAM_FLAG('r',"Range calibration.",true);
	PRINT_MODULE_USAGE_PARAM_FLAG('c',"Center position calibration.",true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("test","Test sensor.");
	return 0;
}

extern "C" __EXPORT int phase_sensor_main(int argc, char *argv[])
{
	return PhaseSensor::main(argc, argv);
}
