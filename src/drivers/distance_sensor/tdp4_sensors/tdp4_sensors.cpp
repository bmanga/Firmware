/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file tdp4_sensors.cpp
 * @author Bruno Manganelli <bruno.manga95@gmail.com>
 *
 * Driver for communication with mbed controller for TDP4 project.
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <px4_defines.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <drivers/device/ringbuffer.h>
#include <stdio.h>
#include <uORB/uORB.h>
#include <termios.h>
#include <drivers/drv_range_finder.h>


#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/tdp4_sensors.h>




#define TDP4_SENSORS_DEVICE_PATH    "/dev/tdp4_sensors"
#define TDP4_SENSORS_I2C_BUS 		PX4_I2C_BUS_EXPANSION

// The address that is advertised by the mbed slave i2c.
#define TDP4_SENSORS_I2C_ADDR          0x70




extern "C" __EXPORT int tdp4_sensors_main(int argc, char *argv[]);

class TDP4_Sensors : public device::I2C
{
public:
	TDP4_Sensors();
	virtual ~TDP4_Sensors();

	virtual int 			init() override;

	int				        start();
	int                     stop();

private:
	static void task_main_trampoline(int argc, char *argv[]);
	void task_main();
	tdp4_sensors_s collect_data();
private:
	int _class_instance = -1;
	int _orb_class_instance = -1;
	int _task_handle = -1;
	bool _task_should_exit = false;
	orb_advert_t _tdp4_sensors_topic = nullptr;
};

namespace tdp4_sensors
{
TDP4_Sensors	*g_dev;
}

TDP4_Sensors::TDP4_Sensors() :
	I2C("TDP4_Sensors", TDP4_SENSORS_DEVICE_PATH, TDP4_SENSORS_I2C_BUS, TDP4_SENSORS_I2C_ADDR, 100000)
{

}

TDP4_Sensors::~TDP4_Sensors()
{
	stop();
	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}
	orb_unadvertise(_tdp4_sensors_topic);
}


int TDP4_Sensors::init()
{
	//int ret = OK;

	if (I2C::init() != OK) {
		return PX4_ERROR;
	}
	// FIXME: Not sure if / why we need to use RANGE_FINDER_BASE_DEVICE_PATH
	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	tdp4_sensors_s ts_report = {};
	_tdp4_sensors_topic = orb_advertise_multi(ORB_ID(tdp4_sensors),
		&ts_report, &_orb_class_instance, ORB_PRIO_HIGH);

	if (_tdp4_sensors_topic == nullptr) {
		DEVICE_LOG("failed to create tdp4_sensors object. Did you start uOrb?");
	}

	// FIXME: Not sure why we need to wait 200ms here, but everyone else does.
	usleep(200000);

	return OK;
}

int TDP4_Sensors::start()
{
	ASSERT(_task_handle == -1);

	_task_should_exit = false;

	/* start the task */
	_task_handle = px4_task_spawn_cmd("tdp4_sensors",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX - 30,
					  800,
					  (px4_main_t)&TDP4_Sensors::task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

int TDP4_Sensors::stop()
{
	_task_should_exit = true;

	for (int cnt = 0; _task_handle != -1 && cnt < 10; ++cnt)
		usleep(50000); // 50ms

	if (_task_handle != -1) {
		px4_task_delete(_task_handle);
	}
	return OK;
}

void TDP4_Sensors::task_main_trampoline(int argc, char *argv[])
{
	tdp4_sensors::g_dev->task_main();
}

void TDP4_Sensors::task_main()
{
	while (!_task_should_exit) {
		usleep(100000); // 10 Hz
		tdp4_sensors_s report = collect_data();
		// A zero'd report means failure in data collection
		if (report.timestamp != 0)
			orb_publish(ORB_ID(tdp4_sensors), _tdp4_sensors_topic, &report);
	}

	// The task is going to exit.
	_task_handle = -1;
}

tdp4_sensors_s TDP4_Sensors::collect_data()
{
	struct tdp_sensor_data
	{
		uint16_t ultrasound_d;
		uint16_t laser1_d;
		uint16_t laser2_d;
		uint16_t laser3_d;
	};

	uint8_t retrieve_data_cmd = 0x02;

	if (transfer(&retrieve_data_cmd, 1, nullptr, 0) < 0) {
		PX4_ERR("could not send data to mbed");
		return {};
	}

	tdp_sensor_data measurements = {};

	static_assert(sizeof(measurements) == 8, "size should be 4 * uint16");
	int res = transfer(nullptr, 0, (uint8_t *)(&measurements), 8);
	if (res != OK) {
		PX4_ERR("could not receive data to mbed");
		return {};
	}

	tdp4_sensors_s report;
	report.timestamp                   = hrt_absolute_time();
	report.current_distance_ultrasound = measurements.ultrasound_d;
	report.current_distance_laser1     = measurements.laser1_d;
	report.current_distance_laser2     = measurements.laser2_d;
	report.current_distance_laser3     = measurements.laser3_d;

	return report;
}

int tdp4_sensors_main(int argc, char *argv[])
{
	if (argc <= 1) {
		PX4_WARN("not enough arguments, usage: start [device_path], stop, info ");
		return 1;
	}

	// check for optional arguments
	int myoptind = 1;



	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (tdp4_sensors::g_dev != nullptr) {
			PX4_WARN("driver already started");
			return 0;
		}

		tdp4_sensors::g_dev = new TDP4_Sensors();


		if (tdp4_sensors::g_dev == nullptr) {
			PX4_ERR("failed to create instance of TDP4 Sensors");
			return 1;
		}

		if (PX4_OK != tdp4_sensors::g_dev->init()) {
			PX4_ERR("failed to init TDP4 Sensors");
			delete tdp4_sensors::g_dev;
			tdp4_sensors::g_dev = nullptr;
			return 1;
		}

		if (OK != tdp4_sensors::g_dev->start()) {
			PX4_ERR("failed to start TDP4 Sensors");
			delete tdp4_sensors::g_dev;
			tdp4_sensors::g_dev = nullptr;
			return 1;
		}

		return 0;
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		if (tdp4_sensors::g_dev != nullptr) {
			delete tdp4_sensors::g_dev;
			tdp4_sensors::g_dev = nullptr;

		} else {
			PX4_WARN("driver not running");
		}

		return 0;
	}

	if (!strcmp(argv[myoptind], "info")) {
		PX4_INFO("Landing sensors for TDP4 project");

		return 0;

	}

	PX4_WARN("unrecognized arguments, try: start [device_path], stop, info ");
	return 1;
}


