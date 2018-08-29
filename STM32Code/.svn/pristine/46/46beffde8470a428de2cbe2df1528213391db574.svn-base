/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file accelerometer_calibration.h
 *
 * Definition of accelerometer calibration.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef ACCELEROMETER_CALIBRATION_H_
#define ACCELEROMETER_CALIBRATION_H_

#include <rtthread.h>
#include "calibration_messages.h"
#include "mpu6500.h"

// 校准面
static const unsigned detect_orientation_side_count = 6;

// 用于传送至校准程序的数据
typedef struct  {
	unsigned	done_count; // 完成的面校准
	float		accel_ref[detect_orientation_side_count][3]; // 加速计校准数据
} cal_worker_data_t;	

// The order of these cannot change since the calibration calculations depend on them in this order
enum detect_orientation_return {
	DETECT_ORIENTATION_TAIL_DOWN,
	DETECT_ORIENTATION_NOSE_DOWN,
	DETECT_ORIENTATION_LEFT,
	DETECT_ORIENTATION_RIGHT,
	DETECT_ORIENTATION_UPSIDE_DOWN,
	DETECT_ORIENTATION_RIGHTSIDE_UP,
	DETECT_ORIENTATION_ERROR
};


/// Wait for vehicle to become still and detect it's orientation
///	@return Returns detect_orientation_return according to orientation when vehicle
///		and ready for measurements
enum detect_orientation_return detect_orientation(int	mavlink_fd,			///< Mavlink fd to write output to
						  int	cancel_sub,			///< Cancel subscription from calibration_cancel_subscribe
						  int	accel_sub,			///< Orb subcription to accel sensor
						  bool	lenient_still_detection);	///< true: Use more lenient still position detection

/// Returns the human readable string representation of the orientation
///	@param orientation Orientation to return string for, "error" if buffer is too small
const char* detect_orientation_str(enum detect_orientation_return orientation);

enum calibrate_return {
	calibrate_return_ok,
	calibrate_return_error,
	calibrate_return_cancelled
};



class Accel_Cal
{
private:

	calibrate_return do_accel_calibration_measurements(float (&accel_offs)[3], float (&accel_T)[3][3]);

	calibrate_return read_accelerometer_avg(float (&accel_avg)[detect_orientation_side_count][3], 
                                          unsigned orient, unsigned samples_num);

	calibrate_return calculate_calibration_values(float (&accel_ref)[detect_orientation_side_count][3], 
                                              float (&accel_T)[3][3], float (&accel_offs)[3], float g);

	calibrate_return accel_calibration_worker(detect_orientation_return orientation, void* data);
    
    calibrate_return calibrate_from_orientation( bool	side_data_collected[detect_orientation_side_count],
                                             void*	worker_data,
                                             bool	lenient_still_position);
    const char* detect_orientation_str(enum detect_orientation_return orientation);

    enum detect_orientation_return detect_orientation(bool lenient_still_position);

    int sphere_fit_least_squares(const float x[], const float y[], const float z[],
			     unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z,
			     float *sphere_radius);
    int mat_invert3(float src[3][3], float dst[3][3]);
    MPU6500& accel_mpu;
public:
	Accel_Cal(MPU6500& accel):accel_mpu(accel)
    {
        
    }
	rt_err_t do_accel_calibration(void);
};

#endif /* ACCELEROMETER_CALIBRATION_H_ */
