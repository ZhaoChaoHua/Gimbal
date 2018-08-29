/**
 * @file accelerometer_calibration.cpp
 *
 * Implementation of accelerometer calibration.
 *
 * Transform acceleration vector to true orientation, scale and offset
 *
 * ===== Model =====
 * accel_corr = accel_T * (accel_raw - accel_offs)
 *
 * accel_corr[3] - fully corrected acceleration vector in body frame
 * accel_T[3][3] - accelerometers transform matrix, rotation and scaling transform
 * accel_raw[3]  - raw acceleration vector
 * accel_offs[3] - acceleration offset vector
 *
 * ===== Calibration =====
 *
 * Reference vectors
 * accel_corr_ref[6][3] = [  g  0  0 ]     // nose up
 *                        | -g  0  0 |     // nose down
 *                        |  0  g  0 |     // left side down
 *                        |  0 -g  0 |     // right side down
 *                        |  0  0  g |     // on back
 *                        [  0  0 -g ]     // level
 * accel_raw_ref[6][3]
 *
 * accel_corr_ref[i] = accel_T * (accel_raw_ref[i] - accel_offs), i = 0...5
 *
 * 6 reference vectors * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs[i] = (accel_raw_ref[i*2][i] + accel_raw_ref[i*2+1][i]) / 2
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr_ref[i*2] = accel_T * (accel_raw_ref[i*2] - accel_offs), i = 0...2
 *
 * Solve separate system for each row of accel_T:
 *
 * accel_corr_ref[j*2][i] = accel_T[i] * (accel_raw_ref[j*2] - accel_offs), j = 0...2
 *
 * A * x = b
 *
 * x = [ accel_T[0][i] ]
 *     | accel_T[1][i] |
 *     [ accel_T[2][i] ]
 *
 * b = [ accel_corr_ref[0][i] ]	// One measurement per side is enough
 *     | accel_corr_ref[2][i] |
 *     [ accel_corr_ref[4][i] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0;2;4, j = 0...2
 *
 * Matrix A is common for all three systems:
 * A = [ a[0][0]  a[0][1]  a[0][2] ]
 *     | a[2][0]  a[2][1]  a[2][2] |
 *     [ a[4][0]  a[4][1]  a[4][2] ]
 *
 * x = A^-1 * b
 *
 * accel_T = A^-1 * g
 * g = 9.80665
 *
 * ===== Rotation =====
 *
 * Calibrating using model:
 * accel_corr = accel_T_r * (rot * accel_raw - accel_offs_r)
 *
 * Actual correction:
 * accel_corr = rot * accel_T * (accel_raw - accel_offs)
 *
 * Known: accel_T_r, accel_offs_r, rot
 * Unknown: accel_T, accel_offs
 *
 * Solution:
 * accel_T_r * (rot * accel_raw - accel_offs_r) = rot * accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * (rot * accel_raw - accel_offs_r) = accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * rot * accel_raw - rot^-1 * accel_T_r * accel_offs_r = accel_T * accel_raw - accel_T * accel_offs)
 * => accel_T = rot^-1 * accel_T_r * rot
 * => accel_offs = rot^-1 * accel_offs_r
 *
 */

// FIXME: Can some of these headers move out with detect_ move?

#include "accelerometer_calibration.h"
#include "stm32f40x_define.h"
#include "mathlib.h"
#include <stdio.h>
#include <AP_Math.h>
#include <float.h>
#include <string.h>
#include "rotation.h"
#include "parameter.h"


// 进行校准
rt_err_t Accel_Cal::do_accel_calibration(void)
{
    // 发送start信息
	rt_kprintf(CAL_QGC_STARTED_MSG);
    
	rt_err_t res = RT_EOK;

	/* reset all sensors */
	// 初始化加速计的校准值
    
    // 校准数据
	float accel_offs[3];
	float accel_T[3][3];

	/* measure and calculate offsets & scales */
	if (res == RT_EOK) {
        // 加速计校准
		calibrate_return cal_return = do_accel_calibration_measurements(accel_offs, accel_T);
		if (cal_return == calibrate_return_cancelled) {
			// Cancel message already displayed, nothing left to do

			return RT_ERROR;
		} else if (cal_return == calibrate_return_ok) {
			res = RT_EOK;
		} else {

			res = RT_ERROR;
		}
	}
	if (res != RT_EOK)
    {
    
        rt_kprintf(CAL_ERROR_SENSOR_MSG);
		return RT_ERROR;
	}
    
    /* measurements completed successfully, rotate calibration values */
    // 传感器位置
	enum Rotation board_rotation_id = ROTATION_NONE;
	math::Matrix<3, 3> board_rotation;
	get_rot_matrix(board_rotation_id, &board_rotation);
	math::Matrix<3, 3> board_rotation_t = board_rotation.transposed();

    /* handle individual sensors, one by one */
    math::Vector<3> accel_offs_vec(accel_offs);
    math::Vector<3> accel_offs_rotated = board_rotation_t * accel_offs_vec;
    math::Matrix<3, 3> accel_T_mat(accel_T);
    math::Matrix<3, 3> accel_T_rotated = board_rotation_t * accel_T_mat * board_rotation;

    // 参数值提取
//    calibrate.acc_offset(accel_offs[0], accel_offs[1], accel_offs[2]);
//    calibrate.acc_gain(accel_gain[0], accel_gain[1], accel_gain[2]);
//    
//    calibrate.gyr_gain.x = 1.0f/calibrate._gyro_scale * DEG_TO_RAD * 1.0f;
//	calibrate.gyr_gain.y = 1.0f/calibrate._gyro_scale * DEG_TO_RAD * 1.0f;
//	calibrate.gyr_gain.z = 1.0f/calibrate._gyro_scale * DEG_TO_RAD * 1.0f;
//	
//    /* set parameters */
//    // 保存参数
//    res = param.param_set(ACCEL_OFFSET,  calibrate.acc_offset);
//    res = param.param_set(ACCEL_SCALE,   calibrate.acc_gain);
//    
//    //res = param.param_set(GYRO_OFFSET,   calibrate.gyr_offset);
//    res = param.param_set(GYRO_SCALE,    calibrate.gyr_gain);
    
    rt_kprintf("[cal]Save calibration result done\n");
    /* if there is a any preflight-check system response, let the barrage of messages through */
    rt_thread_delay(20);

    rt_kprintf(CAL_QGC_DONE_MSG);

	return res;
}

// 采集数据
calibrate_return Accel_Cal::accel_calibration_worker(detect_orientation_return orientation, void* data)
{
	const unsigned samples_num = 3000;
	cal_worker_data_t* worker_data = (cal_worker_data_t*)(data);
	
	rt_kprintf("[cal] Hold still, measuring %s side\n", detect_orientation_str(orientation));
	
	read_accelerometer_avg(worker_data->accel_ref, orientation, samples_num);
	
	rt_kprintf("[cal] %s side acc result: [%d.%d%d %d.%d%d %d.%d%d]\n", detect_orientation_str(orientation),
				     (int)(worker_data->accel_ref[orientation][0]),(int)(abs(worker_data->accel_ref[orientation][0])*10) % 10,(int)(abs(worker_data->accel_ref[orientation][0])*100) % 10,
				     (int)(worker_data->accel_ref[orientation][1]),(int)(abs(worker_data->accel_ref[orientation][1])*10) % 10,(int)(abs(worker_data->accel_ref[orientation][1])*100) % 10,
				     (int)(worker_data->accel_ref[orientation][2]),(int)(abs(worker_data->accel_ref[orientation][2])*10) % 10,(int)(abs(worker_data->accel_ref[orientation][2])*100) % 10);
	
	worker_data->done_count++;
	rt_kprintf(CAL_QGC_PROGRESS_MSG, 17 * worker_data->done_count);
	
	return calibrate_return_ok;
}

// 加速计数据采集
calibrate_return Accel_Cal::do_accel_calibration_measurements(float (&accel_offs)[3], float (&accel_T)[3][3])
{
	calibrate_return result = calibrate_return_ok;
	
	cal_worker_data_t worker_data;
	// 完成的校准面
	worker_data.done_count = 0;
    // 完成校准的面标志
	bool data_collected[detect_orientation_side_count] = { false, false, false, false, false, false };

    if (result == calibrate_return_ok) 
    {   
		result = calibrate_from_orientation(data_collected, &worker_data, false /* normal still */);
	}
    // 六面数据采集完毕后计算偏移及scale
	if (result == calibrate_return_ok)
    {
		/* 计算偏移和变换矩阵 */
        // error work data get error
        result = calculate_calibration_values(worker_data.accel_ref, accel_T, accel_offs, CONSTANTS_ONE_G);

        if (result != calibrate_return_ok) 
        {

            rt_kprintf("[cal] ERROR: calibration calculation error\n");
        }
	}
	return result;
}

/*
 * Read specified number of accelerometer samples, calculate average and dispersion.
 * SENS_BOARD_ROT      默认参数 0      - 无旋转
 * SENS_BOARD_X_OFF    默认参数 0.0f   - Roll offset
 * SENS_BOARD_Y_OFF    默认参数 0.0f   - Pitch offset
 * SENS_BOARD_Z_OFF    默认参数 0.0f   - Yaw offset
 */
calibrate_return Accel_Cal::read_accelerometer_avg(float (&accel_avg)[detect_orientation_side_count][3], unsigned orient, unsigned samples_num)
{
	float board_offset[3];
    // 默认参数，具体偏移值实际情况定
    board_offset[0] = 0.0f;
    board_offset[1] = 0.0f;
    board_offset[2] = 0.0f;

	math::Matrix<3,3> board_rotation_offset;
	board_rotation_offset.from_euler(DEG_TO_RAD * board_offset[0], DEG_TO_RAD * board_offset[1], DEG_TO_RAD * board_offset[2]);

	int32_t board_rotation_int;
    // 默认参数，无旋转，具体参数参照rotation.h
	board_rotation_int = 0;
	enum Rotation board_rotation_id = (enum Rotation)board_rotation_int;
    
	math::Matrix<3,3> board_rotation;
	get_rot_matrix(board_rotation_id, &board_rotation);

	/* combine board rotation with offset rotation */
	board_rotation = board_rotation_offset * board_rotation;

	unsigned int counts = 0;
	float accel_sum[3];
	memset(accel_sum, 0, sizeof(accel_sum));

	unsigned errcount = 0;

	/* use the first sensor to pace the readout, but do per-sensor counts */
	while (counts < samples_num)
    {
        rt_err_t poll_ret = RT_EOK;
		if (poll_ret == RT_EOK)
        {
            bool changed = true;
            if (changed)
            {
                accel_sum[0] += accel_mpu.Acc_ms2.x;
                accel_sum[1] += accel_mpu.Acc_ms2.y;
                accel_sum[2] += accel_mpu.Acc_ms2.z;

                counts++;
            }
		} else {
			errcount++;
			continue;
		}

		if (errcount > samples_num / 10) {
			return calibrate_return_error;
		}
	}
    
    // rotate sensor measurements from body frame into sensor frame using board rotation matrix
    math::Vector<3> accel_sum_vec(&accel_sum[0]);
    accel_sum_vec = board_rotation * accel_sum_vec;
    memcpy(&accel_sum[0], &accel_sum_vec.data[0], sizeof(accel_sum));
    
    for (unsigned i = 0; i < 3; i++)
    {
        accel_avg[orient][i] = accel_sum[i] / counts;
    }

	return calibrate_return_ok;
}

int Accel_Cal::mat_invert3(float src[3][3], float dst[3][3])
{
	float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
		    src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
		    src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	if (fabsf(det) < FLT_EPSILON) {
		return RT_ERROR;        // Singular matrix
	}

	dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
	dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
	dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
	dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
	dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
	dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
	dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
	dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
	dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

	return RT_EOK;
}

// 加速计校准计算
calibrate_return Accel_Cal::calculate_calibration_values(float (&accel_ref)[detect_orientation_side_count][3], 
                                              float (&accel_T)[3][3], float (&accel_offs)[3], float g)
{
    /* calculate offsets */
	for (unsigned i = 0; i < 3; i++) 
    {
		accel_offs[i] = (accel_ref[i * 2][i] + accel_ref[i * 2 + 1][i]) / 2;
	}

	/* fill matrix A for linear equations system*/
	float mat_A[3][3];
	memset(mat_A, 0, sizeof(mat_A));

	for (unsigned i = 0; i < 3; i++) 
    {
		for (unsigned j = 0; j < 3; j++) 
        {
			float a = accel_ref[i * 2][j] - accel_offs[j];
			mat_A[i][j] = a;
		}
	}

	/* calculate inverse matrix for A */
	float mat_A_inv[3][3];

	if (mat_invert3(mat_A, mat_A_inv) != RT_EOK) 
    {
		return calibrate_return_error;
	}

	/* copy results to accel_T */
	for (unsigned i = 0; i < 3; i++) 
    {
		for (unsigned j = 0; j < 3; j++) 
        {
			/* simplify matrices mult because b has only one non-zero element == g at index i */
			accel_T[j][i] = mat_A_inv[j][i] * g;
		}
	}

	return calibrate_return_ok;
}

int Accel_Cal::sphere_fit_least_squares(const float x[], const float y[], const float z[],
			     unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z,
			     float *sphere_radius)
{

	float x_sumplain = 0.0f;
	float x_sumsq = 0.0f;
	float x_sumcube = 0.0f;

	float y_sumplain = 0.0f;
	float y_sumsq = 0.0f;
	float y_sumcube = 0.0f;

	float z_sumplain = 0.0f;
	float z_sumsq = 0.0f;
	float z_sumcube = 0.0f;

	float xy_sum = 0.0f;
	float xz_sum = 0.0f;
	float yz_sum = 0.0f;

	float x2y_sum = 0.0f;
	float x2z_sum = 0.0f;
	float y2x_sum = 0.0f;
	float y2z_sum = 0.0f;
	float z2x_sum = 0.0f;
	float z2y_sum = 0.0f;

	for (unsigned int i = 0; i < size; i++) {

		float x2 = x[i] * x[i];
		float y2 = y[i] * y[i];
		float z2 = z[i] * z[i];

		x_sumplain += x[i];
		x_sumsq += x2;
		x_sumcube += x2 * x[i];

		y_sumplain += y[i];
		y_sumsq += y2;
		y_sumcube += y2 * y[i];

		z_sumplain += z[i];
		z_sumsq += z2;
		z_sumcube += z2 * z[i];

		xy_sum += x[i] * y[i];
		xz_sum += x[i] * z[i];
		yz_sum += y[i] * z[i];

		x2y_sum += x2 * y[i];
		x2z_sum += x2 * z[i];

		y2x_sum += y2 * x[i];
		y2z_sum += y2 * z[i];

		z2x_sum += z2 * x[i];
		z2y_sum += z2 * y[i];
	}

	//
	//Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
	//
	//    P is a structure that has been computed with the data earlier.
	//    P.npoints is the number of elements; the length of X,Y,Z are identical.
	//    P's members are logically named.
	//
	//    X[n] is the x component of point n
	//    Y[n] is the y component of point n
	//    Z[n] is the z component of point n
	//
	//    A is the x coordiante of the sphere
	//    B is the y coordiante of the sphere
	//    C is the z coordiante of the sphere
	//    Rsq is the radius squared of the sphere.
	//
	//This method should converge; maybe 5-100 iterations or more.
	//
	float x_sum = x_sumplain / size;        //sum( X[n] )
	float x_sum2 = x_sumsq / size;    //sum( X[n]^2 )
	float x_sum3 = x_sumcube / size;    //sum( X[n]^3 )
	float y_sum = y_sumplain / size;        //sum( Y[n] )
	float y_sum2 = y_sumsq / size;    //sum( Y[n]^2 )
	float y_sum3 = y_sumcube / size;    //sum( Y[n]^3 )
	float z_sum = z_sumplain / size;        //sum( Z[n] )
	float z_sum2 = z_sumsq / size;    //sum( Z[n]^2 )
	float z_sum3 = z_sumcube / size;    //sum( Z[n]^3 )

	float XY = xy_sum / size;        //sum( X[n] * Y[n] )
	float XZ = xz_sum / size;        //sum( X[n] * Z[n] )
	float YZ = yz_sum / size;        //sum( Y[n] * Z[n] )
	float X2Y = x2y_sum / size;    //sum( X[n]^2 * Y[n] )
	float X2Z = x2z_sum / size;    //sum( X[n]^2 * Z[n] )
	float Y2X = y2x_sum / size;    //sum( Y[n]^2 * X[n] )
	float Y2Z = y2z_sum / size;    //sum( Y[n]^2 * Z[n] )
	float Z2X = z2x_sum / size;    //sum( Z[n]^2 * X[n] )
	float Z2Y = z2y_sum / size;    //sum( Z[n]^2 * Y[n] )

	//Reduction of multiplications
	float F0 = x_sum2 + y_sum2 + z_sum2;
	float F1 =  0.5f * F0;
	float F2 = -8.0f * (x_sum3 + Y2X + Z2X);
	float F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
	float F4 = -8.0f * (X2Z + Y2Z + z_sum3);

	//Set initial conditions:
	float A = x_sum;
	float B = y_sum;
	float C = z_sum;

	//First iteration computation:
	float A2 = A * A;
	float B2 = B * B;
	float C2 = C * C;
	float QS = A2 + B2 + C2;
	float QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);

	//Set initial conditions:
	float Rsq = F0 + QB + QS;

	//First iteration computation:
	float Q0 = 0.5f * (QS - Rsq);
	float Q1 = F1 + Q0;
	float Q2 = 8.0f * (QS - Rsq + QB + F0);
	float aA, aB, aC, nA, nB, nC, dA, dB, dC;

	//Iterate N times, ignore stop condition.
	unsigned int n = 0;

	while (n < max_iterations) {
		n++;

		//Compute denominator:
		aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
		aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
		aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
		aA = (fabsf(aA) < FLT_EPSILON) ? 1.0f : aA;
		aB = (fabsf(aB) < FLT_EPSILON) ? 1.0f : aB;
		aC = (fabsf(aC) < FLT_EPSILON) ? 1.0f : aC;

		//Compute next iteration
		nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
		nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
		nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);

		//Check for stop condition
		dA = (nA - A);
		dB = (nB - B);
		dC = (nC - C);

		if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }

		//Compute next iteration's values
		A = nA;
		B = nB;
		C = nC;
		A2 = A * A;
		B2 = B * B;
		C2 = C * C;
		QS = A2 + B2 + C2;
		QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
		Rsq = F0 + QB + QS;
		Q0 = 0.5f * (QS - Rsq);
		Q1 = F1 + Q0;
		Q2 = 8.0f * (QS - Rsq + QB + F0);
	}

	*sphere_x = A;
	*sphere_y = B;
	*sphere_z = C;
	*sphere_radius = sqrtf(Rsq);

	return 0;
}

// 探测传感器方向
enum detect_orientation_return Accel_Cal::detect_orientation(bool lenient_still_position)
{
	const uint8_t ndim = 3;
	
	float		    accel_ema[ndim] = { 0.0f };		       // exponential moving average of accel
	float		    accel_disp[3] = { 0.0f, 0.0f, 0.0f };  // max-hold dispersion of accel
	float		    ema_len = 0.5f;				           // EMA time constant in seconds
	const float	    normal_still_thr = 0.3;		       // normal still threshold
	float		    still_thr2 = powf(lenient_still_position ? (normal_still_thr * 3) : normal_still_thr, 2);
	float		    accel_err_thr = 5.0f;			       // set accel error threshold to 5m/s^2
	uint32_t    	still_time = lenient_still_position ? 100 : 150;	// still time required in us
	
    uint32_t t_start = rt_tick_get();     // 获取当前的OS Tick
	/* set timeout to 30s */
	uint32_t timeout = 3000;      // 3000个OS Tick 3000*10000us
	uint32_t t_timeout = t_start + timeout;
    
	uint32_t t = SysTick->VAL; // 获取定时器当前计数值;
    uint32_t t_os = t_start;
	uint32_t t_still = 0;
	
	uint32_t poll_errcount = 0;
    
    float accelerometer_m_s2[3];  // 三个加速计的当前加速度
	rt_err_t poll_ret = RT_ERROR;
    
	while (true) 
    {
		/* wait blocking for new data */
		// 阻塞等待数据到来
        //copter.thread_backend[0]
//            accelerometer_m_s2[0] = mpu6500.Acc_ms2.x;
//            accelerometer_m_s2[1] = mpu6500.Acc_ms2.y;
//            accelerometer_m_s2[2] = mpu6500.Acc_ms2.z;
            poll_ret = RT_EOK;

		if (poll_ret == RT_EOK)
        {
			t = SysTick->VAL;
            t_os = rt_tick_get();
			//float dt = (t - t_prev) / 1000000.0f;
			//t_prev = t;
            float dt = 0.006f;
			float w = dt / ema_len;
			
			for (uint8_t i = 0; i < ndim; i++)
            {
				float di = 0.0f;
				switch (i) 
                {
					case 0:
						di = accelerometer_m_s2[0];
						break;
					case 1:
						di = accelerometer_m_s2[1];
						break;
					case 2:
						di = accelerometer_m_s2[2];
						break;
				}
				
				float d = di - accel_ema[i];
				accel_ema[i] += d * w;
				d = d * d;
				accel_disp[i] = accel_disp[i] * (1.0f - w);
				
				if (d > still_thr2 * 8.0f) 
                {
					d = still_thr2 * 8.0f;
				}
				
				if (d > accel_disp[i]) 
                {
					accel_disp[i] = d;
				}
			}
			/* still detector with hysteresis */
			if (accel_disp[0] < still_thr2 &&
			    accel_disp[1] < still_thr2 &&
			    accel_disp[2] < still_thr2)
            {
				/* is still now */
				if (t_still == 0) {
					/* first time */
					rt_kprintf("[cal] detected rest position, hold still...\n");
					t_still = t_os;
					t_timeout = t_os + timeout;
					
				} else 
                {
					/* still since t_still */
					if (t_os > t_still + still_time) 
                    {
						/* vehicle is still, exit from the loop to detection of its orientation */
						break;
					}
				}
			} 
            else if (accel_disp[0] > still_thr2 * 4.0f ||
				   accel_disp[1] > still_thr2 * 4.0f ||
				   accel_disp[2] > still_thr2 * 4.0f) 
            {
				/* not still, reset still start time */
				if (t_still != 0) 
                {
					rt_kprintf("[cal] detected motion, please hold still...\n");
					rt_thread_delay(50); // delay 0.5s
					t_still = 0;
				}
			}
		} else if (poll_ret == RT_ERROR) 
        {
			poll_errcount++;
		}
		// 超时时间判断
		if (t_os > t_timeout) 
        {
			poll_errcount++;
		}
		// 多于1000个错误
		if (poll_errcount > 1000)
        {
			rt_kprintf(CAL_ERROR_SENSOR_MSG);
			return DETECT_ORIENTATION_ERROR;
		}
	}

	if (fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_TAIL_DOWN;        // [ g, 0, 0 ]
	}
	
	if (fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_NOSE_DOWN;        // [ -g, 0, 0 ]
	}
	
	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_LEFT;        // [ 0, g, 0 ]
	}
	
	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_RIGHT;        // [ 0, -g, 0 ]
	}
	
	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr) {
		return DETECT_ORIENTATION_UPSIDE_DOWN;        // [ 0, 0, g ]
	}
	
	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr) {
		return DETECT_ORIENTATION_RIGHTSIDE_UP;        // [ 0, 0, -g ]
	}
	
	rt_kprintf("[cal] ERROR: invalid orientation\n");
	
	return DETECT_ORIENTATION_ERROR;	// Can't detect orientation
}

const char* Accel_Cal::detect_orientation_str(enum detect_orientation_return orientation)
{
	static const char* rgOrientationStrs[] = {
		"back",		// tail down
		"front",	// nose down
		"left",
		"right",
		"up",		// upside-down
		"down",		// right-side up
		"error"
	};
	return rgOrientationStrs[orientation];
}

// 读取6个面的校准数据 lenient_still_position = false
calibrate_return Accel_Cal::calibrate_from_orientation( bool	side_data_collected[detect_orientation_side_count],
                                             void*	worker_data,
                                             bool	lenient_still_position)
{
	calibrate_return result = calibrate_return_ok;
	
	// Setup subscriptions to onboard accel sensor
	// 确认是否有加速计
	
	// 校准失败的面计数
	unsigned orientation_failures = 0;
	
	// Rotate through all requested orientation
	while (true) 
    {
        // 校准超过4个面失败
		if (orientation_failures > 4) 
        {
			result = calibrate_return_error;
			rt_kprintf(CAL_QGC_FAILED_MSG, "timeout: no motion");
			break;
		}
		// 已经完成的面校准
		unsigned int side_complete_count = 0;
		
		// 更新已经完成的面
		for (unsigned i = 0; i < detect_orientation_side_count; i++) 
        {
			if (side_data_collected[i]) {
				side_complete_count++;
			}
		}
		
		if (side_complete_count == detect_orientation_side_count) 
        {
			// 已经完成了所有面的校准
			break;
		}
		
		/* 告知所需要校准的面 */
		char pendingStr[256];
		pendingStr[0] = 0;
		
		for (unsigned int cur_orientation=0; cur_orientation<detect_orientation_side_count; cur_orientation++) 
        {
            // 告知还未校准的面
			if (!side_data_collected[cur_orientation]) 
            {
				strcat(pendingStr, " ");
				strcat(pendingStr, detect_orientation_str((enum detect_orientation_return)cur_orientation));
			}
		}
		rt_kprintf("[cal] pending:*%s*\n", pendingStr);
		
		rt_kprintf("[cal] hold vehicle still on a pending side\n");
        // 探测传感器方向并采集数据
		enum detect_orientation_return orient = detect_orientation(lenient_still_position);
		
		if (orient == DETECT_ORIENTATION_ERROR) 
        {
			orientation_failures++;
			rt_kprintf("[cal] detected motion, hold still...\n");
			continue;
		}
		
		/* 告知用户此面已经采集 */
		if (side_data_collected[orient])
        {
			orientation_failures++;
			rt_kprintf("[cal] %s side completed or not needed\n", detect_orientation_str(orient));
			rt_kprintf("[cal] rotate to a pending side\n");
			continue;
		}
		
		rt_kprintf(CAL_QGC_ORIENTATION_DETECTED_MSG, detect_orientation_str(orient));
		orientation_failures = 0;
		
		// 采集校准数据
		result = accel_calibration_worker(orient, worker_data);
		if (result != calibrate_return_ok )
        {
			break;
		}
		
		rt_kprintf(CAL_QGC_SIDE_DONE_MSG, detect_orientation_str(orient));
		
		// Note that this side is complete
		side_data_collected[orient] = true;
		rt_thread_delay(500); // 延时5s
	}
	return result;
}

