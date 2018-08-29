#ifndef __CONTROL_FILTER_H__
#define __CONTROL_FILTER_H__

#ifdef __cplusplus

#include "time_measure.h"


//
#define FIXED_DT_STEP             0.001f

#define MOTOR_STEP_LIMIT_MAX      M_PI / 45.0f
#define MOTOR_STEP_LIMIT_MIN      -MOTOR_STEP_LIMIT_MAX

#define ACCEL_TAU                 0.1f
#define INPUT_SIGNAL_ALPHA        300.0f
#define MODE_FOLLOW_DEAD_BAND     M_PI / 36.0f

/* Input modes: */
#define INPUT_MODE_ANGLE          0x00
#define INPUT_MODE_SPEED          0x01
#define INPUT_MODE_FOLLOW         0x02

/* 串级PID控制 */
#define ANGLE_TO_MAX_AS 		  30.0f						// 角度误差N时，期望角速度达到最大（可以通过调整CTRL_2的P值调整）

// 继承Time_point类，用于定时
class Control_Filter
{
private:
    float camAtti[3];
    float camRot[3];
    float camRotSpeedPrev[3];
    float rc;
    uint8_t holdIntegrators;
    float dt;
    float g_old[3];
    float thr_pos[3];
    float angle_error[3];
    float pos_exp[3];

    void RotationUpdate(uint8_t i);
    void pid_gyro(void);
    void pid_position(void);

    float standardRadianFormat(float angle);
    int utils_truncate_number(float *number, float min, float max);
    float utils_angle_difference(float angle1, float angle2);

public:
    Control_Filter(void);
    void eliminate_calc(void);
    void pid_control_v2(void);
};

#endif

#endif
