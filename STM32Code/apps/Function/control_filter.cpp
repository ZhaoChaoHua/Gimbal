#include "control_filter.h"
#include "stm32f40x_define.h"
#include "definitions.h"
#include <AP_Math.h>
#include "can_thread.h"

#define CTRL_LOOP_TIMEMS                 1
#define F_CUT 20.0f
#define STANDER_PID                      1

PID_jscop_t pid_pos_scop;
PID_jscop_t pid_gyro_scop;


Control_Filter::Control_Filter(void)
{
    this->rc = 1.0f / ((float) M_TWOPI * F_CUT );
    this->holdIntegrators = false;
//    this->dt = 1.0f/1000.0f;
}

void Control_Filter::pid_control_v2(void)
{
    float sinp, cosp;
    static uint32_t pid_now, pid_prev;
    uint32_t count;
    
//    float sinr, cosr, siny, cosy;
    sinp = sinf(Ctrlstruct[0].angle_err * DEG_TO_RAD);
    cosp = cosf(Ctrlstruct[0].angle_err * DEG_TO_RAD);
//    sinr = sinf(Ctrlstruct[1].angle_err * DEG_TO_RAD);
//    cosr = cosf(Ctrlstruct[1].angle_err * DEG_TO_RAD);
//    siny = sinf(Ctrlstruct[2].angle_err * DEG_TO_RAD);
//    cosy = cosf(Ctrlstruct[2].angle_err * DEG_TO_RAD);
    
    pid_now = SysTick->VAL;
    count = (pid_now > pid_prev)?(SysTick->LOAD + pid_prev - pid_now) : (pid_prev - pid_now);
    this->dt = count / US_T;
    this->dt = this->dt / 1000000.0f;
    pid_prev = pid_now;
    if(this->dt < 0.0001f)
    {
        this->dt = 0.00025f;
    }
    
    angle_error[0] = -Ctrlstruct[0].angle_err;
    angle_error[1] = -Ctrlstruct[1].angle_err * cosp - Ctrlstruct[2].angle_err * sinp;
    angle_error[2] = -Ctrlstruct[2].angle_err * cosp - Ctrlstruct[1].angle_err * sinp;
    
    if(camCtrl[2].mode_id & INPUT_MODE_FOLLOW)
    {
        Ctrlstruct[0].pos_axis = ahrsData.euler.x;
        Ctrlstruct[1].pos_axis = ahrsData.euler.z * sinp + ahrsData.euler.y * cosp;
        Ctrlstruct[2].pos_axis = -ahrsData.euler.y * sinp + ahrsData.euler.z * cosp;
    }
    else
    {
        Ctrlstruct[0].pos_axis = ahrsData.euler.x;
        Ctrlstruct[1].pos_axis = ahrsData.euler.z * sinp + ahrsData.euler.y * cosp;
        Ctrlstruct[2].pos_axis = -ahrsData.euler.y * sinp + ahrsData.euler.z * cosp;
    }
    
    Ctrlstruct[0].gyro_axis = mpu6500.Gyro_rad.x * RAD_TO_DEG;
    Ctrlstruct[1].gyro_axis = (mpu6500.Gyro_rad.z * sinp + mpu6500.Gyro_rad.y * cosp) * RAD_TO_DEG;
    Ctrlstruct[2].gyro_axis = (-mpu6500.Gyro_rad.y * sinp + mpu6500.Gyro_rad.z * cosp) * RAD_TO_DEG;

    RotationUpdate(0);
    camAtti[0] += camRot[0];
    camAtti[0] = circadjust(camAtti[0], 180.0f);
    
    RotationUpdate(1);
    camAtti[1] += camRot[1];
    camAtti[1] = circadjust(camAtti[1], 180.0f);
    
    RotationUpdate(2);
    camAtti[2] += camRot[2];
    camAtti[2] = circadjust(camAtti[2], 180.0f);
    
    pos_exp[0] = camAtti[0];
    pos_exp[1] = camAtti[1] * cosp + camAtti[2] * sinp;
    pos_exp[2] = camAtti[2] * cosp + camAtti[1] * sinp;
//    pos_exp[1] = camAtti[1];
//    pos_exp[2] = camAtti[2];
    
    pid_position();
    pid_gyro();
    eliminate_calc();
}

// camera位置控制
void Control_Filter::RotationUpdate(uint8_t i)
{
    float coef;
    float speedLimit;
    
    speedLimit = (float)camCtrl[i].speed;

    if(camCtrl[i].mode_id & INPUT_MODE_FOLLOW)
    {
        coef = angle_error[i];
//        if (coef > camCtrl[i].fllow_deadband)
//        {
//            coef -= camCtrl[i].fllow_deadband;
//            coef /= INPUT_SIGNAL_ALPHA * FIXED_DT_STEP;
//        } else if (coef < -camCtrl[i].fllow_deadband)
//        {
//            coef += camCtrl[i].fllow_deadband;
//            coef /= INPUT_SIGNAL_ALPHA * FIXED_DT_STEP;
//        } else
//        {
//            coef = 0.0f;
//        }
          coef /= INPUT_SIGNAL_ALPHA * FIXED_DT_STEP;

        coef *= Ctrlstruct[i].dirMotor;
        
    // 禁止输入控制
    }
    else   // 手动控制模式
    {
        coef = (float)ctrlInput[i].input_val / 255.0f;

        if(camCtrl[i].mode_id & INPUT_MODE_SPEED)
        {
            coef *= 2.0f*speedLimit;
            camRotSpeedPrev[i] += (coef - camRotSpeedPrev[i]) / INPUT_SIGNAL_ALPHA;
            coef = camRotSpeedPrev[i];
        } 
        else
        {
            coef *= (camCtrl[i].max_angle - camCtrl[i].min_angle);
            coef += (camCtrl[i].max_angle + camCtrl[i].min_angle) / 2;
            coef = constrain(coef, (float)camCtrl[i].min_angle, (float)camCtrl[i].max_angle);

            coef = (coef - camAtti[i]) / INPUT_SIGNAL_ALPHA / FIXED_DT_STEP;
        }
    }

    coef = constrain(coef, -speedLimit, speedLimit);
    camRot[i] = coef * FIXED_DT_STEP;
}

/* -------------------------------------串级PID控制----------------------------------------- */
void Control_Filter::eliminate_calc(void)
{
    thr_pos[0] = abs(pidctrl_pos[0].error) * 0.3f - 0.1f;  
    pidctrl_pos[0].Thr_Weight  = (1.0f - LIMIT(thr_pos[0], 0, 1));
    pidctrl_gyro[0].Thr_Weight = (1.0f - LIMIT(thr_pos[0], 0, 1));
    pid_pos_scop.Thr_Weight0 = pidctrl_pos[0].Thr_Weight * 100;
    pid_gyro_scop.Thr_Weight0 = pidctrl_gyro[0].Thr_Weight * 100;
    
    thr_pos[1] = abs(pidctrl_pos[1].error) * 0.3f - 0.1f;
    pidctrl_pos[1].Thr_Weight  = (1.0f - LIMIT(thr_pos[1], 0, 1));
    pidctrl_gyro[1].Thr_Weight = (1.0f - LIMIT(thr_pos[1], 0, 1));
    pid_pos_scop.Thr_Weight1 = pidctrl_pos[1].Thr_Weight * 100;
    pid_gyro_scop.Thr_Weight1 = pidctrl_gyro[1].Thr_Weight * 100;
    
    thr_pos[2] = abs(pidctrl_pos[2].error) * 0.3f - 0.1f;
    pidctrl_pos[2].Thr_Weight  = (1.0f - LIMIT(thr_pos[2], 0, 1));
    pidctrl_gyro[2].Thr_Weight = (1.0f - LIMIT(thr_pos[2], 0, 1));
    pid_pos_scop.Thr_Weight2 = pidctrl_pos[2].Thr_Weight * 100;
    pid_gyro_scop.Thr_Weight2 = pidctrl_gyro[2].Thr_Weight * 100;
}


void Control_Filter::pid_gyro(void)
{
    float EXP_LPF_TMP[3];
    
	EXP_LPF_TMP[0] = pidctrl_gyro[0].max_control *(pidctrl_pos[0].OutPut/ANGLE_TO_MAX_AS);
	EXP_LPF_TMP[1] = pidctrl_gyro[1].max_control *(pidctrl_pos[1].OutPut/ANGLE_TO_MAX_AS);
    EXP_LPF_TMP[2] = pidctrl_gyro[2].max_control *(pidctrl_pos[2].OutPut/ANGLE_TO_MAX_AS);
    
    EXP_LPF_TMP[0] = LIMIT(EXP_LPF_TMP[0], -pidctrl_gyro[0].exp_guard, pidctrl_gyro[0].exp_guard);
    EXP_LPF_TMP[1] = LIMIT(EXP_LPF_TMP[1], -pidctrl_gyro[1].exp_guard, pidctrl_gyro[1].exp_guard);
    EXP_LPF_TMP[2] = LIMIT(EXP_LPF_TMP[2], -pidctrl_gyro[2].exp_guard, pidctrl_gyro[2].exp_guard);

	pidctrl_gyro[0].error = EXP_LPF_TMP[0] - Ctrlstruct[0].gyro_axis;
    pidctrl_gyro[1].error = -EXP_LPF_TMP[1] - Ctrlstruct[1].gyro_axis;
    pidctrl_gyro[2].error = -EXP_LPF_TMP[2] - Ctrlstruct[2].gyro_axis;

	pidctrl_gyro[0].err_weight = fabsf(pidctrl_gyro[0].error)/pidctrl_gyro[0].max_control;
    pidctrl_gyro[1].err_weight = fabsf(pidctrl_gyro[1].error)/pidctrl_gyro[1].max_control;
    pidctrl_gyro[2].err_weight = fabsf(pidctrl_gyro[2].error)/pidctrl_gyro[2].max_control;

	pidctrl_gyro[0].dTerm = (pidctrl_gyro[0].D * 0.01f *(pidctrl_gyro[0].error - pidctrl_gyro[0].err_old) / dt);
    pidctrl_gyro[1].dTerm = (pidctrl_gyro[1].D * 0.01f *(pidctrl_gyro[1].error - pidctrl_gyro[1].err_old) / dt);
    pidctrl_gyro[2].dTerm = (pidctrl_gyro[2].D * 0.01f *(pidctrl_gyro[2].error - pidctrl_gyro[2].err_old) / dt);

	pidctrl_gyro[0].iTerm += pidctrl_gyro[0].I *(pidctrl_gyro[0].error - pidctrl_gyro[0].err_old) * dt;
    pidctrl_gyro[1].iTerm += pidctrl_gyro[1].I *(pidctrl_gyro[1].error - pidctrl_gyro[1].err_old) * dt;
    pidctrl_gyro[2].iTerm += pidctrl_gyro[2].I *(pidctrl_gyro[2].error - pidctrl_gyro[2].err_old) * dt;

    pidctrl_gyro[0].eliminate_I = pidctrl_gyro[0].Thr_Weight * pidctrl_gyro[0].max_control * 0.5f;
    pidctrl_gyro[1].eliminate_I = pidctrl_gyro[1].Thr_Weight * pidctrl_gyro[1].max_control * 0.5f;
    pidctrl_gyro[2].eliminate_I = pidctrl_gyro[2].Thr_Weight * pidctrl_gyro[2].max_control * 0.5f;
//    pidctrl_gyro[0].eliminate_I = pidctrl_gyro[0].max_control * 0.5f;
//    pidctrl_gyro[1].eliminate_I = pidctrl_gyro[1].max_control * 0.5f;
//    pidctrl_gyro[2].eliminate_I = pidctrl_gyro[2].max_control * 0.5f;

	pidctrl_gyro[0].iTerm = LIMIT( pidctrl_gyro[0].iTerm, -pidctrl_gyro[0].eliminate_I, pidctrl_gyro[0].eliminate_I);
    pidctrl_gyro[1].iTerm = LIMIT( pidctrl_gyro[1].iTerm, -pidctrl_gyro[1].eliminate_I, pidctrl_gyro[1].eliminate_I);
    pidctrl_gyro[2].iTerm = LIMIT( pidctrl_gyro[2].iTerm, -pidctrl_gyro[2].eliminate_I, pidctrl_gyro[2].eliminate_I);

	pidctrl_gyro[0].OutPut = pidctrl_gyro[0].B *LIMIT((0.45f + 0.55f*pidctrl_gyro[0].err_weight),0,1)* EXP_LPF_TMP[0] + ( 1.0f - pidctrl_gyro[0].B ) * pidctrl_gyro[0].P * (pidctrl_gyro[0].error + pidctrl_gyro[0].dTerm + pidctrl_gyro[0].iTerm);
//  pidctrl_gyro[1].OutPut = pidctrl_gyro[1].B *LIMIT((0.45f + 0.55f*pidctrl_gyro[1].err_weight),0,1)* EXP_LPF_TMP[1] + ( 1.0f - pidctrl_gyro[1].B ) * pidctrl_gyro[1].P * (pidctrl_gyro[1].error + pidctrl_gyro[1].dTerm + pidctrl_gyro[1].iTerm);
//  pidctrl_gyro[2].OutPut = pidctrl_gyro[2].B *LIMIT((0.45f + 0.55f*pidctrl_gyro[2].err_weight),0,1)* EXP_LPF_TMP[2] + ( 1.0f - pidctrl_gyro[2].B ) * pidctrl_gyro[2].P * (pidctrl_gyro[2].error + pidctrl_gyro[2].dTerm + pidctrl_gyro[2].iTerm);

//    pidctrl_gyro[0].OutPut = pidctrl_gyro[0].P * pidctrl_gyro[0].error + pidctrl_gyro[0].dTerm + pidctrl_gyro[0].iTerm;
    pidctrl_gyro[1].OutPut = pidctrl_gyro[1].P * pidctrl_gyro[1].error + pidctrl_gyro[1].dTerm + pidctrl_gyro[1].iTerm;
    pidctrl_gyro[2].OutPut = pidctrl_gyro[2].P * pidctrl_gyro[2].error + pidctrl_gyro[2].dTerm + pidctrl_gyro[2].iTerm;
    
    utils_truncate_number(&pidctrl_gyro[0].OutPut, -pidctrl_gyro[0].out_guard, pidctrl_gyro[0].out_guard);
    utils_truncate_number(&pidctrl_gyro[1].OutPut, -pidctrl_gyro[1].out_guard, pidctrl_gyro[1].out_guard);
    utils_truncate_number(&pidctrl_gyro[2].OutPut, -pidctrl_gyro[2].out_guard, pidctrl_gyro[2].out_guard);

    pidctrl_gyro[0].OutPut = Ctrlstruct[0].dirMotor * pidctrl_gyro[0].OutPut;
    pidctrl_gyro[1].OutPut = Ctrlstruct[1].dirMotor * pidctrl_gyro[1].OutPut;
    pidctrl_gyro[2].OutPut = Ctrlstruct[2].dirMotor * pidctrl_gyro[2].OutPut;

	pidctrl_gyro[0].err_old = pidctrl_gyro[0].error;
    pidctrl_gyro[1].err_old = pidctrl_gyro[1].error;
    pidctrl_gyro[2].err_old = pidctrl_gyro[2].error;

	g_old[0] =  mpu6500.Gyro_rad.x;
    g_old[1] =  mpu6500.Gyro_rad.y;
    g_old[2] =  mpu6500.Gyro_rad.z;

    /* --------------------------J-Scop-------------------------- */
    pid_gyro_scop.exp_data0 = EXP_LPF_TMP[0];
    pid_gyro_scop.exp_data1 = EXP_LPF_TMP[1];
    pid_gyro_scop.exp_data2 = EXP_LPF_TMP[2];
    pid_gyro_scop.damp0 = pidctrl_gyro[0].damp;
    pid_gyro_scop.damp1 = pidctrl_gyro[1].damp;
    pid_gyro_scop.damp2 = pidctrl_gyro[2].damp;
    pid_gyro_scop.error0 = pidctrl_gyro[0].error;
    pid_gyro_scop.error1 = pidctrl_gyro[1].error;
    pid_gyro_scop.error2 = pidctrl_gyro[2].error;
    pid_gyro_scop.err_weight0 = pidctrl_gyro[0].err_weight * 100;
    pid_gyro_scop.err_weight1 = pidctrl_gyro[1].err_weight * 100;
    pid_gyro_scop.err_weight2 = pidctrl_gyro[2].err_weight * 100;
    pid_gyro_scop.dTerm0 = pidctrl_gyro[0].dTerm * 100;
    pid_gyro_scop.dTerm1 = pidctrl_gyro[1].dTerm * 100;
    pid_gyro_scop.dTerm2 = pidctrl_gyro[2].dTerm * 100;
    pid_gyro_scop.eliminate_I0 = pidctrl_gyro[0].eliminate_I * 100;
    pid_gyro_scop.eliminate_I1 = pidctrl_gyro[1].eliminate_I * 100;
    pid_gyro_scop.eliminate_I2 = pidctrl_gyro[2].eliminate_I * 100;
    pid_gyro_scop.iTerm0 = pidctrl_gyro[0].iTerm * 100;
    pid_gyro_scop.iTerm1 = pidctrl_gyro[1].iTerm * 100;
    pid_gyro_scop.iTerm2 = pidctrl_gyro[2].iTerm * 100;
    pid_gyro_scop.OutPut0 = pidctrl_gyro[0].OutPut;
    pid_gyro_scop.OutPut1 = pidctrl_gyro[1].OutPut;
    pid_gyro_scop.OutPut2 = pidctrl_gyro[2].OutPut;
}

void Control_Filter::pid_position(void)
{
    float except_A[3];

	except_A[0] = pos_exp[0];
    except_A[1] = pos_exp[1];
    except_A[2] = pos_exp[2];

	pidctrl_pos[0].error =  except_A[0] - Ctrlstruct[0].pos_axis;
    pidctrl_pos[1].error =  except_A[1] - Ctrlstruct[1].pos_axis;
    pidctrl_pos[2].error =  except_A[2] - Ctrlstruct[2].pos_axis;

	pidctrl_pos[0].err_weight = fabsf(pidctrl_pos[0].error)/ANGLE_TO_MAX_AS;
    pidctrl_pos[1].err_weight = fabsf(pidctrl_pos[1].error)/ANGLE_TO_MAX_AS;
    pidctrl_pos[2].err_weight = fabsf(pidctrl_pos[2].error)/ANGLE_TO_MAX_AS;
    
	pidctrl_pos[0].dTerm = 10.0f * pidctrl_pos[0].D *(pidctrl_pos[0].error - pidctrl_pos[0].err_old) * (0.001f/dt) * ( 0.65f + 0.35f * pidctrl_pos[0].err_weight);
    pidctrl_pos[1].dTerm = 10.0f * pidctrl_pos[1].D *(pidctrl_pos[1].error - pidctrl_pos[1].err_old) * (0.001f/dt) * ( 0.65f + 0.35f * pidctrl_pos[1].err_weight);
    pidctrl_pos[2].dTerm = 10.0f * pidctrl_pos[2].D *(pidctrl_pos[2].error - pidctrl_pos[2].err_old) * (0.001f/dt) * ( 0.65f + 0.35f * pidctrl_pos[2].err_weight);
    
	pidctrl_pos[0].iTerm += pidctrl_pos[0].I * pidctrl_pos[0].error * dt;
    pidctrl_pos[1].iTerm += pidctrl_pos[1].I * pidctrl_pos[1].error * dt;
    pidctrl_pos[2].iTerm += pidctrl_pos[2].I * pidctrl_pos[2].error * dt;
    
	pidctrl_pos[0].eliminate_I = pidctrl_pos[0].Thr_Weight * pidctrl_pos[0].max_control;
    pidctrl_pos[1].eliminate_I = pidctrl_pos[1].Thr_Weight * pidctrl_pos[1].max_control;
    pidctrl_pos[2].eliminate_I = pidctrl_pos[2].Thr_Weight * pidctrl_pos[2].max_control;
//	pidctrl_pos[0].eliminate_I = pidctrl_pos[0].max_control;
//    pidctrl_pos[1].eliminate_I = pidctrl_pos[1].max_control;
//    pidctrl_pos[2].eliminate_I = pidctrl_pos[2].max_control;
    
	pidctrl_pos[0].iTerm = LIMIT(pidctrl_pos[0].iTerm, -pidctrl_pos[0].eliminate_I, pidctrl_pos[0].eliminate_I);
    pidctrl_pos[1].iTerm = LIMIT(pidctrl_pos[1].iTerm, -pidctrl_pos[1].eliminate_I, pidctrl_pos[1].eliminate_I);
    pidctrl_pos[2].iTerm = LIMIT(pidctrl_pos[2].iTerm, -pidctrl_pos[2].eliminate_I, pidctrl_pos[2].eliminate_I);
    
	pidctrl_pos[0].error = LIMIT(pidctrl_pos[0].error, -90, 90);
    pidctrl_pos[1].error = LIMIT(pidctrl_pos[1].error, -90, 90);
    pidctrl_pos[2].error = LIMIT(pidctrl_pos[2].error, -90, 90);

	pidctrl_pos[0].OutPut = 10.0f * pidctrl_pos[0].P * (pidctrl_pos[0].error + pidctrl_pos[0].dTerm + pidctrl_pos[0].iTerm);
    pidctrl_pos[1].OutPut = 10.0f * pidctrl_pos[1].P * (pidctrl_pos[1].error + pidctrl_pos[1].dTerm + pidctrl_pos[1].iTerm);
    pidctrl_pos[2].OutPut = 10.0f * pidctrl_pos[2].P * (pidctrl_pos[2].error + pidctrl_pos[2].dTerm + pidctrl_pos[2].iTerm);
    	
	pidctrl_pos[0].err_old = pidctrl_pos[0].error;
    pidctrl_pos[1].err_old = pidctrl_pos[1].error;
    pidctrl_pos[2].err_old = pidctrl_pos[2].error;
    
    /* --------------------------J-Scop-------------------------- */
    pid_pos_scop.err_weight0 = pidctrl_pos[0].err_weight * 100;
    pid_pos_scop.err_weight1 = pidctrl_pos[1].err_weight * 100;
    pid_pos_scop.err_weight2 = pidctrl_pos[2].err_weight * 100;
        
    pid_pos_scop.dTerm0 = pidctrl_pos[0].dTerm * 100;
    pid_pos_scop.dTerm1 = pidctrl_pos[1].dTerm * 100;
    pid_pos_scop.dTerm2 = pidctrl_pos[2].dTerm * 100;
        
    pid_pos_scop.eliminate_I0 = pidctrl_pos[0].eliminate_I * 100;
    pid_pos_scop.eliminate_I1 = pidctrl_pos[1].eliminate_I * 100;
    pid_pos_scop.eliminate_I2 = pidctrl_pos[2].eliminate_I * 100;
        
    pid_pos_scop.iTerm0 = pidctrl_pos[0].iTerm * 100;
    pid_pos_scop.iTerm1 = pidctrl_pos[1].iTerm * 100;
    pid_pos_scop.iTerm2 = pidctrl_pos[2].iTerm * 100;
        
    pid_pos_scop.error0 = pidctrl_pos[0].error * 100;
    pid_pos_scop.error1 = pidctrl_pos[1].error * 100;
    pid_pos_scop.error2 = pidctrl_pos[2].error * 100;
        
    pid_pos_scop.OutPut0 = pidctrl_pos[0].OutPut * 100;
    pid_pos_scop.OutPut1 = pidctrl_pos[1].OutPut * 100;
    pid_pos_scop.OutPut2 = pidctrl_pos[2].OutPut * 100;
}

float Control_Filter::utils_angle_difference(float angle1, float angle2)
{
	// Faster in most cases
	float difference = angle1 - angle2;
	while (difference < -180.0f) difference += 2.0f * 180.0f;
	while (difference > 180.0f) difference -= 2.0f * 180.0f;
	return difference;
}

int Control_Filter::utils_truncate_number(float *number, float min, float max) 
{
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

float Control_Filter::standardRadianFormat(float angle)
{
    if (angle >= PI)
        return (angle - 2.0f * PI);
    else if (angle < -PI)
        return (angle + 2.0f * PI);
    else
        return (angle);
}

#ifdef RT_USING_FINSH
#include <finsh.h>
void set_mode(int mode)
{
    camCtrl[2].mode_id = INPUT_MODE_FOLLOW;
}
FINSH_FUNCTION_EXPORT(set_mode, Get )

#endif
