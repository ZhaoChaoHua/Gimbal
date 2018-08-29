#include "AccelCal.h"
#include <stdlib.h>

#define ACCELCAL_POSITION_REQUEST_INTERVAL_MS 5000

static bool _start_collect_sample;


int scop_cal_count, scop_cal_dt;
void AccelCal::update()
{
    if (_started) {
			  cal_done = false;
        update_status();

        if(_start_collect_sample) {
            collect_sample();
        }
        switch(_status) {
            case ACCEL_CAL_NOT_STARTED:
                fail();
                return;
            case ACCEL_CAL_WAITING_FOR_ORIENTATION: {
                // if we're waiting for orientation, first ensure that all calibrators are on the same step
                uint8_t step;

                step = _clients->get_num_samples_collected()+1;

                // if we're on a new step, print a message describing the step
                if (step != _step) {
                    _step = step;

                    const char *msg;
                    switch (step) {
                        case ACCELCAL_VEHICLE_POS_LEVEL:
                            msg = "LEVEL";
                            break;
                        case ACCELCAL_VEHICLE_POS_LEFT:
                            msg = "LEFT";
                            break;
                        case ACCELCAL_VEHICLE_POS_RIGHT:
                            msg = "RIGHT";
                            break;
                        case ACCELCAL_VEHICLE_POS_NOSEDOWN:
                            msg = "DOWN";
                            break;
                        case ACCELCAL_VEHICLE_POS_NOSEUP:
                            msg = "UP";
                            break;
                        case ACCELCAL_VEHICLE_POS_BACK:
                            msg = "BACK";
                            break;
                        default:
                            fail();
                            return;
                    }
                    rt_kprintf("[INFO]Place vehicle %s, Waiting 5 second\n", msg);
                }
								
								rt_thread_delay(500);
								_start_collect_sample = true;

                break;
            }
            case ACCEL_CAL_COLLECTING_SAMPLE:
//                // check for timeout
//                _clients->check_for_timeout();

                update_status();

                if (_status == ACCEL_CAL_FAILED) {
                    fail();
                }
                return;
            case ACCEL_CAL_SUCCESS:
                // save
                if (_saving) {
                    success();
                    return;
                } else {
                    _acal_save_calibrations();
                    _saving = true;
									  cal_done = true;
                }
                return;
            default:
            case ACCEL_CAL_FAILED:
                fail();
                return;
        }
    } else if (_last_result != ACCEL_CAL_NOT_STARTED) {
        // only continuously report if we have ever completed a calibration

			  rt_thread_delay(500);
			

            switch (_last_result) {
//                case ACCEL_CAL_SUCCESS:
//                    rt_kprintf("[INFO]Calibration Success,Please reboot\n");
//                    break;
//                case ACCEL_CAL_FAILED:
//                    rt_kprintf("[ERROR]Calibration Error,Please reboot\n");
//                    break;
                default:
//                    // should never hit this state
                    break;
            }
//        }
    }
}

void AccelCal::start(void)
{
    if (_started) {
        return;
    }
    _start_collect_sample = false;
    
    if(_clients == NULL)
        return;
		
		float ox,oy,oz, gx, gy, gz;
		
		AP_Param *off_x;
    AP_Param *off_y;
    AP_Param *off_z;
    
    AP_Param *diag_x;
    AP_Param *diag_y;
    AP_Param *diag_z;
    
    enum ap_var_type off_x_type;
    enum ap_var_type off_y_type;
    enum ap_var_type off_z_type;
    
    enum ap_var_type diag_x_type;
    enum ap_var_type diag_y_type;
    enum ap_var_type diag_z_type;

    off_x = AP_Param::find("MPU_ACC_F_X",&off_x_type);
    off_y = AP_Param::find("MPU_ACC_F_Y",&off_y_type);
    off_z = AP_Param::find("MPU_ACC_F_Z",&off_z_type);
    
    diag_x = AP_Param::find("MPU_ACC_T_X",&diag_x_type);
    diag_y = AP_Param::find("MPU_ACC_T_Y",&diag_y_type);
    diag_z = AP_Param::find("MPU_ACC_T_Z",&diag_z_type);
		
		ox = off_x->cast_to_float(off_x_type);
		oy = off_y->cast_to_float(off_y_type);		
		oz = off_z->cast_to_float(off_z_type);

    gx = diag_x->cast_to_float(diag_x_type);
    gy = diag_y->cast_to_float(diag_y_type);
		gz = diag_z->cast_to_float(diag_z_type);
		
		ox = oy = oz = 0.0f;
		gx = gy = gz = 1.0f;
		
		AP_Param::set_value(off_x_type,off_x,&ox);
		off_x->save();
		AP_Param::set_value(off_y_type,off_y,&oy);
		off_y->save();
		AP_Param::set_value(off_z_type,off_z,&oz);
		off_z->save();
		
		AP_Param::set_value(diag_x_type,diag_x,&gx);
		diag_x->save();
		AP_Param::set_value(diag_y_type,diag_y,&gy);
		diag_y->save();
		AP_Param::set_value(diag_z_type,diag_z,&gz);
		diag_z->save();
		
    _clients->clear();
    _clients->start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, 6, 0.5f);

    _started = true;
    _saving = false;
    _last_position_request_ms = 0;
    _step = 0;

    _last_result = ACCEL_CAL_NOT_STARTED;

    update_status();
}

void AccelCal::success()
{
    rt_kprintf("[INFO]Calibration successful\n");

    _last_result = ACCEL_CAL_SUCCESS;

    clear();
}

void AccelCal::cancel()
{
    rt_kprintf("[INFO]Calibration cancelled\n");

    _last_result = ACCEL_CAL_NOT_STARTED;

    clear();
}

void AccelCal::fail()
{
    rt_kprintf("[ERROR]Calibration FAILED\n");

    _last_result = ACCEL_CAL_FAILED;

    clear();
}

void AccelCal::clear()
{
    if (!_started) {
        return;
    }

    _clients->clear();

    _step = 0;
    _started = false;
    _saving = false;

    update_status();
}

void AccelCal::collect_sample()
{
    if (_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
        return;
    }

    _clients->collect_sample();
    // setup snooping of packets so we can see the COMMAND_ACK
    _start_collect_sample = false;
    update_status();
}

void AccelCal::update_status() 
{

    if (_clients == NULL) {
        // no calibrators
        _status = ACCEL_CAL_NOT_STARTED;
        return;
    }

    if (_clients->get_status() == ACCEL_CAL_FAILED) {
        _status = ACCEL_CAL_FAILED;         //fail if even one of the calibration has
        return;
    }

    if (_clients->get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
        _status = ACCEL_CAL_COLLECTING_SAMPLE;          // move to Collecting sample state if all the callibrators have
        return;
    }

    if (_clients->get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
        _status = ACCEL_CAL_WAITING_FOR_ORIENTATION;    // move to waiting for user ack for orientation confirmation
        return;
    }

    if (_clients->get_status() == ACCEL_CAL_NOT_STARTED) {
        _status = ACCEL_CAL_NOT_STARTED;    // we haven't started if all the calibrators haven't
        return;
    }

    _status = ACCEL_CAL_SUCCESS;    // we have succeeded calibration if all the calibrators have
    return;
}

bool AccelCal::gcs_vehicle_position(float position)
{
    if (_status == ACCEL_CAL_WAITING_FOR_ORIENTATION && is_equal((float) _step, position)) {
        _start_collect_sample = true;
        return true;
    }

    return false;
}

void AccelCal::_acal_save_calibrations(void)
{
    Vector3f off, diag;
    char buf[80];
    
    AP_Param *off_x;
    AP_Param *off_y;
    AP_Param *off_z;
    
    AP_Param *diag_x;
    AP_Param *diag_y;
    AP_Param *diag_z;
    
    enum ap_var_type off_x_type;
    enum ap_var_type off_y_type;
    enum ap_var_type off_z_type;
    
    enum ap_var_type diag_x_type;
    enum ap_var_type diag_y_type;
    enum ap_var_type diag_z_type;

    off_x = AP_Param::find("MPU_ACC_F_X",&off_x_type);
    off_y = AP_Param::find("MPU_ACC_F_Y",&off_y_type);
    off_z = AP_Param::find("MPU_ACC_F_Z",&off_z_type);
    
    diag_x = AP_Param::find("MPU_ACC_T_X",&diag_x_type);
    diag_y = AP_Param::find("MPU_ACC_T_Y",&diag_y_type);
    diag_z = AP_Param::find("MPU_ACC_T_Z",&diag_z_type);
    
    // 提取校准数据
    _clients->get_calibration(off, diag);
		
		Vector3f o,g;
		o.x = off.x;
		o.y = off.y;
		o.z = off.z;
		g.x = diag.x;
		g.y = diag.y;
		g.z = diag.z;
//    
//    sprintf(buf,"\n x_off:%.6f y_off:%.6f z_off:%.6f x_diag:%.6f y_diag:%.6f z_diag:%.6f",o.x, o.y, o.z, g.x, g.y, g.z);
//    rt_kprintf("[INFO] Accel Calibration : %s \n", buf);
//		rt_kprintf("RAW ACC DATA:\n");
//		for(int i = 0; i < 6; i++)
//		{
//			sprintf(buf,"%.6f \t%.6f \t%.6f", 0.001*_clients->_sample_buffer[i].accel.x, 0.001*_clients->_sample_buffer[i].accel.y, 0.001*_clients->_sample_buffer[i].accel.z);
//			rt_kprintf("%s\n", buf);
//		}
    
		
    // 参数设置
    AP_Param::set_value(off_x_type,off_x,&off.x);
    AP_Param::set_value(off_y_type,off_y,&off.y);
    AP_Param::set_value(off_z_type,off_z,&off.z);
    
    AP_Param::set_value(diag_x_type,diag_x,&diag.x);
    AP_Param::set_value(diag_y_type,diag_y,&diag.y);
    AP_Param::set_value(diag_z_type,diag_z,&diag.z);
    
    off_x->save();
    off_y->save();
    off_z->save();
    
    diag_x->save();
    diag_y->save();
    diag_z->save();
    
    free(buf);
}
