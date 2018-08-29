#ifndef __ACCEL_CAL_H__
#define __ACCEL_CAL_H__

#include <rtthread.h>
#include "stm32f4xx.h"
#include "parameter.h"
#include "AccelCalibrator.h"

#ifdef __cplusplus

enum POS 
{
    ACCELCAL_VEHICLE_POS_LEVEL = 1,
    ACCELCAL_VEHICLE_POS_LEFT,
    ACCELCAL_VEHICLE_POS_RIGHT,
    ACCELCAL_VEHICLE_POS_NOSEDOWN,
    ACCELCAL_VEHICLE_POS_NOSEUP,
    ACCELCAL_VEHICLE_POS_BACK
};

class AccelCal 
{
public:
    AccelCal(AccelCalibrator* acc):
    _clients(acc),
    _started(false),
    _saving(false)
    {
        update_status();
    }
		bool cal_done;

    // start all the registered calibrations
    void start(void);

    // called on calibration cancellation
    void cancel();

    // Run an iteration of all registered calibrations
    void update();

    // get the status of the calibrator server as a whole
    accel_cal_status_t get_status() { return _status; }
    
    // Set vehicle position sent by the GCS
    bool gcs_vehicle_position(float position);
    
    void _acal_save_calibrations(void);

private:
    uint32_t _last_position_request_ms;
    uint8_t _step;
    accel_cal_status_t _status;
    accel_cal_status_t _last_result;
    Time_point  cal_timer;
    AccelCalibrator* _clients;

    // called on calibration success
    void success();

    // called on calibration failure
    void fail();

    // reset all the calibrators to there pre calibration stage so as to make them ready for next calibration request
    void clear();

    // proceed through the collection step for each of the registered calibrators
    void collect_sample();

    // update the state of the Accel calibrator server
    void update_status();

    // checks if no new sample has been received for considerable amount of time
    bool check_for_timeout();

    bool _started;
    bool _saving;
};

#endif
#endif
