#include "AccelCalibrator.h"
#include <stdio.h>
#include <stdlib.h>

AccelCalibrator::AccelCalibrator() :
                _sample_buffer(NULL),
								_conf_tolerance(ACCEL_CAL_TOLERANCE)
{
    clear();
}

/*
    Select options, initialise variables and initiate accel calibration
    Options:
    Fit Type:       Will assume that if accelerometer static samples around all possible orientatio
                    are spread in space will cover a surface of AXIS_ALIGNED_ELLIPSOID or any general 
                    ELLIPSOID as chosen

    Num Samples:    Number of samples user should will be gathering, please note that with type of surface
                    chosen the minimum number of samples required will vary, for instance in the case of AXIS
                    ALIGNED ELLIPSOID atleast 6 distinct samples are required while for generic ELLIPSOIDAL fit
                    which will include calculation of offdiagonal parameters too requires atleast 8 parameters
                    to form a distinct ELLIPSOID

    Sample Time:    Time over which the samples will be gathered and averaged to add to a single sample for least
                    square regression

    offset,diag,offdiag: initial parameter values for LSQ calculation
*/
void AccelCalibrator::start(enum accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time) 
{
    start(fit_type, num_samples, sample_time, Vector3f(0,0,0), Vector3f(1,1,1), Vector3f(0,0,0));
}

void AccelCalibrator::start(enum accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time, Vector3f offset, Vector3f diag, Vector3f offdiag) 
{
    if (_status == ACCEL_CAL_FAILED || _status == ACCEL_CAL_SUCCESS) {
        clear();
    }
    if (_status != ACCEL_CAL_NOT_STARTED) {
        return;
    }

    _conf_num_samples = num_samples;
    _conf_sample_time = sample_time;
    _conf_fit_type = fit_type;

    const uint16_t faces = 2*_conf_num_samples-4;
    const float a = (4.0f * M_PI / (3.0f * faces)) + M_PI / 3.0f;
    const float theta = 0.5f * acosf(cosf(a) / (1.0f - cosf(a)));
    _min_sample_dist = CONSTANTS_ONE_G * 2*sinf(theta/2);

    _param.offset = offset;
    _param.diag = diag;
    _param.offdiag = offdiag;

    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
            if (_conf_num_samples < 6) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
        case ACCEL_CAL_ELLIPSOID:
            if (_conf_num_samples < 8) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
    }

    set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
}

// set Accel calibrator status to make itself ready for future accel cals
void AccelCalibrator::clear() 
{
    set_status(ACCEL_CAL_NOT_STARTED);
}

// returns true if accel calibrator is running
bool AccelCalibrator::running() 
{
    return _status == ACCEL_CAL_WAITING_FOR_ORIENTATION || _status == ACCEL_CAL_COLLECTING_SAMPLE;
}

// set Accel calibrator to start collecting samples in the next cycle
void AccelCalibrator::collect_sample() 
{
    set_status(ACCEL_CAL_COLLECTING_SAMPLE);
}


// collect and avg sample to be passed onto LSQ estimator after all requisite orientations are done
void AccelCalibrator::new_sample(const Vector3f& accel_data, float dt) 
{
    if (_status != ACCEL_CAL_COLLECTING_SAMPLE) {
        return;
    }

    if (_samples_collected >= _conf_num_samples) {
        set_status(ACCEL_CAL_FAILED);
        return;
    }

    _sample_buffer[_samples_collected].accel += accel_data;
    _sample_buffer[_samples_collected].delta_time += dt;
    _sample_buffer[_samples_collected].sample_num++;

    if (_sample_buffer[_samples_collected].sample_num > 1000) {
        Vector3f sample = _sample_buffer[_samples_collected].accel/_sample_buffer[_samples_collected].sample_num;
        if (!accept_sample(sample)) {
            set_status(ACCEL_CAL_FAILED);
            return;
        }

        _samples_collected++;

        if (_samples_collected >= _conf_num_samples) {
            run_fit(MAX_ITERATIONS, _fitness);

            if (_fitness < _conf_tolerance && accept_result()) {
                set_status(ACCEL_CAL_SUCCESS);
            } else {
                set_status(ACCEL_CAL_FAILED);
            }
        } else {
            set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
        }
    }
}

// determines if the result is acceptable
bool AccelCalibrator::accept_result() const 
{
    if (fabsf(_param.offset.x) > CONSTANTS_ONE_G ||
        fabsf(_param.offset.y) > CONSTANTS_ONE_G ||
        fabsf(_param.offset.z) > CONSTANTS_ONE_G ||
        _param.diag.x < 0.8f || _param.diag.x > 1.2f ||
        _param.diag.y < 0.8f || _param.diag.y > 1.2f ||
        _param.diag.z < 0.8f || _param.diag.z > 1.2f) {
        return false;
    } else {
        return true;
    }
}

// interface for LSq estimator to read sample buffer sent after conversion from delta velocity
// to averaged acc over time
bool AccelCalibrator::get_sample(uint8_t i, Vector3f& s) const 
{
    if (i >= _samples_collected) {
        return false;
    }
    s = _sample_buffer[i].accel / _sample_buffer[i].sample_num;
    return true;
}

// returns truen and sample corrected with diag offdiag parameters as calculated by LSq estimation procedure
// returns false if no correct parameter exists to be applied along with existing sample without corrections
bool AccelCalibrator::get_sample_corrected(uint8_t i, Vector3f& s) const 
{
    if (_status != ACCEL_CAL_SUCCESS || !get_sample(i,s)) {
        return false;
    }

    Matrix3f M (
        _param.diag.x    , _param.offdiag.x , _param.offdiag.y,
        _param.offdiag.x , _param.diag.y    , _param.offdiag.z,
        _param.offdiag.y , _param.offdiag.z , _param.diag.z
    );

    s = M*(s+_param.offset);

    return true;
}



// returns spherical fit paramteters
void AccelCalibrator::get_calibration(Vector3f& offset) const 
{
    offset = -_param.offset;
}

// returns axis aligned ellipsoidal fit parameters
void AccelCalibrator::get_calibration(Vector3f& offset, Vector3f& diag) const 
{
    offset = -_param.offset;
    diag = _param.diag;
}

// returns generic ellipsoidal fit parameters
void AccelCalibrator::get_calibration(Vector3f& offset, Vector3f& diag, Vector3f& offdiag) const
{
    offset = -_param.offset;
    diag = _param.diag;
    offdiag = _param.offdiag;
}

/////////////////////////////////////////////////////////////
////////////////////// PRIVATE METHODS //////////////////////
/////////////////////////////////////////////////////////////

/*
  The sample acceptance distance is determined as follows:
  For any regular polyhedron with triangular faces, the angle theta subtended
  by two closest points is defined as
 
       theta = arccos(cos(A)/(1-cos(A)))
 
  Where:
       A = (4pi/F + pi)/3
  and
       F = 2V - 4 is the number of faces for the polyhedron in consideration,
       which depends on the number of vertices V
 
  The above equation was proved after solving for spherical triangular excess
  and related equations.
 */
bool AccelCalibrator::accept_sample(const Vector3f& sample)
{
    if (_sample_buffer == NULL) {
        return false;
    }

    for(uint8_t i=0; i < _samples_collected; i++) {
        Vector3f other_sample;
        get_sample(i, other_sample);

        if ((other_sample - sample).length() < _min_sample_dist){
            return false;
        }
    }
    return true;
}

// sets status of calibrator and takes appropriate actions
void AccelCalibrator::set_status(enum accel_cal_status_t status) 
{
    switch (status) {
        case ACCEL_CAL_NOT_STARTED:                 
            //Calibrator not started
            _status = ACCEL_CAL_NOT_STARTED;

            _samples_collected = 0;
            if (_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }

            break;

        case ACCEL_CAL_WAITING_FOR_ORIENTATION:     
            //Callibrator has been started and is waiting for user to ack after orientation setting
            if (!running()) {
                _samples_collected = 0;
                if (_sample_buffer == NULL) {
                    _sample_buffer = (struct AccelSample*)calloc(_conf_num_samples,sizeof(struct AccelSample));
                    if (_sample_buffer == NULL) {
                        set_status(ACCEL_CAL_FAILED);
                        break;
                    }
                }
            }
            if (_samples_collected >= _conf_num_samples) {
                break;
            }
            _status = ACCEL_CAL_WAITING_FOR_ORIENTATION;
            break;

        case ACCEL_CAL_COLLECTING_SAMPLE:
            // Calibrator is waiting on collecting samples from acceleromter for amount of 
            // time as requested by user/GCS
            if (_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
                break;
            }

            _status = ACCEL_CAL_COLLECTING_SAMPLE;
            break;

        case ACCEL_CAL_SUCCESS:
            // Calibrator has successfully fitted the samples to user requested surface model 
            // and has passed tolerance test
            if (_status != ACCEL_CAL_COLLECTING_SAMPLE) {
                break;
            }

            _status = ACCEL_CAL_SUCCESS;
            break;

        case ACCEL_CAL_FAILED:
            // Calibration has failed with reasons that can range from 
            // bad sample data leading to faillure in tolerance test to lack of distinct samples
            if (_status == ACCEL_CAL_NOT_STARTED) {
                break;
            }

            _status = ACCEL_CAL_FAILED;
            break;
    };
}

/*
    Run Gauss Newton fitting algorithm over the sample space and come up with offsets, diagonal/scale factors
    and crosstalk/offdiagonal parameters
*/
void AccelCalibrator::run_fit(uint8_t max_iterations, float& fitness)
{
    if (_sample_buffer == NULL) {
        return;
    }
    fitness = calc_mean_squared_residuals(_param);
    float min_fitness = fitness;
    struct param_t fit_param = _param;
    VectorP fit_param_p;
    
    float JTJ[ACCEL_CAL_MAX_NUM_PARAMS*ACCEL_CAL_MAX_NUM_PARAMS] = {0.0f};
    VectorN<float,ACCEL_CAL_MAX_NUM_PARAMS> jacob;
    VectorP JTFI;
    
    uint8_t num_iterations = 0;

    while(num_iterations < max_iterations)
    {
        JTFI.zero();
        for(uint16_t k = 0; k<_samples_collected; k++) {
            Vector3f sample;
            get_sample(k, sample);

            jacob.zero();

            calc_jacob(sample, fit_param, jacob);

            for(uint8_t i = 0; i < get_num_params(); i++) {
                // compute JTJ
                for(uint8_t j = 0; j < get_num_params(); j++) {
                    JTJ[i*get_num_params()+j] += jacob[i] * jacob[j];
                }
                // compute JTFI
                JTFI[i] += jacob[i] * calc_residual(sample, fit_param);
            }
        }

        if (!inverse(JTJ, JTJ, get_num_params())) {
            return;
        }

        fit_param_p[0] = fit_param.offset.x;
        fit_param_p[1] = fit_param.offset.y;
        fit_param_p[2] = fit_param.offset.z;
        fit_param_p[3] = fit_param.diag.x;
        fit_param_p[4] = fit_param.diag.y;
        fit_param_p[5] = fit_param.diag.z;
        fit_param_p[6] = fit_param.offdiag.x;
        fit_param_p[7] = fit_param.offdiag.y;
        fit_param_p[8] = fit_param.offdiag.z;
        for(uint8_t row=0; row < get_num_params(); row++) {
            for(uint8_t col=0; col < get_num_params(); col++) {
                fit_param_p[row] -= JTFI[col] * JTJ[row*get_num_params()+col];
            }
        }
        fit_param.offset.x = fit_param_p[0];
        fit_param.offset.y = fit_param_p[1];
        fit_param.offset.z = fit_param_p[2];
        fit_param.diag.x = fit_param_p[3];
        fit_param.diag.y = fit_param_p[4];
        fit_param.diag.z = fit_param_p[5];
        fit_param.offdiag.x = fit_param_p[6];
        fit_param.offdiag.y = fit_param_p[7];
        fit_param.offdiag.z = fit_param_p[8];
        
        fitness = calc_mean_squared_residuals(fit_param);

        if (isnan(fitness) || isinf(fitness)) {
            return;
        }

        if (fitness < min_fitness) {
            min_fitness = fitness;
            _param = fit_param;
        }
        num_iterations++;
    }
    free(JTJ);
}

// calculates residual from samples(corrected using supplied parameter) to gravity
// used to create Fitness column matrix
float AccelCalibrator::calc_residual(const Vector3f& sample, const struct param_t& params) const 
{
    Matrix3f M (
        params.diag.x    , params.offdiag.x , params.offdiag.y,
        params.offdiag.x , params.diag.y    , params.offdiag.z,
        params.offdiag.y , params.offdiag.z , params.diag.z
    );
    return CONSTANTS_ONE_G - (M*(sample+params.offset)).length();
}

// calculated the total mean squared fitness of all the collected samples using parameters
// converged to LSq estimator so far
float AccelCalibrator::calc_mean_squared_residuals() const
{
    return calc_mean_squared_residuals(_param);
}

// calculated the total mean squared fitness of all the collected samples using parameters
// supplied
float AccelCalibrator::calc_mean_squared_residuals(const struct param_t& params) const
{
    if (_sample_buffer == NULL || _samples_collected == 0) {
        return 1.0e30f;
    }
    float sum = 0.0f;
    for(uint16_t i=0; i < _samples_collected; i++){
        Vector3f sample;
        get_sample(i, sample);
        float resid = calc_residual(sample, params);
        sum += sq(resid);
    }
    sum /= _samples_collected;
    return sum;
}

// calculate jacobian, a matrix that defines relation to variation in fitness with variation in each of the parameters
// this is used in LSq estimator to adjust variation in parameter to be used for next iteration of LSq
void AccelCalibrator::calc_jacob(const Vector3f& sample, const struct param_t& params, VectorP &ret) const 
{
    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
        case ACCEL_CAL_ELLIPSOID: {
            const Vector3f &offset = params.offset;
            const Vector3f &diag = params.diag;
            const Vector3f &offdiag = params.offdiag;
            Matrix3f M(
                diag.x    , offdiag.x , offdiag.y,
                offdiag.x , diag.y    , offdiag.z,
                offdiag.y , offdiag.z , diag.z
            );

            float A =  (diag.x    * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
            float B =  (offdiag.x * (sample.x + offset.x)) + (diag.y    * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
            float C =  (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z    * (sample.z + offset.z));
            float length = (M*(sample+offset)).length();

            // 0-2: offsets
            ret[0] = -1.0f * (((diag.x    * A) + (offdiag.x * B) + (offdiag.y * C))/length);
            ret[1] = -1.0f * (((offdiag.x * A) + (diag.y    * B) + (offdiag.z * C))/length);
            ret[2] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z    * C))/length);
            // 3-5: diagonals
            ret[3] = -1.0f * ((sample.x + offset.x) * A)/length;
            ret[4] = -1.0f * ((sample.y + offset.y) * B)/length;
            ret[5] = -1.0f * ((sample.z + offset.z) * C)/length;
            // 6-8: off-diagonals
            ret[6] = -1.0f * (((sample.y + offset.y) * A) + ((sample.x + offset.x) * B))/length;
            ret[7] = -1.0f * (((sample.z + offset.z) * A) + ((sample.x + offset.x) * C))/length;
            ret[8] = -1.0f * (((sample.z + offset.z) * B) + ((sample.y + offset.y) * C))/length;
            return;
        }
    };
}

// returns number of parameters are required for selected Fit type
uint8_t AccelCalibrator::get_num_params() const 
{
    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
            return 6;
        case ACCEL_CAL_ELLIPSOID:
            return 9;
        default:
            return 6;
    }
}
