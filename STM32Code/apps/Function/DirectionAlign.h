#ifndef __DIRECTIONALIGN_H__
#define __DIRECTIONALIGN_H__

#include "AP_Math.h"
#include "parameter.h"

class DirectionAlign
{
	public:
		DirectionAlign(void);
	  // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];	
	  
	  void aligning_update(Vector3f *euler, Vector3f *enc, Vector3f *mot);
	
	  uint8_t not_align(void) { if(_not_aligned == 1.0f) return 1; else return 0;};
	
	  void align_enc_direction(Vector3f *enc) { enc->x = enc->x * _enc_x_direction; enc->y = enc->y * _enc_y_direction; enc->z = enc->z * _enc_z_direction;};
	  void align_mot_direction(Vector3f *mot) { mot->x = mot->x * _mot_x_direction; mot->y = mot->y * _mot_y_direction; mot->z = mot->z * _mot_z_direction;};
	
	  void set_enc_direction(float x, float y, float z) { _enc_x_direction = x; _enc_y_direction = y; _enc_z_direction = z; };
		void set_mot_direction(float x, float y, float z) { _mot_x_direction = x; _mot_y_direction = y; _mot_z_direction = z; };
		
		Vector3f get_enc_direction(void) { Vector3f v(_enc_x_direction, _enc_y_direction, _enc_z_direction); return v;};
		
		Vector3f get_mot_direction(void) { Vector3f v(_mot_x_direction, _mot_y_direction, _mot_z_direction); return v;};
	
	protected:
		AP_Float  _not_aligned;
		AP_Float  _enc_x_direction;
	  AP_Float  _enc_y_direction;
	  AP_Float  _enc_z_direction;
	  AP_Float  _mot_x_direction;
	  AP_Float  _mot_y_direction;
	  AP_Float  _mot_z_direction;
	
	private:
		Vector3f  mot_magnitude;
	  Vector3f  mot_freq;
	  Vector3i  axis_not_aligned;
};


#endif

