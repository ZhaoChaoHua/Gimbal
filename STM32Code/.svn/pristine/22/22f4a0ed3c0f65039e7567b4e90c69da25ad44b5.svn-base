/**
 * @file rotation.cpp
 *
 * Vector rotation library
 */

#include "math.h"
#include "rotation.h"
#include "definitions.h"

void get_rot_matrix(enum Rotation rot, math::Matrix<3, 3> *rot_matrix)
{
	float roll  = DEG_TO_RAD * (float)rot_lookup[rot].roll;
	float pitch = DEG_TO_RAD * (float)rot_lookup[rot].pitch;
	float yaw   = DEG_TO_RAD * (float)rot_lookup[rot].yaw;

	rot_matrix->from_euler(roll, pitch, yaw);
}

#define HALF_SQRT_2 0.70710678118654757f

