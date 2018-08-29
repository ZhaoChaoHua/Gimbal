/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file rotation.h
 *
 * Vector rotation library
 */

#ifndef ROTATION_H_
#define ROTATION_H_

#include <mathlib.h>

/**
 * Enum for board and external compass rotations.
 * This enum maps from board attitude to airframe attitude.
 */
enum Rotation {
    ROTATION_NONE                     = 0,
    ROTATION_YAW_90                   = 1,
    ROTATION_YAW_180                  = 2,
    ROTATION_YAW_270                  = 3,
    ROTATION_ROLL_180                 = 4,
    ROTATION_ROLL_180_YAW_90          = 5,
    ROTATION_PITCH_180                = 6,
    ROTATION_ROLL_180_YAW_270         = 7,
    ROTATION_ROLL_90                  = 8,
    ROTATION_ROLL_90_YAW_90           = 9,
    ROTATION_ROLL_270                 = 10,
    ROTATION_ROLL_270_YAW_90          = 11,
    ROTATION_PITCH_90                 = 12,
    ROTATION_PITCH_270                = 13,
    ROTATION_PITCH_180_YAW_90         = 14,
    ROTATION_PITCH_180_YAW_270        = 15,
    
    ROTATION_ROLL_90_PITCH_90         = 16,
    ROTATION_ROLL_180_PITCH_90        = 17,
    ROTATION_ROLL_270_PITCH_90        = 18,
    ROTATION_ROLL_90_PITCH_180        = 19,
    ROTATION_ROLL_270_PITCH_180       = 20,
    ROTATION_ROLL_90_PITCH_270        = 21,
    ROTATION_ROLL_180_PITCH_270       = 22,
    ROTATION_ROLL_270_PITCH_270       = 23,
    ROTATION_ROLL_90_PITCH_180_YAW_90 = 24,
    ROTATION_ROLL_90_YAW_270          = 25,
    ROTATION_MAX
};

typedef struct {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
} rot_lookup_t;

const rot_lookup_t rot_lookup[] = {
	{  0,   0,   0 },  // 0
	{  0,   0,  45 },  // 1
	{  0,   0,  90 },  // 2
	{  0,   0, 135 },  // 3
	{  0,   0, 180 },  // 4
	{  0,   0, 225 },  // 5
	{  0,   0, 270 },  // 6
	{  0,   0, 315 },  // 7
	{180,   0,   0 },  // 8
	{180,   0,  45 },  // 9
	{180,   0,  90 },  // 10
	{180,   0, 135 },  // 11
	{  0, 180,   0 },  // 12
	{180,   0, 225 },  // 13
	{180,   0, 270 },  // 14
	{180,   0, 315 },  // 15
	{ 90,   0,   0 },  // 16
	{ 90,   0,  45 },  // 17
	{ 90,   0,  90 },  // 18
	{ 90,   0, 135 },  // 19
	{270,   0,   0 },  // 20
	{270,   0,  45 },  // 21
	{270,   0,  90 },  // 22
	{270,   0, 135 },  // 23
	{  0,  90,   0 },  // 24
	{  0, 270,   0 },  // 25
    {  0, 180,  90 },  // 26
    {  0, 180, 270 },  // 27
    { 90,  90,   0 },  // 28
    {180,  90,   0 },  // 29
    {270,  90,   0 },  // 30
    { 90, 180,   0 },  // 31
    {270, 180,   0 },  // 32
    { 90, 270,   0 },  // 33
    {180, 270,   0 },  // 34
    {270, 270,   0 },  // 35
    { 90, 180,  90 },  // 36
    { 90,   0, 270 },  // 37
};

const inline char* detect_orientation_str(enum Rotation orientation)
{
	static const char* rgOrientationStrs[] = {
        "ROTATION_NONE",//                     = 0,
        "ROTATION_YAW_90",//                   = 1,
        "ROTATION_YAW_180",//                  = 2,
        "ROTATION_YAW_270",//                  = 3,
        "ROTATION_ROLL_180",//                 = 4,
        "ROTATION_ROLL_180_YAW_90",//          = 5,
        "ROTATION_PITCH_180",//                = 6,
        "ROTATION_ROLL_180_YAW_270",//         = 7,
        "ROTATION_ROLL_90",//                  = 8,
        "ROTATION_ROLL_90_YAW_90",//           = 9,
        "ROTATION_ROLL_270",//                 = 10,
        "ROTATION_ROLL_270_YAW_90",//          = 11,
        "ROTATION_PITCH_90",//                 = 12,
        "ROTATION_PITCH_270",//                = 13,
        "ROTATION_PITCH_180_YAW_90",//         = 14,
        "ROTATION_PITCH_180_YAW_270",//        = 15,
        
        "ROTATION_ROLL_90_PITCH_90",//         = 16,
        "ROTATION_ROLL_180_PITCH_90",//        = 17,
        "ROTATION_ROLL_270_PITCH_90",//        = 30,
        "ROTATION_ROLL_90_PITCH_180",//        = 31,
        "ROTATION_ROLL_270_PITCH_180",//       = 32,
        "ROTATION_ROLL_90_PITCH_270",//        = 33,
        "ROTATION_ROLL_180_PITCH_270",//       = 34,
        "ROTATION_ROLL_270_PITCH_270",//       = 35,
        "ROTATION_ROLL_90_PITCH_180_YAW_90",// = 36,
        "ROTATION_ROLL_90_YAW_270",//          = 37,
        "ROTATION_MAX"
	};
	return rgOrientationStrs[orientation];
}

/**
 * Get the rotation matrix
 */
void get_rot_matrix(enum Rotation rot, math::Matrix<3, 3> *rot_matrix);


#endif /* ROTATION_H_ */
