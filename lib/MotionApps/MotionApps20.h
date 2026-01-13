// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//  2021/09/27 - split implementations out of header files, finally
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
#define _MPU6050_6AXIS_MOTIONAPPS20_H_

// take ownership of the "MPU6050" typedef
#define I2CDEVLIB_MPU6050_TYPEDEF

#include "MPU6050.h"
#include "prog_mem.h"
#include "helper_3dmath.h"

void MotionApps20_setAddress(uint8_t address);

uint8_t MotionApps20_dmpInitialize();
bool MotionApps20_dmpPacketAvailable();
uint8_t MotionApps20_dmpGetAccel(int32_t *data, const uint8_t* packet);
uint8_t MotionApps20_dmpGetAccel_int16(int16_t *data, const uint8_t* packet);
uint8_t  MotionApps20_dmpGetAccel_vectorInt16(VectorInt16 *v, const uint8_t* packet);
uint8_t MotionApps20_dmpGetQuaternion_int32(int32_t *data, const uint8_t* packet);
uint8_t MotionApps20_dmpGetQuaternion_int16(int16_t *data, const uint8_t* packet);
uint8_t MotionApps20_dmpGetQuaternion_qauternion(Quaternion *q, const uint8_t* packet);
uint8_t MotionApps20_dmpGetGyro_int32(int32_t *data, const uint8_t* packet);
uint8_t MotionApps20_dmpGetGyro_int16(int16_t *data, const uint8_t* packet);
uint8_t MotionApps20_dmpGetGyro_vectorInt16(VectorInt16 *v, const uint8_t* packet);
uint8_t MotionApps20_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
uint8_t MotionApps20_dmpGetGravity(int16_t *data, const uint8_t* packet);
uint8_t MotionApps20_dmpGetEuler(float *data, Quaternion *q);
uint8_t MotionApps20_dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t MotionApps20_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);
uint16_t MotionApps20_dmpGetFIFOPacketSize();
uint8_t MotionApps20_dmpGetCurrentFIFOPacket(uint8_t *data);


#endif /* _MPU6050_6AXIS_MOTIONAPPS20_H_ */
