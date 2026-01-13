// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//  2021/09/27 - split implementations out of header files, finally
//  2019/07/08 - merged all DMP Firmware configuration items into the dmpMemory array
//             - Simplified dmpInitialize() to accomidate the dmpmemory array alterations

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2021 Jeff Rowberg

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

// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#include "MotionApps20.h"

static uint8_t devAddr;

void MotionApps20_setAddress(uint8_t address)
{
    devAddr = address;
}

uint8_t MotionApps20_dmpInitialize() 
{

	MPU6050_setSleepEnabled(false);

	// get MPU hardware revision
	MPU6050_setMemoryBank(0x10, true, true);
	MPU6050_setMemoryStartAddress(0x06);
	
	MPU6050_setMemoryBank(0, false, false);

	MPU6050_setI2CMasterModeEnabled(false);

	MPU6050_setSlaveAddress(0, devAddr);

	MPU6050_resetI2CMaster();
	HAL_DelayMs(20);
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

	MPU6050_setIntEnabled(1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT|1<<MPU6050_INTERRUPT_DMP_INT_BIT);

	MPU6050_setRate(4); // 1khz / (1 + 4) = 200 Hz

	MPU6050_setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

	// DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
	MPU6050_setDLPFMode(MPU6050_DLPF_BW_42);

	// DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

	// load DMP code into memory banks
	// DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
	// DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
	// DEBUG_PRINTLN(F(" bytes)"));
	if (!MPU6050_writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, true))
	{
		return 1; // Failed
	}

	// DEBUG_PRINTLN(F("Success! DMP code written and verified."));

	// Set the FIFO Rate Divisor int the DMP Firmware Memory
	unsigned char dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
	MPU6050_writeMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16, true, false); // Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16

	//write start address MSB into register
	MPU6050_setDMPConfig1(0x03);
	//write start address LSB into register
	MPU6050_setDMPConfig2(0x00);

	// DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
	MPU6050_setOTPBankValid(false);

	// DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
	MPU6050_setMotionDetectionThreshold(2);

	// DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
	MPU6050_setZeroMotionDetectionThreshold(156);

	// DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
	MPU6050_setMotionDetectionDuration(80);

	// DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
	MPU6050_setZeroMotionDetectionDuration(0);
	// DEBUG_PRINTLN(F("Enabling FIFO..."));
	MPU6050_setFIFOEnabled(true);

	// DEBUG_PRINTLN(F("Resetting DMP..."));
	MPU6050_resetDMP();

	// DEBUG_PRINTLN(F("DMP is good to go! Finally."));

	// DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
	MPU6050_setDMPEnabled(false);

	// DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
	MPU6050_dmpPacketSize = 42;

	// DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
	MPU6050_resetFIFO();
	MPU6050_getIntStatus();

	return 0; // success
}
// Nothing else changed

bool MotionApps20_dmpPacketAvailable() 
{
    return MPU6050_getFIFOCount() >= dmpGetFIFOPacketSize();
}

// uint8_t MotionApps20_dmpSetFIFORate(uint8_t fifoRate);
// uint8_t MotionApps20_dmpGetFIFORate();
// uint8_t MotionApps20_dmpGetSampleStepSizeMS();
// uint8_t MotionApps20_dmpGetSampleFrequency();
// int32_t MotionApps20_dmpDecodeTemperature(int8_t tempReg);

//uint8_t MotionApps20_dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t MotionApps20_dmpUnregisterFIFORateProcess(inv_obj_func func);
//uint8_t MotionApps20_dmpRunFIFORateProcesses();

// uint8_t MotionApps20_dmpSendQuaternion(uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendPacketNumber(uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MotionApps20_dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

uint8_t MotionApps20_dmpGetAccel(int32_t *data, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = MPU6050_dmpPacketBuffer;
    data[0] = (((uint32_t)packet[28] << 24) | ((uint32_t)packet[29] << 16) | ((uint32_t)packet[30] << 8) | packet[31]);
    data[1] = (((uint32_t)packet[32] << 24) | ((uint32_t)packet[33] << 16) | ((uint32_t)packet[34] << 8) | packet[35]);
    data[2] = (((uint32_t)packet[36] << 24) | ((uint32_t)packet[37] << 16) | ((uint32_t)packet[38] << 8) | packet[39]);
    return 0;
}
uint8_t MotionApps20_dmpGetAccel_int16(int16_t *data, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = MPU6050_dmpPacketBuffer;
    data[0] = (packet[28] << 8) | packet[29];
    data[1] = (packet[32] << 8) | packet[33];
    data[2] = (packet[36] << 8) | packet[37];
    return 0;
}
uint8_t  MotionApps20_dmpGetAccel_vectorInt16(VectorInt16 *v, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = MPU6050_dmpPacketBuffer;
    v -> x = (packet[28] << 8) | packet[29];
    v -> y = (packet[32] << 8) | packet[33];
    v -> z = (packet[36] << 8) | packet[37];
    return 0;
}
uint8_t MotionApps20_dmpGetQuaternion_int32(int32_t *data, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = MPU6050_dmpPacketBuffer;
    data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
    data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
    data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
    data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
    return 0;
}
uint8_t MotionApps20_dmpGetQuaternion_int16(int16_t *data, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = MPU6050_dmpPacketBuffer;
    data[0] = ((packet[0] << 8) | packet[1]);
    data[1] = ((packet[4] << 8) | packet[5]);
    data[2] = ((packet[8] << 8) | packet[9]);
    data[3] = ((packet[12] << 8) | packet[13]);
    return 0;
}

uint8_t MPU6050_dmpGetQuaternion_qauternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0)
    {
        return -1;
    }

    q->w = ((packet[0] << 8) | packet[1]);
    q->x = ((packet[4] << 8) | packet[5]);
    q->y = ((packet[8] << 8) | packet[9]);
    q->z = ((packet[12] << 8) | packet[13]);

    return 0;
}

uint8_t MotionApps20_dmpGetQuaternion_qauternion(Quaternion *q, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    Quaternion qI;
    uint8_t status = MPU6050_dmpGetQuaternion_qauternion(&qI, packet);
    if (status == 0) {
        q -> w = (float)qI.w / 16384.0f;
        q -> x = (float)qI.x / 16384.0f;
        q -> y = (float)qI.y / 16384.0f;
        q -> z = (float)qI.z / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}
// uint8_t MotionApps20_dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t MotionApps20_dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t MotionApps20_dmpGetGyro_int32(int32_t *data, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = MPU6050_dmpPacketBuffer;
    data[0] = (((uint32_t)packet[16] << 24) | ((uint32_t)packet[17] << 16) | ((uint32_t)packet[18] << 8) | packet[19]);
    data[1] = (((uint32_t)packet[20] << 24) | ((uint32_t)packet[21] << 16) | ((uint32_t)packet[22] << 8) | packet[23]);
    data[2] = (((uint32_t)packet[24] << 24) | ((uint32_t)packet[25] << 16) | ((uint32_t)packet[26] << 8) | packet[27]);
    return 0;
}
uint8_t MotionApps20_dmpGetGyro_int16(int16_t *data, const uint8_t* packet)
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = MPU6050_dmpPacketBuffer;
    data[0] = (packet[16] << 8) | packet[17];
    data[1] = (packet[20] << 8) | packet[21];
    data[2] = (packet[24] << 8) | packet[25];
    return 0;
}
uint8_t MotionApps20_dmpGetGyro_vectorInt16(VectorInt16 *v, const uint8_t* packet)
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = MPU6050_dmpPacketBuffer;
    v -> x = (packet[16] << 8) | packet[17];
    v -> y = (packet[20] << 8) | packet[21];
    v -> z = (packet[24] << 8) | packet[25];
    return 0;
}
// uint8_t MotionApps20_dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t MotionApps20_dmpGetLinearAccel(long *data, const uint8_t* packet);

// uint8_t MotionApps20_dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
//     // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
//     v -> x = vRaw -> x - gravity -> x*8192;
//     v -> y = vRaw -> y - gravity -> y*8192;
//     v -> z = vRaw -> z - gravity -> z*8192;
//     return 0;
// }

// uint8_t MotionApps20_dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t MotionApps20_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q)
{
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    VectorInt16Rotate(v, q);
    return 0;
}
// uint8_t MotionApps20_dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t MotionApps20_dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t MotionApps20_dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t MotionApps20_dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t MotionApps20_dmpGetGravity(long *data, const uint8_t* packet);
uint8_t MotionApps20_dmpGetGravity(int16_t *data, const uint8_t* packet) 
{
    /* +1g corresponds to +8192, sensitivity is 2g. */
    Quaternion qI;
    uint8_t status = MPU6050_dmpGetQuaternion_qauternion(&qI, packet);
    data[0] = ((int32_t)qI.x * qI.z - (int32_t)qI.w * qI.y) / 16384;
    data[1] = ((int32_t)qI.w * qI.x + (int32_t)qI.y * qI.z) / 16384;
    data[2] = ((int32_t)qI.w * qI.w - (int32_t)qI.x * qI.x
	       - (int32_t)qI.y * qI.y + (int32_t)qI.z * qI.z) / (int32_t)(2 * 16384L);
    return status;
}

// uint8_t MotionApps20_dmpGetGravity(VectorFloat *v, Quaternion *q) {
//     v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
//     v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
//     v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
//     return 0;
// }

// uint8_t MotionApps20_dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t MotionApps20_dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t MotionApps20_dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t MotionApps20_dmpGetEIS(long *data, const uint8_t* packet);

uint8_t MotionApps20_dmpGetEuler(float *data, Quaternion *q)
{
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}

// #ifdef USE_OLD_DMPGETYAWPITCHROLL
// uint8_t MotionApps20_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
//     // yaw: (about Z axis)
//     data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
//     // pitch: (nose up/down, about Y axis)
//     data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
//     // roll: (tilt left/right, about X axis)
//     data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
//     return 0;
// }
// #else 
// uint8_t MotionApps20_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
//     // yaw: (about Z axis)
//     data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
//     // pitch: (nose up/down, about Y axis)
//     data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
//     // roll: (tilt left/right, about X axis)
//     data[2] = atan2(gravity -> y , gravity -> z);
//     if (gravity -> z < 0) {
//         if(data[1] > 0) {
//             data[1] = PI - data[1]; 
//         } else { 
//             data[1] = -PI - data[1];
//         }
//     }
//     return 0;
// }
// #endif

// uint8_t MotionApps20_dmpGetAccelFloat(float *data, const uint8_t* packet);
// uint8_t MotionApps20_dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t MotionApps20_dmpProcessFIFOPacket(const unsigned char *dmpData)
{
    (void)dmpData; // unused parameter
    /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
        if (dmpData[k] < 0x10) Serial.print("0");
        Serial.print(dmpData[k], HEX);
        Serial.print(" ");
    }
    Serial.print("\n");*/
    //Serial.println((uint16_t)dmpPacketBuffer);
    return 0;
}
uint8_t MotionApps20_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed)
{
    uint8_t status;
    uint8_t buf[MPU6050_dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        MPU6050_getFIFOBytes(buf, MPU6050_dmpPacketSize);

        // process packet
        if ((status = MotionApps20_dmpProcessFIFOPacket(buf)) > 0) return status;
        
        // increment external process count variable, if supplied
        if (processed != 0) (*processed)++;
    }
    return 0;
}

// uint8_t MotionApps20_dmpSetFIFOProcessedCallback(void (*func) (void));

// uint8_t MotionApps20_dmpInitFIFOParam();
// uint8_t MotionApps20_dmpCloseFIFO();
// uint8_t MotionApps20_dmpSetGyroDataSource(uint_fast8_t source);
// uint8_t MotionApps20_dmpDecodeQuantizedAccel();
// uint32_t MotionApps20_dmpGetGyroSumOfSquare();
// uint32_t MotionApps20_dmpGetAccelSumOfSquare();
// void MotionApps20_dmpOverrideQuaternion(long *q);
uint16_t MotionApps20_dmpGetFIFOPacketSize()
{
    return MPU6050_dmpPacketSize;
}



uint8_t MotionApps20_dmpGetCurrentFIFOPacket(uint8_t *data) 
{ // overflow proof
    return(MPU6050_getCurrentFIFOPacket(data, MPU6050_dmpPacketSize));
}
