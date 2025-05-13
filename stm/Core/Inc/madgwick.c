/*
 * madgwick.h
 *
 *  Created on: Mar 8, 2025
 *      Author: keyuan
 */
#ifndef MADGWICK_H
#define MADGWICK_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

/**
 * @brief  Madgwick filter update
 * @param  gx, gy, gz: gyro in rad/s
 * @param  ax, ay, az: accel in 'g'
 * @param  mx, my, mz: mag in microtesla
 * @param  beta: filter gain
 * @param  deltat: time step in seconds
 * @param  q: pointer to quaternion structure
 */
void MadgwickAHRSupdate(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz,
    float beta, float deltat,
    Quaternion *q);

/**
 * @brief  Convert quaternion to roll/pitch/yaw in degrees
 */
void Madgwick_quaternionToEuler(const Quaternion *q, float *roll, float *pitch, float *yaw);

#endif
