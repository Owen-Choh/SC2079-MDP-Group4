/*
 * madgwick.c
 *
 *  Created on: Mar 8, 2025
 *      Author: keyuan
 */

#include "madgwick.h"

void MadgwickAHRSupdate(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz,
    float beta, float deltat,
    Quaternion *q)
{
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    float norm;

    // Normalize accel
    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 0.0f) {
        norm = 1.0f / norm;
        ax *= norm; ay *= norm; az *= norm;
    }

    // Normalize mag
    norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm > 0.0f) {
        norm = 1.0f / norm;
        mx *= norm; my *= norm; mz *= norm;
    }

    // This is a placeholder for the real Madgwick gradient steps
    // For brevity, just do a naive complimentary-like update:
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // If we applied full Madgwick, we'd subtract beta * gradient here

    // Integrate rate of change
    q0 += qDot1 * deltat;
    q1 += qDot2 * deltat;
    q2 += qDot3 * deltat;
    q3 += qDot4 * deltat;

    // Normalize quaternion
    norm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    q->q0 = q0; q->q1 = q1; q->q2 = q2; q->q3 = q3;
}

void Madgwick_quaternionToEuler(const Quaternion *q, float *roll, float *pitch, float *yaw)
{
    // roll (x-axis)
    float sinr_cosp = 2.0f * (q->q0 * q->q1 + q->q2 * q->q3);
    float cosr_cosp = 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / (float)M_PI;

    // pitch (y-axis)
    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp); // clamp
    else
        *pitch = asinf(sinp) * 180.0f / (float)M_PI;

    // yaw (z-axis)
    float siny_cosp = 2.0f * (q->q0 * q->q3 + q->q1 * q->q2);
    float cosy_cosp = 1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / (float)M_PI;
    if (*yaw < 0) *yaw += 360.0f;
}
