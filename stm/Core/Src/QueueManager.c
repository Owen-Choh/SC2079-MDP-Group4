/*
 * QueueManager.c
 *
 *  Created on: Feb 14, 2025
 *      Author: keyuan
 */
#include "QueueManager.h"

#define MOTOR_QUEUE_SIZE 20

static uint32_t motorQueue[MOTOR_QUEUE_SIZE];  // Command queue
static int front = -1, rear = -1;

// Check if queue is empty
bool isQueueEmpty()
{
    return (front == -1);
}

// Push command into queue
bool PushToQueue_MTR_Cmd(uint32_t *cmd)  // Accepts a pointer
{
    if ((rear + 1) % MOTOR_QUEUE_SIZE == front) {
        return false; // Queue is full
    }

    if (front == -1) {
        front = 0;
    }

    rear = (rear + 1) % MOTOR_QUEUE_SIZE;
    motorQueue[rear] = *cmd;  // Dereference pointer
    return true;
}

// Pop command from queue
bool PopFromQueue_MTR_Cmd(uint32_t *cmd)
{
    if (isQueueEmpty()) {
        return false;  // Queue is empty
    }

    *cmd = motorQueue[front];

    if (front == rear) {
        front = rear = -1;  // Reset queue if empty
    } else {
        front = (front + 1) % MOTOR_QUEUE_SIZE;
    }

    return true;
}
