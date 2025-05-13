/*
 * QueueManager.h
 *
 *  Created on: Feb 14, 2025
 *      Author: keyuan
 */

#ifndef QUEUE_MANAGER_H
#define QUEUE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>  // Ensure this is included

bool PushToQueue_MTR_Cmd(uint32_t *cmd);  // Ensure this matches the function definition
bool PopFromQueue_MTR_Cmd(uint32_t *cmd);
bool isQueueEmpty();

#endif // QUEUE_MANAGER_H
