/*
 * process.h
 *
 *  Created on: Nov 19, 2026
 *      Author: timotej-mestanek
 */

#pragma once
#include "types.h"

#ifndef INC_PROCESS_H_
#define INC_PROCESS_H_

void process_sample(const Sample &s);
uint32_t process_count();
const Sample &process_last();



#endif /* INC_PROCESS_H_ */
