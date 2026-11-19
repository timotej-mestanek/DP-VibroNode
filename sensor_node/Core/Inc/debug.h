/*
 * debug.h
 *
 *  Created on: Nov 19, 2026
 *      Author: timotej-mestanek
 */

#pragma once
#include "types.h"

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_


void debug_log(const char *msg);           // BLOCKING - ERRORS etc.
void debug_print_sample(const Sample &s, uint32_t idx);   // DMA - FOR SAMPLES



#endif /* INC_DEBUG_H_ */
