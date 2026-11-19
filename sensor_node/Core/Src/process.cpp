/*
 * process.c
 *
 *  Created on: Nov 19, 2026
 *      Author: timotej-mestanek
 */

#include "process.h"

static uint32_t total_samples = 0;
static Sample   last_sample = {};

void process_sample(const Sample &s) {
    total_samples++;
    last_sample = s;

    // TODO: DC REMOVAL,etc. - EDGE COMPUTING

}

uint32_t process_count() {
    return total_samples;
}

const Sample &process_last() {
    return last_sample;
}

