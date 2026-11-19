/*
 * config.h
 *
 *  Created on: Nov 19, 2026
 *      Author: timot
 */
#pragma once
#include <cstdint>

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

constexpr uint16_t PACKET_SIZE = 12;
constexpr uint16_t MAX_PACKETS = 8;
constexpr uint16_t READ_MAX = PACKET_SIZE + MAX_PACKETS;
constexpr uint16_t FIFO_OVERFLOW = 480;
constexpr uint16_t PRINT_EVERY = 28;

#endif /* INC_CONFIG_H_ */
