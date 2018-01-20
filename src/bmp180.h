/*
 * Part of senseimatic (protothreaded sensor drivers)
 *
 * Copyright (C) 2016 Daniel Thompson <daniel@redfelineninja.org.uk>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RF_BMP180_H_
#define RF_BMP180_H_

#include <stdbool.h>
#include <stdint.h>
#include <librfn/protothreads.h>

#include "i2c_ctx.h"

typedef struct bmp180 {
	pt_t pt;      //!< Protothread state
	i2c_ctx_t i2c;

	uint8_t reply[22];

	uint8_t oss;

	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;

	int32_t b5;
} bmp180_t;

pt_state_t bmp180_init(bmp180_t *s, uint32_t pi2c);
pt_state_t bmp180_get_raw_temp(bmp180_t *s, uint16_t *raw_temp);
int32_t bmp180_get_temp(bmp180_t *s, uint16_t raw_temp);
pt_state_t bmp180_get_raw_pressure(bmp180_t *s, uint32_t *raw_pressure);
int32_t bmp180_get_pressure(bmp180_t *s, uint32_t raw_pressure);

#endif // RF_BMP180_H_
