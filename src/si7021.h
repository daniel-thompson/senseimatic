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

#ifndef RF_SI7021_H_
#define RF_SI7021_H_

#include <stdbool.h>
#include <stdint.h>
#include <librfn/protothreads.h>

#include "i2c_ctx.h"

typedef struct si7021 {
	pt_t pt;      //!< Protothread state
	i2c_ctx_t i2c;

	uint8_t reply[8];

	
} si7021_t;

pt_state_t si7021_init(si7021_t *s, uint32_t pi2c);
pt_state_t si7021_get_raw_temp(si7021_t *s, uint16_t *raw_temp);
int si7021_get_temp(si7021_t *s, uint16_t raw_temp);
pt_state_t si7021_get_raw_humidity(si7021_t *s, uint16_t *raw_rh);
int si7021_get_humidity(si7021_t *s, uint16_t raw_rh);

#endif // RF_SI7021_H_
