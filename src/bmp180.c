/*
 * Part of senseimatic (protothreaded sensor drivers)
 *
 * Copyright (C) 2016 Daniel Thompson <daniel@redfelineninja.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 */

#include "bmp180.h"

#include <stdio.h>
#include <unistd.h>

const uint8_t calibration_base[] = { 0xaa };
const uint8_t ctrl_meas[] = { 0xf4 };
const uint8_t out_base[] = { 0xf6 };

static void bmp180_bist(bmp180_t *s)
{
	uint16_t raw_temp = 27898;
	uint32_t raw_pressure = 23843;

	s->ac1 = 408;
	s->ac2 = -72;
	s->ac3 = -14383;
	s->ac4 = 32741;
	s->ac5 = 32757;
	s->ac6 = 23153;
	s->b1 = 6190;
	s->b2 = 4;
	s->mb = -32768;
	s->mc = -8711;
	s->md = 2868;

	int32_t t = bmp180_get_temp(s, raw_temp);
	printf("T = %d\n", t);

	int32_t p = bmp180_get_pressure(s, raw_pressure);
	printf("p = %d\n", p);
}

static uint16_t get16(uint8_t *p) 
{
	return (p[0] << 8) + p[1];
}

pt_state_t bmp180_init(bmp180_t *s, uint32_t pi2c)
{
	uint8_t val;

	PT_BEGIN(&s->pt);

#if 0
	bmp180_bist(s);
#else
	(void) bmp180_bist;
#endif

	i2c_ctx_init(&s->i2c, pi2c);
	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_getreg(&s->i2c, 0x77, 0xd0, &val));
	if (val != 0x55)
		PT_FAIL();

	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_getreg(&s->i2c, 0x77, 0xaa, &val));

	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_write_read(&s->i2c, 0x77, calibration_base,
					      sizeof(calibration_base),
					      s->reply,
					      sizeof(s->reply)));

	s->oss = 0;

	/* interpret the reply */
	s->ac1 = get16(s->reply + 0);
	s->ac2 = get16(s->reply + 2);
	s->ac3 = get16(s->reply + 4);
	s->ac4 = get16(s->reply + 6);
	s->ac5 = get16(s->reply + 8);
	s->ac6 = get16(s->reply + 10);
	s->b1 = get16(s->reply + 12);
	s->b2 = get16(s->reply + 14);
	s->mb = get16(s->reply + 16);
	s->mc = get16(s->reply + 18);
	s->md = get16(s->reply + 20);


	PT_END();
}

pt_state_t bmp180_get_raw_temp(bmp180_t *s, uint16_t *raw_temp)
{
	PT_BEGIN(&s->pt);

	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_setreg(&s->i2c, 0x77, ctrl_meas[0], 0x2e));
	usleep(50000);

	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_write_read(&s->i2c, 0x77, out_base,
					      sizeof(out_base), s->reply, 2));

	*raw_temp = get16(s->reply);
	PT_END();
}

pt_state_t bmp180_get_raw_pressure(bmp180_t *s, uint32_t *raw_pressure)
{
	PT_BEGIN(&s->pt);

	PT_SPAWN_AND_CHECK(
	    &s->i2c.pt,
	    i2c_ctx_setreg(&s->i2c, 0x77, ctrl_meas[0], 0x34 + (s->oss << 6)));
	usleep(50000);

	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_write_read(&s->i2c, 0x77, out_base,
					      sizeof(out_base), s->reply, 3));

	*raw_pressure = (s->reply[0] << 16) + (s->reply[1] << 8) + s->reply[2];
	*raw_pressure >>= 8 - s->oss;
	PT_END();
}

int32_t bmp180_get_temp(bmp180_t *s, uint16_t raw_temp)
{
	int32_t x1, x2;

	x1 = (raw_temp - s->ac6) * s->ac5 / 32768;
	x2 = s->mc * 2048 / (x1 + s->md);
	s->b5 = x1 + x2;

	return (s->b5 + 8) / 16;
}

int32_t bmp180_get_pressure(bmp180_t *s, uint32_t raw_pressure)
{
	int p;

	int32_t b6 = s->b5 - 4000;
	int32_t x1 = (s->b2 * (b6 * b6 / 4096)) / 2048;
	int32_t x2 = s->ac2 * b6 / 2048;
	int32_t x3 = x1 + x2;
	int32_t b3 = (((s->ac1 * 4 + x3) << s->oss) + 2) / 4;
	x1 = s->ac3 * b6 / 8192;
	x2 = (s->b1 * (b6 * b6 / 4096)) / 65536;
	x3 = ((x1+ x2) + 2) / 4;
	uint32_t b4 = s->ac4 * (uint32_t)(x3 + 32768) / 32768;
	uint32_t b7 = ((unsigned long) raw_pressure - b3) * (50000 >> s->oss);
	if (b7 < 0x80000000)
		p = (b7 * 2) / b4;
	else
		p = (b7 / b4) * 2;
	x1 = (p / 256) * (p / 256);
	x1 = (x1 * 3038) / 65536;
	x2 = (-7357 * p) / 65536;
	p = p + (x1 + x2 + 3791) / 4;

	return p;
}


