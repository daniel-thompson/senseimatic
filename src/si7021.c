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

#include "si7021.h"

#include <unistd.h>

#include <librfn.h>

static const uint8_t cmd_measure_rh[] = { 0xe5 };
static const uint8_t cmd_measure_temp[] = { 0xe3 };

static const uint8_t cmd_read_user_reg[] = { 0xe7 };
static const uint8_t cmd_reset[] = { 0xfe };
static const uint8_t cmd_read_id1[] = { 0xfa, 0x0f };
static const uint8_t cmd_read_id2[] = { 0xfc, 0xc9 };
static const uint8_t cmd_fw_rev[] = { 0x84, 0xb8 };

pt_state_t si7021_init(si7021_t *s, uint32_t pi2c)
{
	uint8_t val;

	PT_BEGIN(&s->pt);

	/* reset the device (reload settings) */
	i2c_ctx_init(&s->i2c, pi2c);
	PT_SPAWN_AND_CHECK(&s->i2c.pt, i2c_ctx_write(&s->i2c, 0x40, cmd_reset,
						     lengthof(cmd_reset)));
	usleep(20000);

	i2c_ctx_init(&s->i2c, pi2c);
	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_getreg(&s->i2c, 0x40, 0xe7, &val));
	PT_FAIL_ON(val != 0x3a);


	/* check the firmware revision */
	i2c_ctx_init(&s->i2c, pi2c);
	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_write_read(&s->i2c, 0x40, cmd_fw_rev,
					      lengthof(cmd_fw_rev), s->reply, 1));
	PT_FAIL_ON(s->reply[0] != 0xff && s->reply[0] != 0x20);

	/*
	 * Trying to access the serial number causes my device to jam with
	 * SDA pulled low. For now we will avoid doing this...
	 */
	i2c_ctx_init(&s->i2c, pi2c);
	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_write_read(&s->i2c, 0x40, cmd_read_id1,
					      lengthof(cmd_read_id1), s->reply,
					      8));
	i2c_ctx_init(&s->i2c, pi2c);
	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_write_read(&s->i2c, 0x40, cmd_read_id2,
					      lengthof(cmd_read_id2), s->reply,
					      6));

	PT_END();
}

pt_state_t si7021_get_raw_temp(si7021_t *s, uint16_t *raw_temp)
{
	PT_BEGIN(&s->pt);

	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_write_read(&s->i2c, 0x40, cmd_measure_temp,
					      lengthof(cmd_measure_temp),
					      s->reply, 3));

	*raw_temp = (s->reply[0] << 8) + s->reply[1];
	PT_END();
}

int si7021_get_temp(si7021_t *s, uint16_t raw_temp)
{
	return (17572 * raw_temp / 655360) - 468;
}

pt_state_t si7021_get_raw_humidity(si7021_t *s, uint16_t *raw_rh)
{
	PT_BEGIN(&s->pt);

	PT_SPAWN_AND_CHECK(&s->i2c.pt,
			   i2c_ctx_write_read(&s->i2c, 0x40, cmd_measure_rh,
					      lengthof(cmd_measure_rh),
					      s->reply, 3));

	*raw_rh = (s->reply[0] << 8) + s->reply[1];
	PT_END();
}

int si7021_get_humidity(si7021_t *s, uint16_t raw_rh)
{
	int rh = (125 * raw_rh / 65536) - 6;

	if (rh < 0)
		return 0;

	if (rh > 100)
		return 100;

	return rh;
}
