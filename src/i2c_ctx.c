/*
 * Part of senseimatic (protothreaded sensor driver)
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

#include "i2c_ctx.h"

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

#include <librfn.h>

static int fd = -1;
static int current_pi2c = -1;

void i2c_ctx_init(i2c_ctx_t *c, uint32_t pi2c)
{
	/* update the file descriptor */
	if (current_pi2c != pi2c) {
		current_pi2c = pi2c;
		if (fd >= 0)
			close(fd);

		char *fname = xstrdup_printf("/dev/i2c-%d", pi2c);
		fd = open(fname, O_RDWR);
		free(fname);

		if (fd < 0)
			fprintf(stderr, "Cannot open I2C device: %s\n",
				strerror(errno));
	}

	i2c_ctx_reset(c);
}

void i2c_ctx_reset(i2c_ctx_t *c)
{
	c->msg_index = -1;
}

pt_state_t i2c_ctx_start(i2c_ctx_t *c)
{
	PT_BEGIN(&c->leaf);
	c->msg_index++;
	PT_END();
}

pt_state_t i2c_ctx_sendaddr(i2c_ctx_t *c, uint16_t addr,
				   uint8_t bytes_to_read)
{
	PT_BEGIN(&c->leaf);

	struct i2c_msg *m = &c->msgs[c->msg_index];
	m->addr = addr;
	m->flags = bytes_to_read ? I2C_M_RD : 0;
	m->len = bytes_to_read;
	m->buf = c->buf + 32 * c->msg_index;
	
	c->bytes_read = 0;

	PT_END();
}

pt_state_t i2c_ctx_senddata(i2c_ctx_t *c, uint8_t data)
{
	PT_BEGIN(&c->leaf);

	struct i2c_msg *m = &c->msgs[c->msg_index];
	m->buf[m->len++] = data;


	PT_END();
}

static int do_rdwr(i2c_ctx_t *c)
{
	struct i2c_rdwr_ioctl_data rdwr;
	rdwr.msgs = c->msgs;
	rdwr.nmsgs = c->msg_index + 1;

#if 0
	for (int i=0; i<rdwr.nmsgs; i++) {
		printf("%02x  %02x  %3d  %p  ", rdwr.msgs[i].addr,
		       rdwr.msgs[i].flags, rdwr.msgs[i].len, rdwr.msgs[i].buf);
		if (0 == (rdwr.msgs[i].flags & I2C_M_RD))
			hex_dump(rdwr.msgs[i].buf, rdwr.msgs[i].len);
	}
#endif

	int res = ioctl(fd, I2C_RDWR, &rdwr);
	if (res == rdwr.nmsgs) {
#if 0
		if (0 != (rdwr.msgs[rdwr.nmsgs - 1].flags & I2C_M_RD))
			hex_dump(rdwr.msgs[rdwr.nmsgs - 1].buf,
				 rdwr.msgs[rdwr.nmsgs - 1].len);
#endif
	} else {
#if 0
		if (0 != (rdwr.msgs[rdwr.nmsgs - 1].flags & I2C_M_RD))
			printf("\n");
#endif
		if (res > 0)
			fprintf(stderr,
				"Incomplete I2C transaction: %d of %d\n", res,
				rdwr.nmsgs);
		else
			fprintf(stderr, "Cannot launch I2C transaction: %s\n",
				strerror(errno));
	}

	return res < 0 ? res : 0;
}

pt_state_t i2c_ctx_getdata(i2c_ctx_t *c, uint8_t *data)
{
	struct i2c_msg *m = &c->msgs[c->msg_index];

	PT_BEGIN(&c->leaf);

	PT_FAIL_ON(c->msg_index < 0);
	PT_FAIL_ON(!(m->flags & I2C_M_RD));
	if (!c->bytes_read) {
		PT_FAIL_ON(0 != do_rdwr(c));
		*data = m->buf[0];
		c->bytes_read = 1;
	} else {
		*data = m->buf[c->bytes_read];
		c->bytes_read++;
	}
	if (c->bytes_read >= m->len)
		i2c_ctx_reset(c);

	PT_END();
}

pt_state_t i2c_ctx_stop(i2c_ctx_t *c)
{
	PT_BEGIN(&c->leaf);
	PT_FAIL_ON(0 != do_rdwr(c));
	i2c_ctx_reset(c);
	PT_END();
}

pt_state_t i2c_ctx_detect(i2c_ctx_t *c, i2c_device_map_t *map)
{
	PT_BEGIN(&c->pt);

	memset(map, 0, sizeof(*map));

	for (c->i = 0; c->i < 0x80; c->i++) {
		PT_SPAWN(&c->leaf, i2c_ctx_start(c));
		if (!PT_CHILD_OK())
			continue;

		PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, c->i, I2C_WRITE));
		if (!PT_CHILD_OK())
			continue;

		PT_SPAWN(&c->leaf, i2c_ctx_stop(c));
		if (!PT_CHILD_OK())
			continue;

		map->devices[c->i / 16] |= 1 << (c->i % 16);
	}

	PT_END();
}

pt_state_t i2c_ctx_setreg(i2c_ctx_t *c, uint16_t addr, uint16_t reg,
				 uint8_t val)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_start(c));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_senddata(c, reg));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_senddata(c, val));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_stop(c));

	PT_END();
}

pt_state_t i2c_ctx_getreg(i2c_ctx_t *c, uint16_t addr, uint16_t reg,
				 uint8_t *val)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_start(c));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_senddata(c, reg));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_start(c));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_READ));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_getdata(c, val));

	/* For reads STOP is generated automatically by sendaddr and/or
	 * getdata
	 */

	PT_END();
}

pt_state_t i2c_ctx_write(i2c_ctx_t *c, uint16_t addr, const uint8_t *data,
			 uint8_t len)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_start(c));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));

	for (c->i = 0; c->i < len; c->i++) {
		PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_senddata(c, data[c->i]));
	}

	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_stop(c));

	PT_END();
}

pt_state_t i2c_ctx_read(i2c_ctx_t *c, uint16_t addr, uint8_t *data,
			uint8_t len)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_start(c));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_sendaddr(c, addr, len));
	for (c->i = 0; c->i < len; c->i++)
		PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_getdata(c, data + c->i));

	PT_END();
}

pt_state_t i2c_ctx_write_read(i2c_ctx_t *c, uint16_t addr, const uint8_t *in,
			      uint8_t in_len, uint8_t *out, uint8_t out_len)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_start(c));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));
	for (c->i = 0; c->i < in_len; c->i++)
		PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_senddata(c, in[c->i]));

	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_start(c));
	PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_sendaddr(c, addr, out_len));
	for (c->i = 0; c->i < out_len; c->i++)
		PT_SPAWN_AND_CHECK(&c->leaf, i2c_ctx_getdata(c, out + c->i));

	PT_END();
}
