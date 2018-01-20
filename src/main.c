/*
 * Part of senseimatic (protothreaded sensor drivers)
 *
 * Copyright (C) 2014-2018 Daniel Thompson <daniel@redfelineninja.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "librfn.h"

#include "bmp180.h"
#include "si7021.h"

/*
 * Calculate the dew point using the August-Roche-Magnus
 * approximation.
 *
 * For now we're going to work using the floating point (and the C
 * library math functions). Performance does matter here although a
 * micro-controller with a very limited FLASH might need an optimized
 * version...
 */
int dew_point(int temp, int rh)
{
	double t = temp / 10.0;
	double dp = 243.04 *
		    (log(rh / 100.0) + ((17.625 * t) / (243.04 + t))) /
		    (17.625 - log(rh / 100.0) - ((17.625 * t) / (243.04 + t)));

	return (10 * dp) + 0.5;
}

static uint32_t pi2c = 1;

static pt_state_t console_i2c(console_t *c)
{
	if (c->argc != 2)
		fprintf(c->out, "Usage: i2c <busno>\n");
	else
		pi2c = strtol(c->argv[1], NULL, 0);

	return PT_EXITED;
}
static const console_cmd_t cmd_i2c =
    CONSOLE_CMD_VAR_INIT("i2c", console_i2c);

static pt_state_t console_bmp180(console_t *c)
{
	static bmp180_t bmp180;
	uint16_t raw_temp;
	uint32_t raw_pressure;

	PT_BEGIN(&c->pt);

	PT_SPAWN_AND_CHECK(&bmp180.pt, bmp180_init(&bmp180, pi2c));
	PT_SPAWN_AND_CHECK(&bmp180.pt, bmp180_get_raw_temp(&bmp180, &raw_temp));
	PT_SPAWN_AND_CHECK(&bmp180.pt,
			   bmp180_get_raw_pressure(&bmp180, &raw_pressure));

	int t = bmp180_get_temp(&bmp180, raw_temp);
	int32_t p = bmp180_get_pressure(&bmp180, raw_pressure);

	printf("Raw pres: %d\n", raw_pressure);
	printf("Temp: %d.%d\n", t / 10, t % 10);
	printf("Pressure: %d\n", p);

	PT_END();
}
static const console_cmd_t cmd_bmp180 =
    CONSOLE_CMD_VAR_INIT("bmp180", console_bmp180);

static pt_state_t console_si7021(console_t *c)
{
	static si7021_t si7021;
	static uint16_t raw_temp, raw_rh;

	PT_BEGIN(&c->pt);

	PT_SPAWN_AND_CHECK(&si7021.pt, si7021_init(&si7021, pi2c));
	PT_SPAWN_AND_CHECK(&si7021.pt, si7021_get_raw_temp(&si7021, &raw_temp));
	PT_SPAWN_AND_CHECK(&si7021.pt,
			   si7021_get_raw_humidity(&si7021, &raw_rh));

	int t = si7021_get_temp(&si7021, raw_temp);
	int rh = si7021_get_humidity(&si7021, raw_rh);
	int dp = dew_point(t, rh);

	printf("Temp: %d.%d\n", t / 10, t % 10);
	printf("Relative humidity: %d\n", rh);
	printf("Dew point: %d.%d\n", dp / 10, dp % 10);

	PT_END();
}
static const console_cmd_t cmd_si7021 =
    CONSOLE_CMD_VAR_INIT("si7021", console_si7021);

static pt_state_t console_csv(console_t *c)
{
	static si7021_t si7021;
	static uint16_t raw_temp1, raw_rh;
	static bmp180_t bmp180;
	static uint16_t raw_temp2;
	static uint32_t raw_pressure;
	static uint64_t start, timeout;

	PT_BEGIN(&c->pt);

	PT_SPAWN_AND_CHECK(&si7021.pt, si7021_init(&si7021, pi2c));
	PT_SPAWN_AND_CHECK(&bmp180.pt, bmp180_init(&bmp180, pi2c));

	start = timeout = time_now();
	(void) start;

	while (1) {
		PT_SPAWN_AND_CHECK(&si7021.pt,
				   si7021_get_raw_temp(&si7021, &raw_temp1));
		PT_SPAWN_AND_CHECK(&si7021.pt,
				   si7021_get_raw_humidity(&si7021, &raw_rh));
		PT_SPAWN_AND_CHECK(&bmp180.pt,
				   bmp180_get_raw_temp(&bmp180, &raw_temp2));
		PT_SPAWN_AND_CHECK(&bmp180.pt, 
				   bmp180_get_raw_pressure(&bmp180,
					                   &raw_pressure));

		int t1 = si7021_get_temp(&si7021, raw_temp1);
		int rh = si7021_get_humidity(&si7021, raw_rh);
		int dp = dew_point(t1, rh);
		int t2 = bmp180_get_temp(&bmp180, raw_temp2);
		int p = bmp180_get_pressure(&bmp180, raw_pressure);

#if 0
		int minutes = (timeout - start) / (60 * 1000000);
		int hours = minutes / 60;
		minutes %= 60;
		int days = hours / 24;
		hours %= 24;
		printf("%2d,%02d:%02d,%2d.%d,%d,%2d.%d,%3d.%03d,%4.1f\n", days, hours,
		       minutes, t1 / 10, t1 % 10, rh, t2 / 10, t2 % 10,
		       p / 1000, p % 1000, dew_point);
#else
		char stamp[24];
		time_t now = time(NULL);
		strftime(stamp, sizeof(stamp), "%FT%H:%M:%S", localtime(&now));
		printf("%s,%2d.%d,%d,%2d.%d,%3d.%03d,%2d.%d\n", stamp,
		       t1 / 10, t1 % 10, rh, t2 / 10, t2 % 10,
		       p / 1000, p % 1000, dp / 10, dp % 10);
#endif

		timeout += 5 * 60 * 1000000;
		PT_WAIT_UNTIL(fibre_timeout(timeout));
	}

	PT_END();
}
static const console_cmd_t cmd_csv =
    CONSOLE_CMD_VAR_INIT("csv", console_csv);


typedef struct {
	int argc;
	char **argv;
	console_t *c;

	int i;
	pt_t pt;

	fibre_t fibre;
} eval_fibre_t;

int eval_fibre(fibre_t *fibre)
{
	eval_fibre_t *eval = containerof(fibre, eval_fibre_t, fibre);

	PT_BEGIN_FIBRE(fibre);

	for (eval->i = 1; eval->i < eval->argc; eval->i++) {
		PT_SPAWN(&eval->pt, console_eval(&eval->pt, eval->c,
						 eval->argv[eval->i]));
		PT_SPAWN(&eval->pt, console_eval(&eval->pt, eval->c, "\n"));
	}

	PT_SPAWN(&eval->pt, console_eval(&eval->pt, eval->c, "exit\n"));

	PT_END();
}

eval_fibre_t eval = { .fibre = FIBRE_VAR_INIT(eval_fibre) };

int main(int argc, char *argv[])
{
	console_t console;

	setvbuf(stdout, NULL, _IOLBF, 0);

	console_init(&console, stdout);
	console_register(&cmd_i2c);
	console_register(&cmd_bmp180);
	console_register(&cmd_si7021);
	console_register(&cmd_csv);

	if (argc > 1) {
		eval.argc = argc;
		eval.argv = argv;
		eval.c = &console;
		console.prompt = "";
		fibre_run(&eval.fibre);
	}

	fibre_scheduler_main_loop();

	return 0;
}
