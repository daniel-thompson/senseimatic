#
# Makefile.am
#
# Part of librfn (a general utility library from redfelineninja.org.uk)
#
# Copyright (C) 2016 Daniel Thompson <daniel@redfelineninja.org.uk> 
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published
# by the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#


ACLOCAL_AMFLAGS = -I librfn/m4

bin_PROGRAMS = src/senseimatic
lib_LIBRARIES =
noinst_LIBRARIES =
noinst_PROGRAMS =
BUILT_SOURCES =
CLEANFILES =
TESTS =
TESTS_ENVIRONMENT=$(VALGRIND)

include Makefile-librfn.am

src_senseimatic_SOURCES = \
	src/bmp180.c \
	src/i2c_ctx.c \
	src/main.c \
	src/si7021.c
src_senseimatic_CPPFLAGS = $(LIBRFN_CFLAGS)
src_senseimatic_LDADD = $(LIBRFN_LIBS)
