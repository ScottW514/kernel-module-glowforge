# Host-side tests for the kernel-independent parts of the module.
#
# These build with the host compiler against the source in ../src; nothing
# here needs a kernel tree. Run: make -C tests
#
# Each test is a standalone binary that returns nonzero on failure.

CC      ?= gcc
CFLAGS  ?= -O2
CFLAGS  += -std=gnu11 -Wall -Wextra -Werror -I../src

TESTS = interlock_test

.PHONY: all check clean

all: $(TESTS)

interlock_test: interlock_test.c ../src/cnc_interlock.c ../src/cnc_interlock.h
	$(CC) $(CFLAGS) -o $@ interlock_test.c ../src/cnc_interlock.c

check: all
	@set -e; for t in $(TESTS); do ./$$t; done

clean:
	rm -f $(TESTS)
