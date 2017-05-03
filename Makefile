CFLAGS= -O2 -Wall
LDFLAGS = -lm
all: iplc-sim.c
	clang $(CFLAGS) origin.c -o iplc-sim $(LDFLAGS)

clean:
	rm iplc-sim
