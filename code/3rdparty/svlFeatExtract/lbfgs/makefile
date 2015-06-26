# Makefile for the modified lbfgs library
# Stephen Gould <sgould@stanford.edu>
#

all : lbfgs.a

# to force 32-bit compile (set to 1 in make.local)
FORCE32BIT = 0
ifeq (,$(MACHTYPE))
  MACHTYPE = $(shell uname -im)
endif
ifeq (,$(findstring x86_64,$(MACHTYPE)))
  FORCE32BIT = 1
endif

# include local user make.local to override any settings
ifeq ($(wildcard ../../make.local), ../../make.local)
  -include ../../make.local
endif

ifeq ($(FORCE32BIT), 0)
  CPP = g++ -O2 -fPIC
else
  CPP = g++ -O2 -m32
endif

lbfgs.a: lbfgs.o ap.o
	ar rv $@ $?
	ranlib $@

lbfgs.o: lbfgs.cpp lbfgs.h
	$(CPP) -c -o lbfgs.o lbfgs.cpp

ap.o: ap.cpp ap.h
	$(CPP) -c -o ap.o ap.cpp

clean:
	-@rm *~                   >/dev/null 2>/dev/null
	-@rm *.o                  >/dev/null 2>/dev/null
	-@rm *.a                  >/dev/null 2>/dev/null
	@echo "Clean OK."
