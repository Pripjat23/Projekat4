# Makefile za virtuelnu platformu (SystemC + TLM 2.0)
#
#   make SYSTEMC_HOME=/putanja/do/systemc
#
# SYSTEMC_HOME je folder sa include/ i lib-linux64/ (ili lib/).
# Ako je SystemC instaliran iz paketa (libsystemc-dev), probaj SYSTEMC_HOME=/usr
#
# VAŽNO: -std mora biti isti kao pri prevodu SystemC biblioteke. Ako linker
# javi "undefined reference to sc_api_version_..._cxx2017..." (ili slično sa
# 2014/2011), promijeni CXXSTD ovdje (npr. c++14) ili ponovo prevedi SystemC
# sa -DCMAKE_CXX_STANDARD=17. Ostatak koda traži C++17 (std::clamp).

SYSTEMC_HOME ?= /usr/local/systemc
CXXSTD ?= c++17

CXXFLAGS = -std=$(CXXSTD) -O2 -Wall -I$(SYSTEMC_HOME)/include
LDFLAGS  = -L$(SYSTEMC_HOME)/lib-linux64 -L$(SYSTEMC_HOME)/lib \
           -Wl,-rpath,$(SYSTEMC_HOME)/lib-linux64 -Wl,-rpath,$(SYSTEMC_HOME)/lib -lsystemc

vp: vp_main.cpp vp_platform.hpp bfs_core.hpp bfs_model.hpp card_algo.hpp downscale.hpp
	g++ $(CXXFLAGS) vp_main.cpp -o vp $(LDFLAGS)

clean:
	rm -f vp
