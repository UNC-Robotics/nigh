SHELL = bash
CXX ?= clang++
#CXX = g++-7
CXXFLAGS += -std=c++17 -I../src -Wall -pedantic -Wno-ignored-attributes

# Use libstdc++ on Linux, and libc++ on 
# UNAME_S := $(shell uname -s)
# ifeq ($(UNAME_S),Linux)
# CXXFLAGS += -stdlib=libstdc++ -lpthread
# else ifeq ($(UNAME_S),Darwin)
# CXXFLAGS += -stdlib=libc++
# endif

# g++-7 complains generates warnings in Eigen without the following:
CXXVER=$(shell $(CXX) --version)

CXXFLAGS += -Wno-int-in-bool-context
#LIBS += -latomic -lpthread

# Make sure to set the benchmarking flags BEFORE modifying CXXFLAGS to
# include the BUILD_TYPE-based flags.
CXXBENCHFLAGS = $(CXXFLAGS) -march=native -O3 -DNDEBUG

# Support for different types of test builds.  These BUILD_TYPE
# variable does NOT effect benchmark compilation.
BUILD_TYPE ?= release
ifeq ($(BUILD_TYPE),release)
CXXFLAGS += -march=native -O3 -DBUILD_TYPE_RELEASE
else ifeq ($(BUILD_TYPE),debug)
CXXFLAGS += -march=native -g  -DBUILD_TYPE_DEBUG
else ifeq ($(BUILD_TYPE),valgrind)
CXXFLAGS += -march=native -O0 -g -DBUILD_TYPE_VALGRIND
else
$(error Bad value for BUILD_TYPE.  Must be one of 'release', 'debug', or 'valgrind')
endif

# it would be preferable to do something like this, but gmake does not
# include a simple toupper solution.  So we include the
# -DBUILD_TYPE_XXX above.
#
# If we add GNU make standard library (http://gmsl.sourceforge.net/),
# then we can use the 'uc' function.
#
# CXXFLAGS += -DBUILD_TYPE_$(toupper $(BUILD_TYPE))

BUILD_ROOT = build/$(BUILD_TYPE)
BUILD_BENCH_ROOT = build/bench

# UNAME_S := $(shell uname -s)
# ifeq ($(UNAME_S),Linux)
# CXXFLAGS += -stdlib=libstdc++ -lpthread
# else # ifeq ($(UNAME_S),Darwin)
# CXXFLAGS += -stdlib=libc++
# endif

PKG_CONFIG ?= pkg-config

ifeq ($(shell $(PKG_CONFIG) --exists eigen3 && echo 1),1)
CXXFLAGS += $(shell $(PKG_CONFIG) --cflags eigen3)
endif

testsrc := $(wildcard *_test.cpp)
ctestsrc := $(wildcard *_ctest.cpp)
benchsrc := $(wildcard *_bench.cpp)

# Declare testrun to be all the tests to run.  These are simply the
# *_test.cpp files with the .cpp removed.  This allows us to type
#    make foo_test
# to run the test in foo_test.cpp
testrun := $(patsubst %.cpp,%,$(testsrc))
ctestrun := $(patsubst %.cpp,%,$(ctestsrc))
benchrun := $(patsubst %.cpp,%,$(benchsrc))

.PHONY: all clean test ctest bench $(testrun) $(ctestrun) $(benchrun)
all: test ctest bench

test: $(testrun)

ctest: $(ctestrun)

bench: $(benchrun)

clean:
	rm -rf build

# remove default rule
%: %.cpp

# Compile _test.cpp files directly to executable (no intermediate .o)
.PRECIOUS: $(patsubst %test.cpp,$(BUILD_ROOT)/bin/%test,$(testsrc))
$(BUILD_ROOT)/bin/%test: %test.cpp $(BUILD_ROOT)/dep/%test.d
	@echo "Compiling" $@
	@mkdir -p $(@D)
	@mkdir -p $(BUILD_ROOT)/dep
	$(CXX) -MMD -MF $(patsubst %test.cpp,$(BUILD_ROOT)/dep/%test.dtmp,$<) $(CXXFLAGS) -o $@ $< $(LIBS)
	@mv $(patsubst %test.cpp,$(BUILD_ROOT)/dep/%test.dtmp,$<) $(patsubst %test.cpp,$(BUILD_ROOT)/dep/%test.d,$<)
	@touch $@

.PRECIOUS: $(patsubst %_bench.cpp,$(BUILD_ROOT)/bin/%_bench,$(benchsrc))
$(BUILD_ROOT)/bin/%_bench: %_bench.cpp $(BUILD_ROOT)/dep/%_bench.d
	@echo "Compiling" $@
	@mkdir -p $(@D)
	@mkdir -p $(BUILD_ROOT)/dep
	@$(CXX) -MMD -MF $(patsubst %_bench.cpp,$(BUILD_ROOT)/dep/%_bench.dtmp,$<) $(CXXFLAGS) -DNDEBUG -o $@ $<
	@mv $(patsubst %_bench.cpp,$(BUILD_ROOT)/dep/%_bench.dtmp,$<) $(patsubst %_bench.cpp,$(BUILD_ROOT)/dep/%_bench.d,$<)
	@touch $@

$(BUILD_ROOT)/bin/old_%_bench: old_%_bench.cpp $(BUILD_ROOT)/dep/old_%_bench.d
	@echo "Compiling OLD" $@
	@mkdir -p $(@D)
	@mkdir -p $(BUILD_ROOT)/dep
	@$(CXX) -MMD -MF $(patsubst old_%_bench.cpp,$(BUILD_ROOT)/dep/old_%_bench.dtmp,$<) $(CXXFLAGS) -I../../nigh/src -DNDEBUG -o $@ $<
	@mv $(patsubst old_%_bench.cpp,$(BUILD_ROOT)/dep/old_%_bench.dtmp,$<) $(patsubst old_%_bench.cpp,$(BUILD_ROOT)/dep/old_%_bench.d,$<)
	@touch $@

# Runs a test.  On success it touches the .success file.  Either way it writes the .log file.
$(BUILD_ROOT)/log/%test.success: $(BUILD_ROOT)/bin/%test
	@echo "Running test" $<
	@mkdir -p $(@D)
	@set -o pipefail ; $< | tee $(BUILD_ROOT)/log/$*.log && touch $@


$(BUILD_ROOT)/bench/%.dat: $(BUILD_ROOT)/bin/%_bench
	@echo "Running benchmark" $<
	@mkdir -p $(@D)
	@set -o pipefail ; $< > $(BUILD_ROOT)/bench/$*.dat

$(testrun): %: $(BUILD_ROOT)/log/%.success

$(ctestrun): %: $(BUILD_ROOT)/log/%.success

$(benchrun): %_bench: $(BUILD_ROOT)/bench/%.dat

%.d: ;
.PRECIOUS: %.d

# Include generated dependencies
-include $(patsubst %.cpp,$(BUILD_ROOT)/dep/%.d,$(testsrc))
-include $(patsubst %.cpp,$(BUILD_ROOT)/dep/%.d,$(ctestsrc))
-include $(patsubst %.cpp,$(BUILD_ROOT)/dep/%.d,$(benchsrc))
-include $(wildcard $(BUILD_BENCH_ROOT)/dep/bench-*.d)
