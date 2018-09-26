#!/bin/bash

declare -a tests
for file in *_test.cpp ; do
    tests+=(${file%.*})
done

declare -a ctests
for file in *_ctest.cpp ; do
    ctests+=(${file%.*})
done

declare -a benches
for file in *_bench.cpp ; do
    benches+=(${file%.*})
done

CFLAGS="-std=c++17 -I../src -I/usr/include/eigen3 -Wall -pedantic -Wno-ignored-attributes -march=native -O3"

CXX=clang++
LIBS=-lpthread

#CXX=g++
#CFLAGS="$CFLAGS -Wno-int-in-bool-context"
#LIBS="-lpthread -latomic"

cat <<EOF
ninja_required_version = 1.3
root = .
builddir = build
cxx = $CXX
cflags = $CFLAGS
libs = $LIBS

pool concurrent_pool
  depth = 1

rule cxx
  command = \$cxx -MMD -MT \$out -MF \$out.d \$cflags \$ndebug \$in -o \$out \$libs
  description = CXX \$out
  depfile = \$out.d
  deps = gcc

rule test
  command = \$in > \$in.log && touch \$out
  description = TEST \$in

rule ctest
  command = \$in > \$in.log && touch \$out
  description = CONCURRENT TEST \$in
  pool = concurrent_pool

rule bench
  command = \$in > \$out
  description = BENCH \$in
  pool = concurrent_pool

rule plot
  command = ./plot.sh \$out \$title \$in
  description = PLOT \$out

EOF

for test in ${tests[@]} ; do
    echo "build \$builddir/$test: cxx $test.cpp"
    echo "build \$builddir/$test.success: test \$builddir/$test"
    echo "build $test: phony \$builddir/$test.success"
done

for test in ${ctests[@]} ; do
    echo "build \$builddir/$test: cxx $test.cpp"
    echo "build \$builddir/$test.success: ctest \$builddir/$test"
    echo "build $test: phony \$builddir/$test.success"
done

for bench in ${benches[@]} ; do
    echo "build \$builddir/$bench: cxx $bench.cpp"
    echo "  ndebug = -DNDEBUG"
    echo "build \$builddir/$bench.dat: bench \$builddir/$bench"
    echo "build $bench: phony \$builddir/$bench.dat"
done

declare -a plots
for plot in l2_3 so3 so2_5 se3 ; do
    echo -n "build ${plot}.svg: plot"
    plots+=("${plot}.svg")
    state="eigenmatrix";
    case $plot in
        se3) state="tuple" ;;
        so3) state="eigenquaternion" ;;
    esac
    for strategy in batch_8 linear gnat ; do
        echo -n " \$builddir/generated_${plot}_double_${state}_${strategy}_bench.dat"
    done
    echo ""
    echo -n "  title = "
    case $plot in
        l2_3) echo "'R^3 with L^2'" ;;
        so3) echo "'SO(3)'" ;;
        so2_5) echo "'SO(2)x5'" ;;
        se3) echo "'SE(3) 1:1'" ;;
    esac
done

cat <<EOF

build plots: phony ${plots[@]}
build all: phony ${tests[@]} ${ctests[@]} plots

EOF



