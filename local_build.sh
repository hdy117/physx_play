#!/bin/bash

root_dir=$(pwd)
build_dir=$root_dir/build

mkdir -p $build_dir

cd $build_dir && cmake .. && make clean && make -j32 && make install

cd $root_dir