#!/bin/bash

root_dir=$(pwd)
build_dir=$root_dir/build

rm -rf $build_dir
mkdir -p $build_dir

cd $build_dir && cmake .. && make clean && make -j32 && make install

cd $root_dir