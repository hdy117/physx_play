cmake_minimum_required(VERSION 3.16)

project(physx_play)

set(root_dir ${CMAKE_CURRENT_SOURCE_DIR})
set(bin_dir ${root_dir}/bin)

set(physx_root /opt/PhysX)

include_directories(${physx_root}/include)
link_directories(${physx_root}/linux.x86_64/debug)

file(GLOB physx_static_libs "${physx_root})/*.a")
file(GLOB physx_dyn_libs "${physx_root})/*.so")
set(lib_sys pthread)

add_subdirectory(${root_dir}/perpendicular_gear)