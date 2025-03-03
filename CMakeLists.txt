cmake_minimum_required(VERSION 3.16)

project(physx_play)

set(root_dir ${CMAKE_CURRENT_SOURCE_DIR})
set(bin_dir ${root_dir}/bin)

set(physx_root /opt/PhysX)

include_directories(${physx_root}/include)
link_directories(${physx_root}/bin/linux.x86_64/debug)

file(GLOB physx_static_libs "${physx_root}/bin/linux.x86_64/debug/*_static_64")
file(GLOB physx_dyn_libs "${physx_root}/bin/linux.x86_64/debug/*.so")
set(lib_sys pthread)
set(lib_physx PhysXPvdSDK_static_64 PhysXExtensions_static_64 PhysXCooking_static_64 PhysX_static_64
    PhysXPvdSDK_static_64 PhysXCommon_static_64 PhysXFoundation_static_64 PVDRuntime_64)

message(STATUS ${physx_static_libs})

add_subdirectory(${root_dir}/perpendicular_gear)

file(COPY ${physx_dyn_libs} DESTINATION ${bin_dir})