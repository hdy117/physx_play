cmake_minimum_required(VERSION 3.16)

set(bin_name perpendicular_gear)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} src_files)

add_executable(${bin_name} ${src_files})
# target_link_libraries(${bin_name} ${physx_dyn_libs} ${lib_sys} ${physx_static_libs})
target_link_libraries(${bin_name} ${lib_physx} ${lib_sys})

install(TARGETS ${bin_name} RUNTIME DESTINATION ${bin_dir}) 