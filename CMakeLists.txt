cmake_minimum_required(VERSION 3.1.0)
project(picnic)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Widgets finds its own depndencies.
add_library(${PROJECT_NAME} SHARED
	    src/module.c
	    src/module_funcs.c
	    src/picnic_device.c
)

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

add_definitions(-DRTAPI -DCONNECTION_DEVICE)
include_directories(${PROJECT_NAME} include /usr/include/linuxcnc)

message("Soruces root is ${CMAKE_SOURCE_DIR}")
