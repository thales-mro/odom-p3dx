cmake_minimum_required(VERSION 2.8)
project(Main)
# Generic
#set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")
add_definitions(-std=c++11)
add_definitions(-fmax-errors=2)
add_definitions(-O3)

option(DO_NOT_USE_SHARED_MEMORY ON)
#add_definitions(-g)
#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)


# V-REP
set(VREP_DIR "/home/thales/V-REP_PRO_EDU_V3_5_0_Linux/programming")
include_directories("${VREP_DIR}/remoteApi"
					"${VREP_DIR}/include"
					"${VREP_DIR}/common")

add_library(VREP
	"${VREP_DIR}/remoteApi/extApi.c"
	"${VREP_DIR}/remoteApi/extApiPlatform.c")
target_compile_definitions(VREP
	PRIVATE MAX_EXT_API_CONNECTIONS=255
	PRIVATE NON_MATLAB_PARSING)

# Program
add_library(SMTR Simulator.cpp)
target_link_libraries(SMTR VREP)

add_library(RBOT Robot.cpp)
target_link_libraries(RBOT SMTR)

add_executable(main Main.cpp)
target_link_libraries(main pthread RBOT -lssl -lcrypto -ldl -lboost_iostreams -lboost_system -lboost_filesystem)

#target_compile_definitions(RBOT PRIVATE DEBUG_BUILD=1)
#target_compile_definitions(Main PRIVATE DEBUG_BUILD=1)

