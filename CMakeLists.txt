project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)
add_definitions(-W -Wno-int-in-bool-context -Wno-unused-parameter -Wreturn-type -Wsign-compare -Wswitch -Wunused-variable -Wunused-but-set-variable -Wunused-function -Wall -Wextra -pedantic)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include_directories(gnuplot-iostream)

set(sources src/main.cpp src/log.cpp src/Map.cpp src/BehaviorPlanner.cpp src/TrajectoryPlanner.cpp src/JMT.cpp src/TrajectoryCost.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set(ANALYZEMAPONLY OFF)
set(PLOTSIGNALS OFF)
set(PLOTMAP OFF)

if(${PLOTSIGNALS})
add_definitions(-DPLOTSIGNALS)
endif(${PLOTSIGNALS})
if(${PLOTMAP})
add_definitions(-DPLOTMAP)
endif(${PLOTMAP})
if(${ANALYZEMAPONLY})
add_definitions(-DANALYZEMAPONLY)
endif(${ANALYZEMAPONLY})

add_executable(path_planning ${sources})

if(${PLOTSIGNALS} OR ${PLOTMAP} OR ${ANALYZEMAPONLY})
target_link_libraries(path_planning z ssl uv uWS boost_iostreams boost_system boost_filesystem)
else(${PLOTSIGNALS} OR ${PLOTMAP} OR ${ANALYZEMAPONLY})
target_link_libraries(path_planning z ssl uv uWS)
endif(${PLOTSIGNALS} OR ${PLOTMAP} OR ${ANALYZEMAPONLY})
