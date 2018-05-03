project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp)



if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning 
	${sources}
	src/planner/utils/spline.cpp
	src/planner/data_type/poly.cpp
	src/planner/data_type/planning_data_type.cpp
	src/planner/data_type/poly_candidate.cpp
	src/planner/module/start_selector.cpp
	src/planner/module/candidate_builder.cpp
	src/planner/module/optimal_selector.cpp
	src/planner/module/object_manager.cpp
	src/planner/poly_planner.cpp)


target_link_libraries(path_planning z ssl uv uWS)
