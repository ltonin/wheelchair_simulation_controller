set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_simulation_controller)
find_package(catkin REQUIRED COMPONENTS roscpp 
									    std_msgs
										geometry_msgs
										nav_msgs
										tf)


SET(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(PkgConfig)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
	geometry_msgs
	nav_msgs
	tf
  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS} 
					include
)


add_library(${PROJECT_NAME} 
			src/GoalGenerator.cpp
			src/CommandGenerator.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

add_executable(goalgenerator src/goalgenerator.cpp)
target_link_libraries(goalgenerator ${PROJECT_NAME})

add_executable(commandgenerator src/commandgenerator.cpp)
target_link_libraries(commandgenerator ${PROJECT_NAME})


