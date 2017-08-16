cmake_minimum_required( VERSION 2.8 )
project( change )

set( CMAKE_BUILD_TYPE Debug)
#set( CMAKE_BUILD_TYPE Release )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )


find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( PCL 1.7 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib )

include_directories("/usr/include/eigen3")

set( build_libs
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

add_executable( main main.cpp )
target_link_libraries( main
    ${build_libs} 
)

