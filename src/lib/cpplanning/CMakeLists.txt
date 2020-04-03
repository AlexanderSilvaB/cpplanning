include_directories(..)

find_package( OpenCV REQUIRED )

set(SRC cpplanning.cpp CPPAlgorithm.cpp)

add_library(cpplanning ${SRC})

target_link_libraries( cpplanning ${OpenCV_LIBS} )