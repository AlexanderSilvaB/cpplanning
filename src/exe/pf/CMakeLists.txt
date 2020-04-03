include_directories(../../lib)

set(SRC main.cpp)

add_executable(pf ${SRC})
target_link_libraries(pf  cpplanning)
set_target_properties(pf PROPERTIES OUTPUT_NAME pf)