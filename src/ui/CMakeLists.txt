add_executable(netctrl main.cpp
                       cmd_arguments.cpp
                       graph_util.cpp)
target_link_libraries(netctrl netctrl0 igraphpp ${igraph_LIBRARIES})

install(TARGETS netctrl
        RUNTIME DESTINATION bin)
