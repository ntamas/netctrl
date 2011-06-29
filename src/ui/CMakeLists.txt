add_executable(netctrl main
                       cmd_arguments
                       graph_util)
message(${igraph_LIBRARIES})
target_link_libraries(netctrl netctrl0 igraphpp ${igraph_LIBRARIES})

install(TARGETS netctrl
        RUNTIME DESTINATION bin)
