add_executable(netctrl main
                       cmd_arguments
                       graph_util
					   vertex_set_spec)
target_link_libraries(netctrl netctrl0 igraphpp ${igraph_LIBRARIES})

install(TARGETS netctrl
        RUNTIME DESTINATION bin)
