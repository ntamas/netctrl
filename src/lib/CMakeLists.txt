add_library(netctrl0 STATIC model/controllability.cpp
	                        model/liu.cpp
                            model/switchboard.cpp
							util/directed_matching.cpp
)
target_include_directories(
	netctrl0 PRIVATE
    $<TARGET_PROPERTY:igraph::igraph,INTERFACE_INCLUDE_DIRECTORIES>
)

