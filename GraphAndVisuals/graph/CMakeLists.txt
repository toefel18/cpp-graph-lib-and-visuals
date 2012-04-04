set (Graph_VERSION_MAJOR 1)
set (Graph_VERSION_MINOR 0)

configure_file ( "${PROJECT_SOURCE_DIR}/graph/graphconfig.h.in"
		 "${PROJECT_BINARY_DIR}/graphconfig.h")

include_directories ("${PROJECT_BINARY_DIR}")

set (CMAKE_CXX_FLAGS "-std=c++0x")

set (GRAPH_LIBRARY_HEADERS adjacencylist.h dijkstraalgorithm.h kruskalalgorithm.h primalgorithm.h weightcomputer.h astaralgorithm.h edge.h heuristic.h vertex.h graphconfig.h)

install (FILES ${GRAPH_LIBRARY_HEADERS} DESTINATION include)
