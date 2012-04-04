cmake_minimum_required (VERSION 2.6)
project (Graph)

include_directories ("${PROJECT_SOURCE_DIRECTORY}/graph")
add_subdirectory (graph)
add_subdirectory (graphtest)
add_subdirectory (visualgraph)