
if (CMAKE_COMPILER_IS_GNUCXX)
	set (CMAKE_CXX_FLAGS "-std=c++0x")
endif ()


set (Boost_USE_STATIC_LIBS ON)
set (Boost_USE_MULTITHREADED ON)
set (Boost_USE_STATIC_RUNTIME OFF)
find_package (Boost COMPONENTS date_time REQUIRED)

if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif ()

set (TEST_CLASSES astaralgorithmtester.cpp dijkstraalgorithmtester.cpp mstalgorithmstester.cpp)

add_executable (GraphTest main.cpp ${TEST_CLASSES})

if (Boost_FOUND)
    target_link_libraries(GraphTest ${Boost_LIBRARIES})
endif ()