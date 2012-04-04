
#SEE CMake-QT tutorial:   http://qt-project.org/quarterly/view/using_cmake_to_build_qt_projects

if (CMAKE_COMPILER_IS_GNUCXX)
	set (CMAKE_CXX_FLAGS "-std=c++0x")
endif ()

FIND_PACKAGE (Qt4 REQUIRED)

#only include headers that need to be processed by MOC
SET (VisualGraph_HEADERS visualgraph.h graphscene.h)
SET (VisualGraph_SOURCES main.cpp visualgraph.cpp guivertex.cpp guiedge.cpp graphscene.cpp)
#SET (VisualGraph_FORMS form.ui) #NOT USING FORMS!
SET (VisualGraph_RESOURCES visualgraph.qrc)

#Run the QT meta object compiler (moc) on the headers that contain Q_OBJECT macro
QT4_WRAP_CPP (VisualGraph_HEADERS_MOC ${VisualGraph_HEADERS})
#QT4_WRAP_UI(helloworld_FORMS_HEADERS ${helloworld_FORMS})
QT4_ADD_RESOURCES (VisualGraph_RESOURCES_RCC ${VisualGraph_RESOURCES})

INCLUDE (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE (VisualGraph ${VisualGraph_SOURCES} ${VisualGraph_HEADERS_MOC} ${VisualGraph_RESOURCES_RCC})
TARGET_LINK_LIBRARIES (VisualGraph ${QT_LIBRARIES})


#TO ADD MORE QT MODULES add these lines
#SET(QT_USE_QTOPENGL TRUE)