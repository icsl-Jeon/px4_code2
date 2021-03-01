cmake_minimum_required(VERSION 3.0.2)
project(px4_code2)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS -fPIC)
## Include qpOASES
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
find_package(qpOASES REQUIRED)
message( "cmake module path : ${CMAKE_MODULE_PATH}")

## ROS
find_package(catkin REQUIRED COMPONENTS
        mavros roscpp geometry_msgs message_generation tf nav_msgs rqt_gui rqt_gui_cpp
        px4_code2_msgs
)




if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    find_package(Qt5Widgets REQUIRED)
else()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
endif()

set(CMAKE_AUTORCC true) # to include qt resource collection
set(CMAKE_INCLUDE_CURRENT_DIR ON) # This finds ui_*.h







catkin_package(

)

## Client QT and RQT
set(QT_SRC resource/gcs_widget/widget.cpp resource/gcs_rqt/gcs_plugin.cpp)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    qt5_wrap_cpp(QT_MOCS resource/gcs_widget/widget.h resource/gcs_rqt/gcs_plugin.h include/traj_gen/TrajGen.hpp)
    qt5_wrap_ui(QT_UIS_H resource/gcs_widget/widget.ui)
    qt5_add_resources(resource/gcs_widget/img.qrc)
else()
    qt4_wrap_cpp(QT_MOCS resource/gcs_widget/widget.h resource/gcs_rqt/gcs_plugin.h include/traj_gen/TrajGen.hpp)
    qt4_wrap_ui(QT_UIS_H resource/gcs_widget/widget.ui)
    qt4_add_resources(resource/gcs_widget/img.qrc)
endif()

include_directories(
 include resource
  ${catkin_INCLUDE_DIRS}

)


#########################
##       CLIENT        ##
#########################
message("${RQT_SRC}")
add_library(${PROJECT_NAME}  ${QT_MOCS} ${QT_SRC} ${QT_UIS_H} src/server.cpp src/utils/utility.cpp  resource/gcs_widget/img.qrc include/traj_gen/TrajGen.hpp)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${catkin_LIBRARIES} ${qpOASES_LIB})
else()
    target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${catkin_LIBRARIES} ${qpOASES_LIB})
endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#########################
##       SERVER        ##
#########################

add_executable(server_node src/exe/server_node.cpp src/server.cpp src/utils/utility.cpp )
target_link_libraries(server_node ${catkin_LIBRARIES} ${qpOASES_LIB})

