cmake_minimum_required(VERSION 3.0.2)
project(px4_code2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        mavros roscpp geometry_msgs message_generation tf nav_msgs rqt_gui rqt_gui_cpp
)



if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    find_package(Qt5Widgets REQUIRED)
else()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON) # This finds ui_*.h

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Takeoff.srv
 )

generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
        mavros_msgs
        nav_msgs
        )


catkin_package(




)
set(QT_SRC resource/gcs_widget/widget.cpp)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    qt5_wrap_cpp(QT_MOCS resource/gcs_widget/widget.h)
    qt5_wrap_ui(QT_UIS_H resource/gcs_widget/widget.ui)
else()
    qt4_wrap_cpp(QT_MOCS resource/gcs_widget/widget.h)
    qt4_wrap_ui(QT_UIS_H resource/gcs_widget/widget.ui)
endif()

include_directories(
 include resource
  ${catkin_INCLUDE_DIRS}

)
set(RQT_SRC resource/gcs_rqt/gcs_plugin.cpp)
set(RQT_H resource/gcs_rqt/gcs_plugin.h)


#########################
##       CLIENT        ##
#########################
message("${RQT_SRC}")
add_library(${PROJECT_NAME} ${RQT_SRC} ${RQT_H} ${QT_MOCS} ${QT_SRC} ${QT_UIS_H})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${catkin_LIBRARIES})
else()
    target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${catkin_LIBRARIES})
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

add_executable(server_node src/exe/server_node.cpp src/server.cpp src/utils/utility.cpp)
target_link_libraries(server_node ${catkin_LIBRARIES})

