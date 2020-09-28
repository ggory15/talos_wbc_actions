cmake_minimum_required(VERSION 2.8.3)
project(talos_wbc_actions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED pal_locomotion pluginlib)
find_package(Armadillo REQUIRED) 

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS pal_locomotion
    )

include_directories(
    include
    )
include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    )

add_subdirectory(src/actions)
add_subdirectory(src/nodes)

add_library(${PROJECT_NAME} src/icp_control_utils.cpp
                            ${SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    )

install(FILES biped_walking_actions.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
