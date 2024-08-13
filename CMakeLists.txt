cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(phoxi_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Adding RPATH settings
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "$ENV{PHOXI_CONTROL_PATH}/API/lib")

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    pcl_ros
    dynamic_reconfigure
    diagnostic_updater
    geometry_msgs
    eigen_conversions
    cv_bridge
    message_generation
)

catkin_python_setup()



include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PHOXI_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_message_files(
  FILES
    PhoXiSize.msg
    DeviceConnectionStatus.msg
    DeviceType.msg
    DeviceInformation.msg
)

add_service_files(
  FILES
    Deprecated/Empty.srv        #deprecated
    Deprecated/IsConnected.srv  #deprecated
    Deprecated/IsAcquiring.srv  #deprecated
    ConnectCamera.srv
    TriggerImage.srv
    GetDeviceList.srv
    GetBool.srv
    GetFrame.srv
    GetHardwareIdentification.srv
    GetSupportedCapturingModes.srv
    SaveFrame.srv
    SaveLastFrame.srv
    SetCoordinatesSpace.srv
    SetTransformationMatrix.srv
    GetString.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/phoxi_camera.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
    ${PHOXI_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_PhoXi_Interface
    ${PROJECT_NAME}_Ros_Interface
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    sensor_msgs
    pcl_ros
    dynamic_reconfigure
    diagnostic_updater
    geometry_msgs
    cv_bridge
    eigen_conversions
  DEPENDS
    PhoXi
)

add_compile_options(-w)
add_compile_options(-fpermissive)
add_compile_options(-pthread)
add_compile_options(-O2)

# add PhoXi version definitions to code
add_definitions(-DPHOXI_API_VERSION_MAJOR=${PHO_SOFTWARE_VERSION_MAJOR})
add_definitions(-DPHOXI_API_VERSION_MINOR=${PHO_SOFTWARE_VERSION_MINOR})
add_definitions(-DPHOXI_API_VERSION_PATCH=${PHO_SOFTWARE_VERSION_PATCH})

add_library(
  ${PROJECT_NAME}_PhoXi_Interface
  src/PhoXiInterface.cpp
)

add_library(
  ${PROJECT_NAME}_Ros_Interface
  src/RosInterface.cpp
)

add_executable(
  ${PROJECT_NAME}
  src/phoxi_camera_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}_Ros_Interface
  ${PROJECT_NAME}_PhoXi_Interface
  ${PROJECT_NAME}_gencfg
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_Ros_Interface
)

target_link_libraries(
  ${PROJECT_NAME}_PhoXi_Interface
  ${PHOXI_LIB_DEBUG}  # We don't use ${PHOXI_LIBRARY} because we have crashing issues in docker.
  rt
  pthread
)
target_link_libraries(
  ${PROJECT_NAME}_Ros_Interface
  ${PROJECT_NAME}_PhoXi_Interface
  ${catkin_LIBRARIES}
)

target_link_libraries( ${PROJECT_NAME}
    PRIVATE
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_Ros_Interface
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_Ros_Interface ${PROJECT_NAME}_PhoXi_Interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/launch/phoxi_camera_ros_interfaces_exist.test)
    add_rostest(test/launch/phoxi_camera_ros_interfaces.test)

    add_rostest_gtest(${PROJECT_NAME}_unittest
            test/launch/phoxi_api_interface_class.test
            test/gtest/test_phoxi_api_interface.cpp)

    target_link_libraries(${PROJECT_NAME}_unittest
            ${catkin_LIBRARIES}
            ${PROJECT_NAME}_Ros_Interface)
endif()
