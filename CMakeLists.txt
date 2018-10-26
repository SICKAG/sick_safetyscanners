cmake_minimum_required(VERSION 2.8.3)
project(sick_microscan3_ros_driver)

## Use C++11
#add_definitions(-std=c++11)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-std=c++11 -Wall)
#add_definitions(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roslint
    roscpp
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    message_generation
)

## Find system libraries
#find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(
  config/Microscan3Configuration.cfg
)


add_message_files(
  FILES
  ExtendedLaserScanMsg.msg
  RawMicroScanDataMsg.msg
  DataHeaderMsg.msg
  DerivedValuesMsg.msg
  GeneralSystemStateMsg.msg
  IntrusionDataMsg.msg
  IntrusionDatumMsg.msg
  MeasurementDataMsg.msg
  ScanPointMsg.msg
  ApplicationDataMsg.msg
  ApplicationInputsMsg.msg
  ApplicationOutputsMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    message_runtime
  DEPENDS
    Boost
)



###########
## Build ##
###########

roslint_cpp()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
  src/Microscan3.cpp
  src/communication/AsyncUDPClient.cpp
  src/communication/AsyncTCPClient.cpp
  src/datastructure/PacketBuffer.cpp
  src/datastructure/DatagramHeader.cpp
  src/datastructure/Data.cpp
  src/datastructure/DataHeader.cpp
  src/datastructure/GeneralSystemState.cpp
  src/datastructure/DerivedValues.cpp
  src/datastructure/MeasurementData.cpp
  src/datastructure/ScanPoint.cpp
  src/datastructure/IntrusionData.cpp
  src/datastructure/IntrusionDatum.cpp
  src/datastructure/ApplicationData.cpp
  src/datastructure/ApplicationInputs.cpp
  src/datastructure/ApplicationOutputs.cpp
  src/datastructure/CommSettings.cpp
  src/datastructure/TypeCode.cpp
  src/datastructure/FieldData.cpp
  src/data_processing/UDPPacketMerger.cpp
  src/data_processing/ParseDatagramHeader.cpp
  src/data_processing/ParseData.cpp
  src/data_processing/ParseDataHeader.cpp
  src/data_processing/ParseDerivedValues.cpp
  src/data_processing/ParseMeasurementData.cpp
  src/data_processing/ParseGeneralSystemState.cpp
  src/data_processing/ParseIntrusionData.cpp
  src/data_processing/ParseApplicationData.cpp
  src/data_processing/ReadWriteHelper.cpp
  src/data_processing/TCPPacketMerger.cpp
  src/data_processing/ParseTCPPacket.cpp
  src/data_processing/ParseTypeCodeData.cpp
  src/data_processing/ParseFieldHeaderData.cpp
  src/data_processing/ParseFieldGeometryData.cpp
  src/data_processing/ParseDeviceName.cpp
  src/cola2/Cola2Session.cpp
  src/cola2/Command.cpp
  src/cola2/CreateSession.cpp
  src/cola2/CloseSession.cpp
  src/cola2/MethodCommand.cpp
  src/cola2/ChangeCommSettingsCommand.cpp
  src/cola2/VariableCommand.cpp
  src/cola2/TypeCodeVariableCommand.cpp
  src/cola2/FieldHeaderVariableCommand.cpp
  src/cola2/FieldGeometryVariableCommand.cpp
  src/cola2/MonitoringCaseTableHeaderVariableCommand.cpp
  src/cola2/DeviceNameVariableCommand.cpp
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  nodes/${PROJECT_NAME}_node.cpp
  src/Microscan3Ros.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(
  DIRECTORY docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test
#  test/test_ros_package_template.cpp
#  test/AlgorithmTest.cpp)
#endif()

#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#  endif()
