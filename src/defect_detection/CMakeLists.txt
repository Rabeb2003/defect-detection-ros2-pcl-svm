cmake_minimum_required(VERSION 3.8)
project(defect_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters features registration segmentation ml)
find_package(OpenCV REQUIRED)

add_executable(defect_detector 
  src/defect_detector.cpp
  src/modules/point_cloud_acquisition.cpp
  src/modules/cad_registration.cpp
  src/modules/defect_segmentation.cpp
  src/modules/defect_classifier.cpp
)

target_include_directories(defect_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(defect_detector
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

target_compile_features(defect_detector PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  defect_detector
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
  "std_msgs"
  "geometry_msgs"
  "visualization_msgs"
)

# Cloud publisher for testing
add_executable(cloud_publisher src/cloud_publisher.cpp)
target_include_directories(cloud_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(cloud_publisher
  ${PCL_LIBRARIES}
)
target_compile_features(cloud_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  cloud_publisher
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
)

# Training data generator
add_executable(generate_training_data src/generate_training_data.cpp)
target_include_directories(generate_training_data PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(generate_training_data
  ${PCL_LIBRARIES}
)
target_compile_features(generate_training_data PUBLIC c_std_99 cxx_std_17)

# SVM classifier trainer
add_executable(train_svm_classifier src/train_svm_classifier.cpp)
target_include_directories(train_svm_classifier PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(train_svm_classifier
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
target_compile_features(train_svm_classifier PUBLIC c_std_99 cxx_std_17)

# PCD to ASCII converter
add_executable(convert_pcd_to_ascii src/convert_pcd_to_ascii.cpp)
target_include_directories(convert_pcd_to_ascii PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(convert_pcd_to_ascii
  ${PCL_LIBRARIES}
)
target_compile_features(convert_pcd_to_ascii PUBLIC c_std_99 cxx_std_17)

# 3D Defect Visualizer
add_executable(visualize_defects src/visualize_defects.cpp)
target_include_directories(visualize_defects PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(visualize_defects
  ${PCL_LIBRARIES}
)
target_compile_features(visualize_defects PUBLIC c_std_99 cxx_std_17)

# 3D Visualizer with Screenshot
add_executable(visualize_and_save src/visualize_and_save.cpp)
target_include_directories(visualize_and_save PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(visualize_and_save
  ${PCL_LIBRARIES}
)
target_compile_features(visualize_and_save PUBLIC c_std_99 cxx_std_17)

install(TARGETS defect_detector cloud_publisher generate_training_data train_svm_classifier convert_pcd_to_ascii visualize_defects visualize_and_save
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  PATTERN ".svn" EXCLUDE)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  PATTERN ".svn" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
