cmake_minimum_required(VERSION 3.8)
project(quadrotor_simulator_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
        cmake/CheckAvxSupport.cmake
        cmake/FindOrFetch.cmake
        cmake/MujocoHarden.cmake
        cmake/MujocoLinkOptions.cmake
        cmake/MujocoMacOS.cmake
        cmake/SampleDependencies.cmake
        cmake/SampleOptions.cmake
        )

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Mujoco location
if(NOT TARGET mujoco)
  add_library(mujoco SHARED IMPORTED)
  set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libmujoco.so)
endif()

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
# Include location of the executables files
file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)

# Add executable
add_executable(quadrotor_simulator ${source})
# Include libraries
target_link_libraries(quadrotor_simulator mujoco glfw pthread ${OpenCV_LIBRARIES} rt ${CMAKE_DL_LIBS})

# Include directories
target_include_directories(quadrotor_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 
target_compile_options(quadrotor_simulator PUBLIC ${DEEPBREAK_CXX_FLAGS})  
target_compile_features(quadrotor_simulator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  quadrotor_simulator
  "ament_cmake"
  "sensor_msgs"
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
)

install(TARGETS quadrotor_simulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
         DESTINATION share/${PROJECT_NAME})

install(DIRECTORY model
DESTINATION share/${PROJECT_NAME})

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
