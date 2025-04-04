cmake_minimum_required(VERSION 3.8)
project(navigation_utilities_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Add include directories
include_directories(include/navigation_utilities_package
        ${EIGEN3_INCLUDE_DIR})

# Add the library
add_library(${PROJECT_NAME} SHARED
  src/navigation_utilities.cpp
)

# Link the library with Eigen3 (if needed)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
 
# Install the library and headers
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips uncrustify (source code formatting)
  # set(ament_cmake_uncrustify_FOUND TRUE)

  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FILES
      test/main.cpp
      test/navigation_utilities_test.cpp)

  ament_add_gtest(${PROJECT_NAME}_test ${TEST_FILES})

  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  install(TARGETS
          ${PROJECT_NAME}_test
          DESTINATION lib/${PROJECT_NAME})

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify)

  # enforce linters and static code analyzers defined in ament_lint_common package
  ament_lint_auto_find_test_dependencies()

  # uncomment to include uncrustify explicitly
  # find_package(ament_cmake_uncrustify)
  # ament_uncrustify(CONFIG_FILE "./uncrustify.cfg" TESTNAME "custom_uncrustify")

endif()

# Export the package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
