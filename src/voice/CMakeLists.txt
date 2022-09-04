cmake_minimum_required(VERSION 3.8)
project(voice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

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


set(dependencies
  rclcpp
  std_msgs
)

add_executable(iat_publish src/iat_publish.cpp src/speech_recognizer.c src/linuxrec.c)
ament_target_dependencies(iat_publish ${dependencies})

add_executable(tts_subscribe src/tts_subscribe.cpp)
ament_target_dependencies(tts_subscribe ${dependencies})

add_executable(voice_assistant src/voice_assistant.cpp)
ament_target_dependencies(voice_assistant ${dependencies})

target_link_libraries(iat_publish
  ${PROJECT_SOURCE_DIR}
  libmsc.so -ldl -lpthread -lm -lrt -lasound
)

target_link_libraries(tts_subscribe
  ${PROJECT_SOURCE_DIR}
  libmsc.so -ldl -pthread 
)

target_link_libraries(
   voice_assistant
   ${PROJECT_SOURCE_DIR}
   libmsc.so -ldl -pthread
)

install(TARGETS
  iat_publish
  tts_subscribe
  voice_assistant
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)
