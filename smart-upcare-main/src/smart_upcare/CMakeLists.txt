cmake_minimum_required(VERSION 3.8)
project(smart_upcare)

find_package(ament_cmake REQUIRED)

# urdf, launch, worlds, rviz 폴더를 설치 경로로 복사
# (이게 있어야 ros2 launch가 파일을 찾음)
install(
  DIRECTORY urdf launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
