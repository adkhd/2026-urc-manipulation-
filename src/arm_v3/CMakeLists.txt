cmake_minimum_required(VERSION 3.5)
project(arm_v3)

# 빌드 시스템 패키지 불러오기
find_package(ament_cmake REQUIRED)

# 설치 규칙: ROS 2는 패키지의 리소스들을 share 디렉토리에 설치해야 함
install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# ROS 2 패키지로 등록
ament_package()