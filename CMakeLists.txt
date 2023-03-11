cmake_minimum_required(VERSION 3.26.0-rc3)  # 사용가능한 cmake 최소버전 정의

project(linetracer)  # 실행파일명

find_package(OpenCV REQUIRED)   # Opencv의 헤더파일경로, 라이브러리파일을 자동으로 검색하여 각각 변수                     

                                                             # OpenCV_INCLUDE_DIRS, OpenCV_LIBS에 저장한다.
include_directories(${OpenCV_INCLUDE_DIRS})  # 헤더파일경로 추가

add_executable(linetracer src/main.cpp src/dxl.cpp src/dxl.hpp)  # 실행파일명과 소스파일명을 지정

target_link_libraries(linetracer dxl_x64_cpp)  # 라이브러리파일명을 직접지정, 경로는 설치시 시스템환경변수에 저장됨

target_link_libraries(linetracer ${OpenCV_LIBS})   # 실행파일명과 링크할 라이브러리 파일 추가

target_include_directories(linetracer PRIVATE /usr/local/include/dynamixel_sdk_cpp) # 헤더파일의 경로를 직접지정
