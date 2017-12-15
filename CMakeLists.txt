CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

PROJECT(nvmtools)

ADD_DEFINITIONS(
	-std=c++11
	)

find_package(JPEG REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Gflags REQUIRED)
FIND_PACKAGE(glog 0.3.5 REQUIRED)

ADD_SUBDIRECTORY(thirdLibs/stlplus3)

SET(NVM_INCS
	${PROJECT_SOURCE_DIR}/src
	${JPEG_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${GFLAGS_INCLUDE_DIRS}
	${GLOG_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/thirdLibs
	${PROJECT_SOURCE_DIR}/thirdLibs/cimg
	)

SET(NVM_LIBS
	nvmtools 
	stlplus3_nvmtools 
	${JPEG_LIBRARIES}
	${GFLAGS_LIBRARIES} 
	glog::glog
	)

INCLUDE_DIRECTORIES(${NVM_INCS})

add_library(nvmtools STATIC 
	src/nvmtools/NVMFile.cpp
	src/nvmtools/NVMFileVersion.cpp
	src/nvmtools/CameraModel.cpp
	src/nvmtools/NVMCameraModel.cpp
	src/nvmtools/PinholeModel.cpp
	src/nvmtools/UndistortModel.cpp
	)
	
# the binary
ADD_EXECUTABLE(undistort_nvm src/undistort_nvm.cpp)
TARGET_LINK_LIBRARIES(undistort_nvm ${NVM_LIBS} )

SET(NVMTOOLS_INCLUDE_DIR ${NVM_INCS} PARENT_SCOPE)
SET(NVMTOOLS_LIBRARIES ${NVM_LIBS} PARENT_SCOPE) 
