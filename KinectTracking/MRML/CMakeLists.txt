project(vtkSlicer${MODULE_NAME}ModuleMRML)

set(KIT "${PROJECT_NAME}")

set(${KIT}_EXPORT_DIRECTIVE "VTK_SLICER_KINECTTRACKING_MODULE_MRML_EXPORT")
FIND_LIBRARY(OpenIGTLinkLib OpenIGTLink /Users/longquanchen/Desktop/Github/Slicer-build/OpenIGTLink-build/bin/Debug)
FIND_LIBRARY(OpenH264Lib openh264 /Users/longquanchen/Desktop/Github/OpenIGTLink-xcodeBuild/Testing/OpenH264)
FIND_LIBRARY(lib openh264 /Users/longquanchen/Desktop/Github/OpenIGTLink-xcodeBuild/Testing/OpenH264)
FIND_LIBRARY(AVCODEC avcodec /Users/longquanchen/Downloads/libav-10.7/libavcodec)
FIND_LIBRARY(AVUTIL avutil /Users/longquanchen/Downloads/libav-10.7/libavutil)
set(H264Directory /Users/longquanchen/Desktop/Github/OpenIGTLink-xcodeBuild/Testing/OpenH264)

set(${KIT}_INCLUDE_DIRECTORIES
  ${Slicer_Base_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../Logic
  ${CMAKE_CURRENT_BINARY_DIR}/../Logic
  ${H264Directory}
  "/usr/local/include/libiomp"
  "/Users/longquanchen/Downloads/libav-10.7/"
  )

set(${KIT}_SRCS
  vtkIGTLCircularBuffer.cxx
	vtkIGTLCircularBuffer.h
	vtkIGTLToMRMLBase.cxx
	vtkIGTLToMRMLBase.h
	vtkIGTLToMRMLDepthVideo.cxx
	vtkIGTLToMRMLDepthVideo.h
	vtkMRMLIGTLConnectorNode.cxx
	vtkMRMLIGTLConnectorNode.h
	vtkMRMLIGTLQueryNode.cxx
	vtkMRMLIGTLQueryNode.h
)

message(${OpenH264Lib})
LINK_DIRECTORIES("/usr/local/lib")
LINK_DIRECTORIES("/Users/longquanchen/Desktop/Github/OpenIGTLink-xcodeBuild/Testing/OpenH264")
set(${KIT}_TARGET_LIBRARIES
  ${OpenIGTLinkLib}
  ${MRML_LIBRARIES}
  ${OpenH264Lib}
  SlicerBaseLogic
  vtkSlicerAnnotationsModuleMRML
  vtkSlicerMarkupsModuleMRML
  iomp5
  ${AVCODEC}
  ${AVUTIL}
  )

#-----------------------------------------------------------------------------
SlicerMacroBuildModuleMRML(
  NAME ${KIT}
  EXPORT_DIRECTIVE ${${KIT}_EXPORT_DIRECTIVE}
  INCLUDE_DIRECTORIES ${${KIT}_INCLUDE_DIRECTORIES}
  SRCS ${${KIT}_SRCS}
  TARGET_LIBRARIES ${${KIT}_TARGET_LIBRARIES}
  )
