cmake_minimum_required(VERSION 2.8.9)

project(KinectTracking)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "http://slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/KinectTracking")
set(EXTENSION_CATEGORY "Extensions")
set(EXTENSION_CONTRIBUTORS "Longquan Chen (Brigham and Womens's Hospital)")
set(EXTENSION_DESCRIPTION "This is a module in charge of getting data fron the kinect device and using the poly data for the purpose of tracking organs")
set(EXTENSION_ICONURL "http://www.example.com/Slicer/Extensions/KinectTracking.png")
set(EXTENSION_SCREENSHOTURLS "http://www.example.com/Slicer/Extensions/KinectTracking/Screenshots/1.png")
set(EXTENSION_DEPENDS "NA") # Specified as a space separated string, a list or 'NA' if any

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(KinectTracking)
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_CPACK})
