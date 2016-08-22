//
//  CupModeling.hpp
//  openni_tracking
//
//  Created by Longquan Chen on 8/4/16.
//
//

#ifndef Tracking_h
#define Tracking_h

#include <stdio.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSmartPointer.h>
#endif /* Tracking_h */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZRGB PointT;

void TrackCylindarObject(vtkSmartPointer<vtkPolyData> polyData);
void trackingInitialization(pcl::PointCloud<PointT>::Ptr target_cloud);
void trackingInitializationWithName(const std::string targetFileName);
void trackingInitializationPLY(vtkSmartPointer<vtkPolyData> polyData);
Eigen::Affine3f GetTransform();