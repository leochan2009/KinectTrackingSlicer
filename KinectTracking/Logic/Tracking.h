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

void TrackCylindarObject(vtkSmartPointer<vtkPolyData> polyData);
void trackingInitialization(const std::string targetFileName);