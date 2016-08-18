//
//  openni_tracking
//
//  Created by Longquan Chen on 8/4/16.
//
//

#ifndef SurfaceRendering_h
#define SurfaceRendering_h

#include <stdio.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSmartPointer.h>
#endif

namespace KinectDataRendering {

  void SurfaceRendering(vtkSmartPointer<vtkPolyData> polyData, vtkSmartPointer<vtkImageData> imageData);

}