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
#include <Vector>
#endif

namespace KinectDataRendering {
  
  class SurfaceRender {
  
  public:
    SurfaceRender();
    ~SurfaceRender();
    
    vtkSmartPointer<vtkPolyData> _polyData;
    vtkSmartPointer<vtkImageData> _imageData;
    vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter;
    
    std::vector<int> correspondPos;
    
    bool checkPointInRange(double * position);
    
    std::vector<double> boundary;
    
    void setPolyData(vtkSmartPointer<vtkPolyData> polyData);
    
    void setImageData(vtkImageData* imageData);
    
    void Rendering();
  };
}
