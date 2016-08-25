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
#include <vtkReverseSense.h>
#include <Vector>
#endif

namespace KinectDataRendering {
  
  class SurfaceRender {
  
  public:
    SurfaceRender();
    ~SurfaceRender();
    
    vtkSmartPointer<vtkPolyData> _polyData;
    vtkSmartPointer<vtkPolyData>  _polyDataOverlay;
    vtkSmartPointer<vtkReverseSense>  _surfRecon;
    vtkSmartPointer<vtkImageData> _imageData;
    vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter;
    
    int _threshold;
    
    std::vector<int> correspondPos;
    
    bool checkPointInRange(double * position);
    
    std::vector<double> boundary;
    
    void setPolyData(vtkSmartPointer<vtkPolyData> polyData);
    
    vtkSmartPointer<vtkPolyData> getPolyDataOverlay();
    
    void setImageData(vtkImageData* imageData);
    
    void Rendering();
    
    bool setThreshold(int threshold);
  };
}
