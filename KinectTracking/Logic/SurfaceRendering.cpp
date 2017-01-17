//
//  openni_tracking
//
//  Created by Longquan Chen on 8/3/16.
//
//

#include <stdio.h>

//PCL Include
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <iomanip>
#include <math.h>
#include <cstdlib>
#include <cstring>

// VTK includes
#include <vtkNew.h>
#include <vtkCallbackCommand.h>
#include <vtkImageData.h>
#include <vtkTransform.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkVector.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkUnsignedCharArray.h>
#include <vtkDelaunay2D.h>
#include <vtkDataSetMapper.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkReverseSense.h>
#include <vtkContourFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include "SurfaceRendering.h"
namespace KinectDataRendering {
  
  SurfaceRender::SurfaceRender()
  {
    boundary = std::vector<double>(6);
    correspondPos = std::vector<int>(3);
    vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    _surfRecon = vtkSmartPointer<vtkReverseSense>::New();
    _threshold = 10;
  }
  SurfaceRender::~SurfaceRender()
  {}
  
  void SurfaceRender::setPolyData(vtkSmartPointer<vtkPolyData> polyData)
  {
    _polyData = polyData;
  }
  
  vtkSmartPointer<vtkPolyData> SurfaceRender::getPolyDataOverlay()
  {
    return _polyDataOverlay;
  }

  
  void SurfaceRender::setImageData(vtkImageData* imageData)
  {
    _imageData = imageData;
    int* dimension = imageData->GetDimensions();
    double* spacing = imageData->GetSpacing();
    boundary[0] = 0;
    boundary[1] = spacing[0]*dimension[0];
    boundary[2] = 0;
    boundary[3] = spacing[1]*dimension[1];
    boundary[4] = 0;
    boundary[5] = spacing[2]*dimension[2];
  }
  
  bool SurfaceRender::checkPointInRange(double * position)
  {
    if (position[0] > boundary[0] && position[0] < boundary[1] && \
        position[1] > boundary[2] && position[1] < boundary[3] && \
        position[2] > boundary[4] && position[2] < boundary[5])
    {
      correspondPos[0] = (int)std::floor(position[0]);
      correspondPos[1] = (int)std::floor(position[1]);
      correspondPos[2] = (int)std::floor(position[2]);
      return true;
    }
    return false;
  }
  
  bool SurfaceRender:: setThreshold(int threshold)
  {
    this->_threshold = threshold;
    return true;
  }
  
  void SurfaceRender::Rendering()
  {
    if (this->_polyData && this->_imageData)
    {
      vtkPoints * points = _polyData->GetPoints();
      vtkSmartPointer<vtkPoints> pointsOverlay = vtkSmartPointer<vtkPoints>::New();
      vtkSmartPointer<vtkUnsignedCharArray> colorsOverlay = vtkSmartPointer<vtkUnsignedCharArray>::New();
      colorsOverlay->SetName("Color");
      colorsOverlay->SetNumberOfComponents(3);

      vtkUnsignedCharArray* colors =(vtkUnsignedCharArray*)_polyData->GetPointData()->GetScalars();
      int pointNum = points->GetNumberOfPoints();
      int augmentedNum = 0;
      int threshold = this-> _threshold;
      for (int i = 0; i < pointNum; i++)
      {
        double* position = points->GetPoint(i);
        int ijk[3]={0};
        double pcoords[3]={0.0};
        if (this->_imageData->ComputeStructuredCoordinates(position, ijk, pcoords))
        {
          double voxelGreyValue = _imageData->GetScalarComponentAsDouble(ijk[0],ijk[1],ijk[2],0);
          if (voxelGreyValue > threshold)
          {
            pointsOverlay->InsertNextPoint(position[0],position[1],position[2]);
            unsigned char color[3] = {voxelGreyValue,voxelGreyValue,voxelGreyValue};
            colorsOverlay->InsertNextTypedTuple(color);
            augmentedNum++;
          }
        }
      }
      vtkSmartPointer<vtkPolyData> inputPolyData =
      vtkSmartPointer<vtkPolyData>::New();
      inputPolyData->SetPoints(pointsOverlay);
      vtkSmartPointer<vtkDelaunay2D> delaunay =
      vtkSmartPointer<vtkDelaunay2D>::New();
      delaunay->SetInputData(inputPolyData);
      delaunay->Update();
      
      vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter =
      vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
      smoothFilter->SetInputConnection(delaunay->GetOutputPort());
      smoothFilter->SetNumberOfIterations(15);
      smoothFilter->SetRelaxationFactor(0.1);
      smoothFilter->FeatureEdgeSmoothingOff();
      smoothFilter->BoundarySmoothingOn();
      smoothFilter->Update();
      
      inputPolyData = smoothFilter->GetOutput();
      if (inputPolyData->GetPoints())
      {
        points = inputPolyData->GetPoints();
        pointNum = points->GetNumberOfPoints();
        pointsOverlay->Reset();
        colorsOverlay->Reset();
        for (int i = 0; i < pointNum; i++)
        {
          double* position = points->GetPoint(i);
          int ijk[3]={0};
          double pcoords[3]={0.0};
          if (this->_imageData->ComputeStructuredCoordinates(position, ijk, pcoords))
          {
            double voxelGreyValue = _imageData->GetScalarComponentAsDouble(ijk[0],ijk[1],ijk[2],0);
            if (voxelGreyValue > threshold)
            {
              pointsOverlay->InsertNextPoint(position[0],position[1],position[2]);
              unsigned char color[3] = {voxelGreyValue,voxelGreyValue,voxelGreyValue};
              colorsOverlay->InsertNextTypedTuple(color);
              augmentedNum++;
            }
          }
        }
        inputPolyData->SetPoints(pointsOverlay);
        delaunay->SetInputData(inputPolyData);
        delaunay->Update();
      }
      
      this->_polyDataOverlay = delaunay->GetOutput();
      this->_polyDataOverlay->GetPointData()->SetScalars(colorsOverlay);
      
    }
  }
}

