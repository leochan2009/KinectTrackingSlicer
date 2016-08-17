//
//  Tracking.cpp
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

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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

#include "Tracking.h"

typedef pcl::PointXYZRGB PointT;
pcl::PointCloud<PointT>::Ptr cloud_cylinder_total (new pcl::PointCloud<PointT> ());
vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter2(vtkSmartPointer<vtkVertexGlyphFilter>::New());
vtkSmartPointer<vtkUnsignedCharArray> colorsProcessed(vtkSmartPointer<vtkUnsignedCharArray>::New());
vtkSmartPointer<vtkPoints>  cloudProcessed(vtkSmartPointer<vtkPoints>::New());
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef PointT RefPointType;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
typedef PointT RefPointType;
pcl::PointCloud<PointT>::Ptr target_cloud (new pcl::PointCloud<PointT>);
CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
boost::mutex mtx_;
boost::shared_ptr<ParticleFilter> tracker_;

void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<PointT> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

void trackingInitialization(const std::string targetFileName)
{
  if(strncmp(&targetFileName.c_str()[targetFileName.size()-3], "ply",3)==0)
  {
    pcl::PLYReader reader;
    reader.read (targetFileName, *target_cloud);
    std::cerr << "PointCloud has: " << target_cloud->points.size () << " data points." << std::endl;
  }
  else if(strncmp(&targetFileName.c_str()[targetFileName.size()-3], "STL",3)==0||strncmp(&targetFileName.c_str()[targetFileName.size()-3], "stl",3)==0)
  {
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL (targetFileName, mesh);
    pcl::fromPCLPointCloud2(mesh.cloud, *target_cloud);
  }
  else
  {
    pcl::PCDReader reader;
    reader.read (targetFileName, *target_cloud);
  }
  Eigen::Vector4f center;
  pcl::compute3DCentroid<RefPointType> (*target_cloud, center);
  pcl::PointCloud<PointT>::Ptr target_cloud_half (new pcl::PointCloud<PointT>);
  for (int i = 0;i<target_cloud->size(); i++)
  {
    if(target_cloud->at(i).data[2] < center[2])
    {
      target_cloud_half->push_back(target_cloud->at(i));
    }
    target_cloud->at(i).data[2] = target_cloud->at(i).data[2] - center[2] + 1400;
    target_cloud->at(i).data[1] = target_cloud->at(i).data[1] - center[1];
    target_cloud->at(i).data[0] = target_cloud->at(i).data[0] - center[0];
    target_cloud->at(i).r = 255;
    target_cloud->at(i).g = 255;
    target_cloud->at(i).b = 255;
  }
  //pcl::PCDWriter writer;
  //writer.write("/Users/longquanchen/Desktop/Github/TrackingSample/build/Head/SkinRotatedHalf.pcd",*target_cloud_half);
  std::cerr << "PointCloud has: " << target_cloud->points.size () << " data points." << std::endl;
  //Set parameters
  float downsampling_grid_size_ =  2;
  
  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[0] *= 10000.0;
  default_step_covariance[1] *= 10000.0;
  default_step_covariance[2] *= 10000.0;
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;
  
  std::vector<double> initial_noise_covariance = std::vector<double> (6, 100);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
  
  boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
  (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));
  
  ParticleT bin_size;
  bin_size.x = 10.0f;
  bin_size.y = 10.0f;
  bin_size.z = 10.0f;
  bin_size.roll = 0.1f;
  bin_size.pitch = 0.1f;
  bin_size.yaw = 0.1f;
  
  
  //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
  tracker->setMaximumParticleNum (100);
  tracker->setDelta (0.99);
  tracker->setEpsilon (0.2);
  tracker->setBinSize (bin_size);
  
  //Set all parameters for  ParticleFilter
  tracker_ = tracker;
  tracker_->setTrans (Eigen::Affine3f::Identity ());
  tracker_->setStepNoiseCovariance (default_step_covariance);
  tracker_->setInitialNoiseCovariance (initial_noise_covariance);
  tracker_->setInitialNoiseMean (default_initial_mean);
  tracker_->setIterationNum (1);
  tracker_->setParticleNum (100);
  tracker_->setResampleLikelihoodThr(0.00);
  tracker_->setMinIndices(500);
  tracker_->setUseNormal (false);
  //tracker_->setMotionRatio(0.5);
  
  
  //Setup coherence object for tracking
  pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
  (new pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType> ());
  
  boost::shared_ptr<pcl::tracking::DistanceCoherence<RefPointType> > distance_coherence
  = boost::shared_ptr<pcl::tracking::DistanceCoherence<RefPointType> > (new pcl::tracking::DistanceCoherence<RefPointType> ());
  boost::shared_ptr<pcl::tracking::HSVColorCoherence<RefPointType> > hsvColor_coherence
  = boost::shared_ptr<pcl::tracking::HSVColorCoherence<RefPointType> > (new pcl::tracking::HSVColorCoherence<RefPointType> ());
  coherence->addPointCoherence (distance_coherence);
  //coherence->addPointCoherence (hsvColor_coherence);
  
  boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (2));
  coherence->setSearchMethod (search);
  coherence->setMaximumDistance (10);
  
  tracker_->setCloudCoherence (coherence);
  
  //prepare the model of tracker's target
  Eigen::Vector4f c;
  Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
  CloudPtr transed_ref (new Cloud);
  CloudPtr transed_ref_downsampled (new Cloud);
  
  pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
  trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
  pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
  gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
  
  //set reference model and trans
  tracker_->setReferenceCloud (transed_ref_downsampled);
  tracker_->setTrans (trans);
  
}

//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
  pcl::PassThrough<PointT> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (500.0, 3800.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}

//Draw model reference point cloud
void
drawResult ()
{
  pcl::tracking::ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
  
  //move close to camera a little for better visualization
  //transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
  CloudPtr result_cloud (new Cloud ());
  ParticleFilter::PointCloudInConstPtr refCloud = tracker_->getReferenceCloud ();
  pcl::transformPointCloud<RefPointType> (*(refCloud), *result_cloud, transformation);
  for(int index = 0; index<result_cloud->size();index++)
  {
    cloudProcessed->InsertNextPoint(result_cloud->at(index).data[0],result_cloud->at(index).data[1],result_cloud->at(index).data[2]);
    unsigned char color[3] = {refCloud->at(index).r,refCloud->at(index).g,refCloud->at(index).b};//result_cloud->at(index).r,result_cloud->at(index).g,result_cloud->at(index).b};
    colorsProcessed->InsertNextTypedTuple(color);
  }
  
  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
  //Set pointCloud with particle's points
  if(particles)
  {
    for (size_t i = 0; i < particles->points.size (); i++)
    {
      cloudProcessed->InsertNextPoint(particles->points[i].x,particles->points[i].y,particles->points[i].z);
      unsigned char color[3] = {255,0,0};
      colorsProcessed->InsertNextTypedTuple(color);
    }
  }
}

void TrackCylindarObject (vtkSmartPointer<vtkPolyData> polydata)
{
  boost::mutex::scoped_lock lock (mtx_);
  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
  CloudPtr cloud;
  cloud.reset(new Cloud);
  pcl::io::vtkPolyDataToPointCloud(polydata.GetPointer(), *cloud);
  filterPassThrough (cloud, *cloud_pass_);
  
  //Track the object
  tracker_->setInputCloud (cloud_pass_);
  tracker_->compute ();
  cloudProcessed->Reset();
  colorsProcessed->Reset();
  colorsProcessed->SetNumberOfComponents(3);
  colorsProcessed->SetName("Colors");
  for(int index = 0; index<cloud_pass_->size();index++)
  {
    cloudProcessed->InsertNextPoint(cloud_pass_->at(index).data[0],cloud_pass_->at(index).data[1],cloud_pass_->at(index).data[2]);
    unsigned char color[3] = {cloud_pass_->at(index).r,cloud_pass_->at(index).g,cloud_pass_->at(index).b};
    colorsProcessed->InsertNextTypedTuple(color);
  }
  drawResult();
  polydata->SetPoints(cloudProcessed);
  polydata->GetPointData()->SetScalars(colorsProcessed);
  vertexFilter2->SetInputData(polydata);
  vertexFilter2->Update();
  polydata->ShallowCopy(vertexFilter2->GetOutput());
}


