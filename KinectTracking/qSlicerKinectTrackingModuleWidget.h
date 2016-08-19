/*==============================================================================
 
 Program: 3D Slicer
 
 Portions (c) Copyright Brigham and Women's Hospital (BWH) All Rights Reserved.
 
 See COPYRIGHT.txt
 or http://www.slicer.org/copyright/copyright.txt for details.
 
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 
 ==============================================================================*/

#ifndef __qSlicerKinectTrackingModuleWidget_h
#define __qSlicerKinectTrackingModuleWidget_h

// SlicerQt includes
#include "qSlicerAbstractModuleWidget.h"

#include "qSlicerKinectTrackingModuleExport.h"

// CTK includes
#include <ctkVTKObject.h>

// VTK includes
#include <vtkSmartPointer.h>
#include "vtkRenderer.h"
#include <vtkSmartPointer.h>
#include "vtkImageData.h"
#include "vtkImageActor.h"
#include "vtkRenderer.h"
#include "vtkActor.h"
#include "vtkImageActor.h"
#include "vtkInformation.h"
#include <vtkRenderWindow.h>
class qSlicerKinectTrackingModuleWidgetPrivate;
class vtkMRMLNode;
class vtkMRMLIGTLConnectorNode;
class vtkMRMLScalarVolumeNode;

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_KINECTTRACKING_EXPORT qSlicerKinectTrackingModuleWidget :
public qSlicerAbstractModuleWidget
{
  Q_OBJECT
  QVTK_OBJECT
public:
  
  typedef qSlicerAbstractModuleWidget Superclass;
  qSlicerKinectTrackingModuleWidget(QWidget *parent=0);
  virtual ~qSlicerKinectTrackingModuleWidget();
  
  public slots:
  void setMRMLScene(vtkMRMLScene* scene);
  /// the type
  void importDataAndEvents();
  
  
  protected slots:
  /// Internal function to update the widgets based on the IGTLConnector node
  void onMRMLNodeModified();
  
  void setMRMLIGTLConnectorNode(vtkMRMLIGTLConnectorNode* node);
  
  void startVideoTransmission(bool value);
  
  void UpdateTargetModel(vtkObject* sceneObject, vtkObject* nodeObject);
  
  void startCurrentIGTLConnector(bool enabled);
  
  /// Internal function to update the IGTLConnector node based on the property widget
  void updateIGTLConnectorNode();
  
  void AddingTargetModel(vtkObject* sceneObject, vtkObject* nodeObject);
  
  void AddingImage(vtkObject* sceneObject, vtkObject* nodeObject);
  
  void SelectModel(vtkMRMLNode* node);
  
  void SelectImageModel(vtkMRMLNode* node);
  
protected:
  QScopedPointer<qSlicerKinectTrackingModuleWidgetPrivate> d_ptr;
  
  
private:
  Q_DECLARE_PRIVATE(qSlicerKinectTrackingModuleWidget);
  Q_DISABLE_COPY(qSlicerKinectTrackingModuleWidget);
};

#endif
