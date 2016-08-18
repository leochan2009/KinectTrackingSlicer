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

// .NAME vtkSlicerKinectTrackingLogic - slicer logic class for volumes manipulation
// .SECTION Description
// This class manages the logic associated with reading, saving,
// and changing propertied of the volumes


#ifndef __vtkSlicerKinectTrackingLogic_h
#define __vtkSlicerKinectTrackingLogic_h


// Slicer includes
#include "vtkSlicerModuleLogic.h"

// MRML includes
#include "vtkIGTLToMRMLDepthVideo.h"
#include "vtkIGTLToMRMLBase.h"

// STD includes
#include <cstdlib>

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
#include <vtkUnsignedCharArray.h>

//Local includes
#include "Tracking.h"
#include "SurfaceRendering.h"

#include "vtkSlicerKinectTrackingModuleLogicExport.h"

class vtkMRMLIGTLConnectorNode;
/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_KINECTTRACKING_MODULE_LOGIC_EXPORT vtkSlicerKinectTrackingLogic :
public vtkSlicerModuleLogic
{
public:
  
  enum {  // Events
    StatusUpdateEvent       = 50001,
    //SliceUpdateEvent        = 50002,
  };
  
  typedef struct {
    std::string name;
    std::string type;
    int io;
    std::string nodeID;
  } IGTLMrmlNodeInfoType;
  
  typedef std::vector<IGTLMrmlNodeInfoType>         IGTLMrmlNodeListType;
  typedef std::vector<vtkIGTLToMRMLBase*>           MessageConverterListType;
  
  // Work phase keywords used in NaviTrack (defined in BRPTPRInterface.h)
  
public:
  
  static vtkSlicerKinectTrackingLogic *New();
  vtkTypeMacro(vtkSlicerKinectTrackingLogic, vtkSlicerModuleLogic);
  
  virtual void RegisterNodes();
  
  //----------------------------------------------------------------
  // Connector and converter Management
  //----------------------------------------------------------------
  
  // Access connectors
  vtkMRMLIGTLConnectorNode* GetConnector(const char* conID);
  
  // Call timer-driven routines for each connector
  vtkSmartPointer<vtkPolyData>  CallConnectorTimerHander();
  
  int  RegisterMessageConverter(vtkIGTLToMRMLBase* converter);
  int  UnregisterMessageConverter(vtkIGTLToMRMLBase* converter);
  
  unsigned int       GetNumberOfConverters();
  vtkIGTLToMRMLBase* GetConverter(unsigned int i);
  vtkIGTLToMRMLBase* GetConverterByMRMLTag(const char* mrmlTag);
  vtkIGTLToMRMLBase* GetConverterByDeviceType(const char* deviceType);
  
  //----------------------------------------------------------------
  // MRML Management
  //----------------------------------------------------------------
  
  virtual void ProcessMRMLNodesEvents(vtkObject* caller, unsigned long event, void * callData);
  //virtual void ProcessLogicEvents(vtkObject * caller, unsigned long event, void * callData);
  
  void ProcCommand(const char* nodeName, int size, unsigned char* data);
  
  void GetDeviceNamesFromMrml(IGTLMrmlNodeListType &list);
  void GetDeviceNamesFromMrml(IGTLMrmlNodeListType &list, const char* mrmlTagName);
  //void GetDeviceTypes(std::vector<char*> &list);
  unsigned char * GetFrame(){return RGBFrame;};
  
  vtkSmartPointer<vtkPolyData> polyData;
  
  vtkSmartPointer<vtkImageData> imageData;
  
  void ResetTargetModel(vtkSmartPointer<vtkPolyData> targetPolyData);
  
  bool EnableTracking;
  bool SurfaceRendering;
  
protected:
  unsigned char * DepthFrame;
  unsigned char * RGBFrame;
  unsigned char * DepthIndex;
  
  
  //----------------------------------------------------------------
  // Constructor, destructor etc.
  //----------------------------------------------------------------
  
  vtkSlicerKinectTrackingLogic();
  virtual ~vtkSlicerKinectTrackingLogic();
  
  void AddMRMLConnectorNodeObserver(vtkMRMLIGTLConnectorNode * connectorNode);
  void RemoveMRMLConnectorNodeObserver(vtkMRMLIGTLConnectorNode * connectorNode);
  
  void RegisterMessageConverters(vtkMRMLIGTLConnectorNode * connectorNode);
  
  void UpdateAll();
  void UpdateSliceDisplay();
  vtkSmartPointer<vtkPolyData> ConvertDepthToPoints(unsigned char* buf, unsigned char* bufIndex, unsigned char* bufColor, int depth_width_, int depth_height_);
  
private:
  
  int Initialized;
  
  //----------------------------------------------------------------
  // Connector Management
  //----------------------------------------------------------------
  
  //ConnectorMapType              ConnectorMap;
  MessageConverterListType      MessageConverterList;
  
  
  //----------------------------------------------------------------
  // IGTL-MRML converters
  //----------------------------------------------------------------
  vtkIGTLToMRMLDepthVideo * PolyConverter;

  std::vector<int> lu;
  vtkSmartPointer<vtkUnsignedCharArray> colors;
  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter;
  vtkSmartPointer<vtkPoints>  cloud;
  
private:
  
  vtkSlicerKinectTrackingLogic(const vtkSlicerKinectTrackingLogic&); // Not implemented
  void operator=(const vtkSlicerKinectTrackingLogic&);               // Not implemented
};


#endif
