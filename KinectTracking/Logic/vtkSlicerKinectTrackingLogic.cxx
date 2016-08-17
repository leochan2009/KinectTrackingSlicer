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

// KinectTracking Logic includes
#include "vtkSlicerKinectTrackingLogic.h"

// MRML includes
#include <vtkMRMLScene.h>
#include "vtkMRMLIGTLQueryNode.h"
#include "vtkMRMLIGTLConnectorNode.h"

// VTK includes
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>

// STD includes
#include <cassert>

//PCL IO
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerKinectTrackingLogic);
vtkSmartPointer<vtkPolyData> vtkSlicerKinectTrackingLogic::ConvertDepthToPoints(unsigned char* buf, unsigned char* bufIndex, unsigned char* bufColor, int depth_width_, int depth_height_) // now it is fixed to 512, 424
{
  ;//(depth_width_*depth_height_,vtkVector<float, 3>());
  
  bool isDepthOnly = false;
  cloud->Reset();
  colors->Reset();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");
  //I inserted 525 as Julius Kammerl said as focal length
  register float constant = 0;
  
  if (isDepthOnly)
  {
    constant = 3.501e-3f;
  }
  else
  {
    constant = 1.83e-3f;
  }
  
  register int centerX = (depth_width_ >> 1);
  int centerY = (depth_height_ >> 1);
  
  //I also ignore invalid values completely
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  bool _useDemux = true;
  int DemuxMethod = 2;
  if(_useDemux)
  {
    if (DemuxMethod == 1)
    {
      std::vector<uint8_t> pBuf(depth_width_*depth_height_*4, 0);
      for (int i = 0; i< depth_width_*depth_height_*4 ; i++)
      {
        pBuf[i] = *(buf+i);
      }
      register int depth_idx = 0;
      std::vector<int> strides(4,0);
      int pointNum = 0;
      for (int v = -centerY; v < centerY; ++v)
      {
        for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
        {
          strides[0] = depth_width_*2 * (v+centerY) + u + centerX;
          strides[1] = depth_width_*2 * (v+centerY) + depth_width_ + u + centerX;
          strides[2] = depth_width_*2 * (v+centerY) + depth_height_*depth_width_*2 + u + centerX;
          strides[3] = depth_width_*2 * (v+centerY) + depth_height_*depth_width_*2 + depth_width_ + u + centerX;
          for (int k = 0 ; k < 4; k++)
          {
            vtkVector<float, 3> pt;
            //This part is used for invalid measurements, I removed it
            int pixelValue = buf[strides[k]];
            if (pixelValue == 0 || (bufColor[3*depth_idx] == 0 && bufColor[3*depth_idx+1] == 0 && bufColor[3*depth_idx+2]==0) )
            {
              // not valid
              pt[0] = pt[1] = pt[2] = bad_point;
              continue;
            }
            pt[2] = pixelValue+k*255 + 500;
            pt[0] = static_cast<float> (-u) * pt[2] * constant;
            pt[1] = static_cast<float> (-v) * pt[2] * constant;
            cloud->InsertNextPoint(pt[0],pt[1],pt[2]);
            unsigned char color[3] = {bufColor[3*depth_idx],bufColor[3*depth_idx+1],bufColor[3*depth_idx+2]};
            colors->SetTypedTuple(pointNum,color);
            pointNum ++;
            break;
          }
        }
      }
      pBuf.clear();
    }
    else if(DemuxMethod==2)
    {
      std::vector<uint8_t> pBuf(depth_width_*depth_height_, 0);
      for (int i = 0; i< depth_width_*depth_height_; i++)
      {
        pBuf[i] = *(bufIndex+i);
      }
      register int depth_idx = 0;
      int pointNum = 0;
      for (int v = -centerY; v < centerY; ++v)
      {
        for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
        {
          vtkVector<float, 3> pt;
          //This part is used for invalid measurements, I removed it
          int pixelValue = buf[depth_idx];
          if((bufIndex[depth_idx] == 0) || (bufColor[3*depth_idx] == 0 && bufColor[3*depth_idx+1] == 0 && bufColor[3*depth_idx+2]==0) )
          {
            // not valid
            pt[0] = pt[1] = pt[2] = bad_point;
            continue;
          }
          pt[2] = pixelValue + (bufIndex[depth_idx]-1)*256 + 500;
          pt[0] = static_cast<float> (-u) * pt[2] * constant;
          pt[1] = static_cast<float> (-v) * pt[2] * constant;
          cloud->InsertNextPoint(pt[0],pt[1],pt[2]);
          unsigned char color[3] = {bufColor[3*depth_idx],bufColor[3*depth_idx+1],bufColor[3*depth_idx+2]};
          colors->SetTypedTuple(pointNum, color);
          pointNum ++;
        }
      }
    }
  }
  else
  {
    std::vector<uint8_t> pBuf(depth_width_*depth_height_, 0);
    for (int i = 0; i< depth_width_*depth_height_; i++)
    {
      pBuf[i] = *(buf+i);
    }
    register int depth_idx = 0;
    int pointNum = 0;
    for (int v = -centerY; v < centerY; ++v)
    {
      for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
      {
        vtkVector<float, 3> pt;
        //This part is used for invalid measurements, I removed it
        int pixelValue = pBuf[depth_idx];
        if (bufColor[3*depth_idx] == 0 && bufColor[3*depth_idx+1] == 0 && bufColor[3*depth_idx+2]==0 )
        {
          // not valid
          pt[0] = pt[1] = pt[2] = bad_point;
          continue;
        }
        pt[2] = pixelValue + 500;
        pt[0] = static_cast<float> (-u) * pt[2] * constant;
        pt[1] = static_cast<float> (-v) * pt[2] * constant;
        cloud->InsertNextPoint(pt[0],pt[1],pt[2]);
        unsigned char color[3] = {bufColor[3*depth_idx],bufColor[3*depth_idx+1],bufColor[3*depth_idx+2]};
        colors->SetTypedTuple(pointNum,color);
        //delete[] color;
        pointNum ++;
      }
    }
    pBuf.clear();
  }
  int pointNum = cloud->GetNumberOfPoints();
  if (pointNum>0)
  {
    polyData->SetPoints(cloud);
    polyData->GetPointData()->SetScalars(colors);
    vertexFilter->SetInputData(polyData);
    vertexFilter->Update();
    polyData->ShallowCopy(vertexFilter->GetOutput());
  }
  return polyData;;
}

vtkSlicerKinectTrackingLogic::vtkSlicerKinectTrackingLogic()
{
  const std::string targetFileName = "/Users/longquanchen/Desktop/Github/TrackingSample/build/Head/SkinRotatedHalf.pcd";
  this->Initialized   = 0;
  this->MessageConverterList.clear();
  this->polyData = vtkSmartPointer<vtkPolyData>::New();
  // register default data types
  this->PolyConverter = vtkIGTLToMRMLDepthVideo::New();
  
  colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  cloud = vtkSmartPointer<vtkPoints>::New();
  RegisterMessageConverter(this->PolyConverter);
  trackingInitialization(targetFileName);
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB >::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read (targetFileName, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
  pcl::io::pointCloudTovtkPolyData<pcl::PointXYZRGB >(*cloud, polyData.GetPointer());

  //this->LocatorTransformNode = NULL;
}

//---------------------------------------------------------------------------
vtkSlicerKinectTrackingLogic::~vtkSlicerKinectTrackingLogic()
{
  if (this->PolyConverter)
  {
    UnregisterMessageConverter(this->PolyConverter);
    this->PolyConverter->Delete();
  }

}

//----------------------------------------------------------------------------
void vtkSlicerKinectTrackingLogic::AddMRMLConnectorNodeObserver(vtkMRMLIGTLConnectorNode * connectorNode)
{
  if (!connectorNode)
  {
    return;
  }
  // Make sure we don't add duplicate observation
  vtkUnObserveMRMLNodeMacro(connectorNode);
  // Start observing the connector node
  vtkNew<vtkIntArray> connectorNodeEvents;
  connectorNodeEvents->InsertNextValue(vtkMRMLIGTLConnectorNode::DeviceModifiedEvent);
  vtkObserveMRMLNodeEventsMacro(connectorNode, connectorNodeEvents.GetPointer());
}

//----------------------------------------------------------------------------
void vtkSlicerKinectTrackingLogic::RemoveMRMLConnectorNodeObserver(vtkMRMLIGTLConnectorNode * connectorNode)
{
  if (!connectorNode)
  {
    return;
  }
  vtkUnObserveMRMLNodeMacro(connectorNode);
}

//---------------------------------------------------------------------------
void vtkSlicerKinectTrackingLogic::RegisterMessageConverters(vtkMRMLIGTLConnectorNode * connectorNode)
{
  if (!connectorNode)
  {
    return;
  }
  for (unsigned short i = 0; i < this->GetNumberOfConverters(); i ++)
  {
    connectorNode->RegisterMessageConverter(this->GetConverter(i));
  }
}


//---------------------------------------------------------------------------
vtkMRMLIGTLConnectorNode* vtkSlicerKinectTrackingLogic::GetConnector(const char* conID)
{
  vtkMRMLNode* node = this->GetMRMLScene()->GetNodeByID(conID);
  if (node)
  {
    vtkMRMLIGTLConnectorNode* conNode = vtkMRMLIGTLConnectorNode::SafeDownCast(node);
    return conNode;
  }
  else
  {
    return NULL;
  }
}

//---------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkSlicerKinectTrackingLogic::CallConnectorTimerHander()
{
  //ConnectorMapType::iterator cmiter;
  std::vector<vtkMRMLNode*> nodes;
  this->GetMRMLScene()->GetNodesByClass("vtkMRMLIGTLConnectorNode", nodes);
  
  std::vector<vtkMRMLNode*>::iterator iter;
  RGBFrame = NULL;
  DepthFrame = NULL;
  DepthIndex = NULL;
  //for (cmiter = this->ConnectorMap.begin(); cmiter != this->ConnectorMap.end(); cmiter ++)
  for (iter = nodes.begin(); iter != nodes.end(); iter ++)
  {
    vtkMRMLIGTLConnectorNode* connector = vtkMRMLIGTLConnectorNode::SafeDownCast(*iter);
    if (connector == NULL)
    {
      continue;
    }
    connector->ImportDataFromCircularBuffer();
    connector->ImportEventsFromEventBuffer();
    connector->PushOutgoingMessages();
    RGBFrame = connector->RGBFrame;
    DepthFrame = connector->DepthFrame;
    DepthIndex = connector->DepthIndex;
  }
  if (DepthFrame && RGBFrame && DepthIndex)
  {
    int64_t conversionTime = Connector::getTime();
    TrackCylindarObject(ConvertDepthToPoints((unsigned char*)DepthFrame,DepthIndex, RGBFrame, 512, 424));
    return this->polyData;
    std::cerr<<"Depth Image conversion Time: "<<(Connector::getTime()-conversionTime)/1e6 << std::endl;
    
  }
  return NULL;
}

//---------------------------------------------------------------------------
int vtkSlicerKinectTrackingLogic::RegisterMessageConverter(vtkIGTLToMRMLBase* converter)
{
  if (converter == NULL)
  {
    return 0;
  }
  
  // Search the list and check if the same converter has already been registered.
  int found = 0;
  
  MessageConverterListType::iterator iter;
  for (iter = this->MessageConverterList.begin();
       iter != this->MessageConverterList.end();
       iter ++)
  {
    if ((converter->GetIGTLName() && strcmp(converter->GetIGTLName(), (*iter)->GetIGTLName()) == 0) &&
        (converter->GetMRMLName() && strcmp(converter->GetMRMLName(), (*iter)->GetMRMLName()) == 0))
    {
      found = 1;
    }
  }
  if (found)
  {
    return 0;
  }
  
  if (converter->GetIGTLName() && converter->GetMRMLName())
    // TODO: is this correct? Shouldn't it be "&&"
  {
    this->MessageConverterList.push_back(converter);
  }
  else
  {
    return 0;
  }
  
  // Add the converter to the existing connectors
  if (this->GetMRMLScene())
  {
    std::vector<vtkMRMLNode*> nodes;
    this->GetMRMLScene()->GetNodesByClass("vtkMRMLIGTLConnectorNode", nodes);
    
    std::vector<vtkMRMLNode*>::iterator niter;
    for (niter = nodes.begin(); niter != nodes.end(); niter ++)
    {
      vtkMRMLIGTLConnectorNode* connector = vtkMRMLIGTLConnectorNode::SafeDownCast(*niter);
      if (connector)
      {
        connector->RegisterMessageConverter(converter);
      }
    }
  }
  
  return 1;
}

//---------------------------------------------------------------------------
int vtkSlicerKinectTrackingLogic::UnregisterMessageConverter(vtkIGTLToMRMLBase* converter)
{
  if (converter == NULL)
  {
    return 0;
  }
  
  // Look up the message converter list
  MessageConverterListType::iterator iter;
  iter = this->MessageConverterList.begin();
  while ((*iter) != converter) iter ++;
  
  if (iter != this->MessageConverterList.end())
    // if the converter is on the list
  {
    this->MessageConverterList.erase(iter);
    // Remove the converter from the existing connectors
    std::vector<vtkMRMLNode*> nodes;
    if (this->GetMRMLScene())
    {
      this->GetMRMLScene()->GetNodesByClass("vtkMRMLIGTLConnectorNode", nodes);
      
      std::vector<vtkMRMLNode*>::iterator iter;
      for (iter = nodes.begin(); iter != nodes.end(); iter ++)
      {
        vtkMRMLIGTLConnectorNode* connector = vtkMRMLIGTLConnectorNode::SafeDownCast(*iter);
        if (connector)
        {
          connector->UnregisterMessageConverter(converter);
        }
      }
    }
    return 1;
  }
  else
  {
    return 0;
  }
}

//---------------------------------------------------------------------------
unsigned int vtkSlicerKinectTrackingLogic::GetNumberOfConverters()
{
  return this->MessageConverterList.size();
}

//---------------------------------------------------------------------------
vtkIGTLToMRMLBase* vtkSlicerKinectTrackingLogic::GetConverter(unsigned int i)
{
  
  if (i < this->MessageConverterList.size())
  {
    return this->MessageConverterList[i];
  }
  else
  {
    return NULL;
  }
}

//---------------------------------------------------------------------------
vtkIGTLToMRMLBase* vtkSlicerKinectTrackingLogic::GetConverterByMRMLTag(const char* mrmlTag)
{
  //Currently, this function cannot find multiple converters
  // that use the same mrmlType (e.g. vtkIGTLToMRMLLinearTransform
  // and vtkIGTLToMRMLPosition). A converter that is found first
  // will be returned.
  
  vtkIGTLToMRMLBase* converter = NULL;
  
  MessageConverterListType::iterator iter;
  for (iter = this->MessageConverterList.begin();
       iter != this->MessageConverterList.end();
       iter ++)
  {
    if (strcmp((*iter)->GetMRMLName(), mrmlTag) == 0)
    {
      converter = *iter;
      break;
    }
  }
  
  return converter;
}

//---------------------------------------------------------------------------
vtkIGTLToMRMLBase* vtkSlicerKinectTrackingLogic::GetConverterByDeviceType(const char* deviceType)
{
  vtkIGTLToMRMLBase* converter = NULL;
  
  MessageConverterListType::iterator iter;
  for (iter = this->MessageConverterList.begin();
       iter != this->MessageConverterList.end();
       iter ++)
  {
    if ((*iter)->GetConverterType() == vtkIGTLToMRMLBase::TYPE_NORMAL)
    {
      if (strcmp((*iter)->GetIGTLName(), deviceType) == 0)
      {
        converter = *iter;
        break;
      }
    }
    else
    {
      int n = (*iter)->GetNumberOfIGTLNames();
      for (int i = 0; i < n; i ++)
      {
        if (strcmp((*iter)->GetIGTLName(i), deviceType) == 0)
        {
          converter = *iter;
          break;
        }
      }
    }
  }
  
  return converter;
}

//---------------------------------------------------------------------------
void vtkSlicerKinectTrackingLogic::ProcessMRMLNodesEvents(vtkObject * caller, unsigned long event, void * callData)
{
  
  if (caller != NULL)
  {
    vtkSlicerModuleLogic::ProcessMRMLNodesEvents(caller, event, callData);
    
    vtkMRMLIGTLConnectorNode* cnode = vtkMRMLIGTLConnectorNode::SafeDownCast(caller);
    if (cnode && event == vtkMRMLIGTLConnectorNode::DeviceModifiedEvent)
    {
      // Check visibility
      int nnodes;
      
      // Incoming nodes
      nnodes = cnode->GetNumberOfIncomingMRMLNodes();
      for (int i = 0; i < nnodes; i ++)
      {
        vtkMRMLNode* inode = cnode->GetIncomingMRMLNode(i);
        if (inode)
        {
          vtkIGTLToMRMLBase* converter = GetConverterByMRMLTag(inode->GetNodeTagName());
          if (converter)
          {
            const char * attr = inode->GetAttribute("IGTLVisible");
            if (attr && strcmp(attr, "true") == 0)
            {
              converter->SetVisibility(1, this->GetMRMLScene(), inode);
            }
            else
            {
              converter->SetVisibility(0, this->GetMRMLScene(), inode);
            }
          }
        }
      }
      
      // Outgoing nodes
      nnodes = cnode->GetNumberOfOutgoingMRMLNodes();
      for (int i = 0; i < nnodes; i ++)
      {
        vtkMRMLNode* inode = cnode->GetOutgoingMRMLNode(i);
        if (inode)
        {
          vtkIGTLToMRMLBase* converter = GetConverterByMRMLTag(inode->GetNodeTagName());
          if (converter)
          {
            const char * attr = inode->GetAttribute("IGTLVisible");
            if (attr && strcmp(attr, "true") == 0)
            {
              converter->SetVisibility(1, this->GetMRMLScene(), inode);
            }
            else
            {
              converter->SetVisibility(0, this->GetMRMLScene(), inode);
            }
          }
        }
      }
    }
  }
}

//---------------------------------------------------------------------------
void vtkSlicerKinectTrackingLogic::RegisterNodes()
{
  vtkMRMLScene * scene = this->GetMRMLScene();
  if(!scene)
  {
    return;
  }
  
  scene->RegisterNodeClass(vtkNew<vtkMRMLIGTLQueryNode>().GetPointer(), "vtkMRMLIGTLQueryNode");
  scene->RegisterNodeClass(vtkNew<vtkMRMLIGTLConnectorNode>().GetPointer(), "vtkMRMLIGTLConnectorNode");
  
}


//---------------------------------------------------------------------------
void vtkSlicerKinectTrackingLogic::ProcCommand(const char* vtkNotUsed(nodeName), int vtkNotUsed(size), unsigned char* vtkNotUsed(data))
{
}

