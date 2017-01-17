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

// Qt includes
#include <QDebug>
#include <QTimer>
#include <QPointer>

// SlicerQt includes
#include "qSlicerKinectTrackingModuleWidget.h"
#include "ui_qSlicerKinectTrackingModuleWidget.h"
#include "qSlicerApplication.h"
#include "qSlicerLayoutManager.h"
#include "qSlicerWidget.h"
#include "qMRMLSliceWidget.h"
#include "qMRMLSliceControllerWidget.h"

// Logic Include
#include "vtkSlicerKinectTrackingLogic.h"

// OpenIGTLinkIF MRML includes
#include "vtkMRMLIGTLConnectorNode.h"
#include "vtkIGTLToMRMLDepthVideo.h"
#include "vtkIGTLToMRMLString.h"
#include "vtkMRMLIGTLQueryNode.h"
#include "vtkMRMLTextNode.h"
#include <qMRMLNodeFactory.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLTransformNode.h>
#include <vtkMRMLScalarVolumeNode.h>
#include <vtkMRMLSliceCompositeNode.h>

#include "igtlStringMessage.h"

//VTK include
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSetMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkReverseSense.h>

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerKinectTrackingModuleWidgetPrivate: public Ui_qSlicerKinectTrackingModuleWidget
{
  Q_DECLARE_PUBLIC(qSlicerKinectTrackingModuleWidget);
protected:
  qSlicerKinectTrackingModuleWidget* const q_ptr;
public:
  qSlicerKinectTrackingModuleWidgetPrivate(qSlicerKinectTrackingModuleWidget& object);
  
  vtkMRMLIGTLConnectorNode * IGTLConnectorNode;
  vtkMRMLIGTLConnectorNode * IGTLRobotConnectorNode;
  vtkMRMLIGTLQueryNode * IGTLDataQueryNode;
  vtkIGTLToMRMLDepthVideo* converter;
  vtkIGTLToMRMLString* targetPosConverter;
  QTimer ImportDataAndEventsTimer;
  vtkSlicerKinectTrackingLogic * logic();
  vtkRenderer* activeRenderer;
  vtkRenderer*   PolyDataRenderer;
  vtkSmartPointer<vtkActor> PolyDataActor;
  vtkSmartPointer<vtkActor> SurfaceActor;
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkPolyData> polyDataOverlay;
  vtkSmartPointer<vtkDataSetMapper> mapper;
  vtkSmartPointer<vtkDataSetMapper> sliceMapper;
  vtkSmartPointer<vtkMRMLSliceCompositeNode> RedSliceCompositionNode;
  uint8_t * RGBFrame;
  int picWidth = 512;
  int picHeight = 424;
  
};

//-----------------------------------------------------------------------------
// qSlicerKinectTrackingModuleWidgetPrivate methods

//-----------------------------------------------------------------------------
qSlicerKinectTrackingModuleWidgetPrivate::qSlicerKinectTrackingModuleWidgetPrivate(qSlicerKinectTrackingModuleWidget& object):q_ptr(&object)
{
  this->IGTLConnectorNode = NULL;
  this->IGTLRobotConnectorNode = NULL;
  this->IGTLDataQueryNode = NULL;
  RGBFrame = new uint8_t[picWidth*picHeight*3];
}

//-----------------------------------------------------------------------------
vtkSlicerKinectTrackingLogic * qSlicerKinectTrackingModuleWidgetPrivate::logic()
{
  Q_Q(qSlicerKinectTrackingModuleWidget);
  return vtkSlicerKinectTrackingLogic::SafeDownCast(q->logic());
}

//-----------------------------------------------------------------------------
// qSlicerKinectTrackingModuleWidget methods

//-----------------------------------------------------------------------------
qSlicerKinectTrackingModuleWidget::qSlicerKinectTrackingModuleWidget(QWidget* _parent)
: Superclass( _parent )
, d_ptr( new qSlicerKinectTrackingModuleWidgetPrivate(*this) )
{
  Q_D(qSlicerKinectTrackingModuleWidget);

  d->setupUi(this);
  this->Superclass::setup();
  QObject::connect(&d->ImportDataAndEventsTimer, SIGNAL(timeout()),
                   this, SLOT(importDataAndEvents()));
  QObject::connect(d->ConnectorPortEdit, SIGNAL(editingFinished()),
                   this, SLOT(updateIGTLConnectorNode()));
  QObject::connect(d->ConnectorHostAddressEdit, SIGNAL(editingFinished()),
                   this, SLOT(updateIGTLConnectorNode()));
  QObject::connect(d->ConnectorPortEdit_2, SIGNAL(editingFinished()),
                   this, SLOT(updateIGTLConnectorNode()));
  QObject::connect(d->ConnectorHostAddressEdit_2, SIGNAL(editingFinished()),
                   this, SLOT(updateIGTLConnectorNode()));
  QObject::connect(d->ConnectorStateCheckBox, SIGNAL(toggled(bool)),
                   this, SLOT(startCurrentIGTLConnector(bool)));
  QObject::connect(d->StartVideoCheckBox, SIGNAL(toggled(bool)),
                   this, SLOT(startVideoTransmission(bool)));
  QObject::connect(d->NodeSelector, SIGNAL(currentNodeChanged(vtkMRMLNode*)), this, SLOT(SelectModel(vtkMRMLNode*)));
  QObject::connect(d->NodeSelectorTransform, SIGNAL(currentNodeChanged(vtkMRMLNode*)), this, SLOT(SelectTransform(vtkMRMLNode*)));
  qSlicerApplication *  app = qSlicerApplication::application();
  qMRMLSliceWidget* sliceWidget = app->layoutManager()->sliceWidget("Red");
  qMRMLSliceControllerWidget* sliceControllerWidget = sliceWidget->sliceController();
  qMRMLNodeComboBox* comboBox =
  qobject_cast<qMRMLNodeComboBox*>(sliceControllerWidget->findChild<qMRMLNodeComboBox*>("BackgroundComboBox"));
  QObject::connect(comboBox, SIGNAL(currentNodeChanged(vtkMRMLNode*)),
                      SLOT(SelectImageModel(vtkMRMLNode*)));
  
  vtkRenderer* activeRenderer = app->layoutManager()->activeThreeDRenderer();
  d->PolyDataRenderer = activeRenderer;
  vtkRenderWindow* activeRenderWindow = activeRenderer->GetRenderWindow();
  d->polydata = vtkSmartPointer<vtkPolyData>::New();
  d->polyDataOverlay = vtkSmartPointer<vtkPolyData>::New();
  if (activeRenderer)
  {
    d->mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    d->sliceMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    d->sliceMapper->SetInputData(d->polyDataOverlay);
    d->mapper->SetInputData(d->polydata);
    d->PolyDataActor = vtkSmartPointer<vtkActor>::New();
    d->PolyDataActor->SetMapper(d->mapper);
    d->SurfaceActor = vtkSmartPointer<vtkActor>::New();
    d->SurfaceActor->SetMapper(d->sliceMapper);
    d->PolyDataRenderer->AddActor(d->PolyDataActor);
    d->PolyDataRenderer->AddActor(d->SurfaceActor);
    activeRenderWindow->AddRenderer(d->PolyDataRenderer);
    activeRenderWindow->Render();
    activeRenderWindow->GetInteractor()->Start();
    d->graphicsView->setFixedSize(d->picWidth, d->picHeight);
  }
}

//-----------------------------------------------------------------------------
qSlicerKinectTrackingModuleWidget::~qSlicerKinectTrackingModuleWidget()
{
}

//-----------------------------------------------------------------------------
void qSlicerKinectTrackingModuleWidget::setMRMLScene(vtkMRMLScene* scene)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  
  this->Superclass::setMRMLScene(scene);
  if (this->mrmlScene())
  {
    d->IGTLConnectorNode = vtkMRMLIGTLConnectorNode::SafeDownCast(this->mrmlScene()->CreateNodeByClass("vtkMRMLIGTLConnectorNode"));
    d->IGTLRobotConnectorNode = vtkMRMLIGTLConnectorNode::SafeDownCast(this->mrmlScene()->CreateNodeByClass("vtkMRMLIGTLConnectorNode"));
    d->IGTLDataQueryNode = vtkMRMLIGTLQueryNode::SafeDownCast(this->mrmlScene()->CreateNodeByClass("vtkMRMLIGTLQueryNode"));
    this->mrmlScene()->AddNode(d->IGTLConnectorNode); //node added cause the IGTLConnectorNode be initialized
    this->mrmlScene()->AddNode(d->IGTLRobotConnectorNode);
    this->mrmlScene()->AddNode(d->IGTLDataQueryNode);
    d->converter = vtkIGTLToMRMLDepthVideo::New();
    d->converter->SetIGTLName("ColoredDepth");
    d->IGTLConnectorNode->RegisterMessageConverter(d->converter);
    d->IGTLConnectorNode->SetConnectionTagName("Kinect");
    d->targetPosConverter = vtkIGTLToMRMLString::New();
    d->IGTLRobotConnectorNode->RegisterMessageConverter(d->targetPosConverter);
    d->IGTLRobotConnectorNode->SetConnectionTagName("Robot");
    qvtkReconnect( this->mrmlScene(), scene, vtkMRMLScene::NodeAddedEvent, this, SLOT( AddingNode(vtkObject*,vtkObject*) ) );
    d->RedSliceCompositionNode = vtkMRMLSliceCompositeNode::SafeDownCast(this->mrmlScene()->GetNodeByID("vtkMRMLSliceCompositeNodeRed"));
    if (d->IGTLConnectorNode)
    {
      // If the timer is not active
      if (!d->ImportDataAndEventsTimer.isActive())
      {
        d->ImportDataAndEventsTimer.start(5);
      }
    }
  }
}


void qSlicerKinectTrackingModuleWidget::SelectImageModel(vtkMRMLNode* node)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  vtkMRMLScalarVolumeNode* localNode = vtkMRMLScalarVolumeNode::SafeDownCast(node);
  if (localNode)
    d->logic()->SetImage(localNode->GetImageData());
}

void qSlicerKinectTrackingModuleWidget::AddingNode(vtkObject* sceneObject, vtkObject* nodeObject)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  vtkMRMLScene* scene = vtkMRMLScene::SafeDownCast(sceneObject);
  if (!scene || (strcmp(nodeObject->GetClassName(),"vtkMRMLScalarVolumeNode")==0))
  {
  // Connect segment added and removed events to plugin to update subject hierarchy accordingly
    vtkMRMLScalarVolumeNode* node = vtkMRMLScalarVolumeNode::SafeDownCast(nodeObject);
    if (node)
    {
      d->RedSliceCompositionNode->SetBackgroundVolumeID(node->GetID());
      SelectImageModel(node);
    }
  }
  if (!scene || (strcmp(nodeObject->GetClassName(),"vtkMRMLTransformNode")==0))
  {
  
    // Connect segment added and removed events to plugin to update subject hierarchy accordingly
    vtkMRMLTransformNode* node = vtkMRMLTransformNode::SafeDownCast(nodeObject);
    if (node)
    {
      vtkMRMLTransformNode* nodePre = vtkMRMLTransformNode::SafeDownCast(d->NodeSelectorTransform->currentNode());
      if(nodePre)
      {
        nodePre->SetDisplayVisibility(false);
      }
      d->NodeSelectorTransform->setCurrentNode(node);
      qvtkReconnect( node, vtkMRMLTransformNode::TransformModifiedEvent, this, SLOT( UpdateTransform(vtkObject*,vtkObject*) ) );
    }
  }
  
  if (!scene && (strcmp(nodeObject->GetClassName(),"vtkMRMLModelNode")==0))
  {
    
    // Connect segment added and removed events to plugin to update subject hierarchy accordingly
    vtkMRMLModelNode* node = vtkMRMLModelNode::SafeDownCast(nodeObject);
    if (node)
    {
      vtkMRMLModelNode* nodePre = vtkMRMLModelNode::SafeDownCast(d->NodeSelector->currentNode());
      if(nodePre)
      {
        nodePre->SetDisplayVisibility(false);
      }
      d->NodeSelector->setCurrentNode(node);
      qvtkReconnect( node, vtkMRMLModelNode::PolyDataModifiedEvent, this, SLOT( UpdateTargetModel(vtkObject*,vtkObject*) ) );
    }
  }
}


void qSlicerKinectTrackingModuleWidget::SelectModel(vtkMRMLNode* node)
{
  this->UpdateTargetModel(this->mrmlScene(), node);
}

void qSlicerKinectTrackingModuleWidget::SelectTransform(vtkMRMLNode* node)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  vtkMRMLTransformNode* transform = vtkMRMLTransformNode::SafeDownCast(node);
  if(transform)
  {
  vtkSlicerKinectTrackingLogic::SafeDownCast(d->logic())->ResetRobotToSlicerRegistration(transform->GetMatrixTransformToParent()); // check the matrix!!!!!
  }
}

void qSlicerKinectTrackingModuleWidget::UpdateTargetModel(vtkObject* sceneObject, vtkObject* nodeObject)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  vtkMRMLModelNode* node = vtkMRMLModelNode::SafeDownCast(nodeObject);
  if (node && node->GetPolyData())
  {
    QList<vtkMRMLNode*> nodes = d->NodeSelector->nodes();
    for (QList<vtkMRMLNode*>::iterator qNodeIter = nodes.begin(); qNodeIter< nodes.end(); qNodeIter ++)
    {
      vtkMRMLModelNode* qNodeObject = vtkMRMLModelNode::SafeDownCast(*(qNodeIter));
      if(qNodeObject)
        qNodeObject->SetDisplayVisibility(false);
    }
    node->SetDisplayVisibility(true);
    vtkSlicerKinectTrackingLogic::SafeDownCast(d->logic())->ResetTargetModel(node->GetPolyData());
  }
}

void qSlicerKinectTrackingModuleWidget::UpdateTransform(vtkObject* sceneObject, vtkObject* nodeObject)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  vtkMRMLTransformNode* node = vtkMRMLTransformNode::SafeDownCast(nodeObject);
  if (node && node->GetMatrixTransformToParent())
  {
    QList<vtkMRMLNode*> nodes = d->NodeSelectorTransform->nodes();
    for (QList<vtkMRMLNode*>::iterator qNodeIter = nodes.begin(); qNodeIter< nodes.end(); qNodeIter ++)
    {
      vtkMRMLTransformNode* qNodeObject = vtkMRMLTransformNode::SafeDownCast(*(qNodeIter));
      if(qNodeObject)
        qNodeObject->SetDisplayVisibility(false);
    }
    node->SetDisplayVisibility(true);
    vtkSlicerKinectTrackingLogic::SafeDownCast(d->logic())->ResetRobotToSlicerRegistration(node->GetMatrixTransformToParent());
  }
}

//------------------------------------------------------------------------------
void qSlicerKinectTrackingModuleWidget::setMRMLIGTLConnectorNode(vtkMRMLIGTLConnectorNode * connectorNode)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  qvtkReconnect(d->IGTLConnectorNode, connectorNode, vtkCommand::ModifiedEvent,
                this, SLOT(onMRMLNodeModified()));
  foreach(int evendId, QList<int>()
          << vtkMRMLIGTLConnectorNode::ActivatedEvent
          << vtkMRMLIGTLConnectorNode::ConnectedEvent
          << vtkMRMLIGTLConnectorNode::DisconnectedEvent
          << vtkMRMLIGTLConnectorNode::DeactivatedEvent)
  {
    qvtkReconnect(d->IGTLConnectorNode, connectorNode, evendId,
                  this, SLOT(onMRMLNodeModified()));
  }
  
  this->onMRMLNodeModified();
  this->setEnabled(connectorNode != 0);
}

//------------------------------------------------------------------------------
void qSlicerKinectTrackingModuleWidget::onMRMLNodeModified()
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  if (!d->IGTLConnectorNode)
  {
    return;
  }
  
  d->ConnectorPortEdit->setText(QString("%1").arg(d->IGTLConnectorNode->GetServerPort()));
  
  bool deactivated = d->IGTLConnectorNode->GetState() == vtkMRMLIGTLConnectorNode::STATE_OFF;
  d->ConnectorStateCheckBox->setChecked(!deactivated);
}

//------------------------------------------------------------------------------
void qSlicerKinectTrackingModuleWidget::startCurrentIGTLConnector(bool value)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  
  Q_ASSERT(d->IGTLConnectorNode);
  if (value)
  {
    d->IGTLConnectorNode->SetTypeClient(d->ConnectorHostAddressEdit->text().toStdString(), d->ConnectorPortEdit->text().toInt());
    
    bool success = false;
    int attemptTimes = 0;
    while(attemptTimes<10 && (not success) )
    {
      success = d->IGTLConnectorNode->Start();
      if (success)
        break;
      int milliseconds = 300;
      struct timespec req;
      req.tv_sec  = milliseconds / 1000;
      req.tv_nsec = (milliseconds % 1000) * 1000000;
      while ((nanosleep(&req, &req) == -1) && (errno == EINTR))
      {
        continue;
      }
      attemptTimes++;
    }
    if(not success)
    {
      d->ConnectorStateCheckBox->setCheckState(Qt::CheckState::Unchecked);
    }
  }
  else
  {
    d->IGTLConnectorNode->Stop();
  }
}

//------------------------------------------------------------------------------
void qSlicerKinectTrackingModuleWidget::startVideoTransmission(bool value)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  Q_ASSERT(d->IGTLConnectorNode);
  if(d->StartVideoCheckBox->checkState() == Qt::CheckState::Checked)
  {
    if (d->FrameFrequency->text().toInt()>0.0000001 && d->FrameFrequency->text().toInt()<1000000)
    {
      d->IGTLDataQueryNode->SetIGTLName("ColoredDepth");
      int interval = (int) (1000.0 / d->FrameFrequency->text().toInt());
      d->IGTLDataQueryNode->SetQueryType(d->IGTLDataQueryNode->TYPE_START);
      d->IGTLDataQueryNode->SetQueryStatus(d->IGTLDataQueryNode->STATUS_PREPARED);
      d->IGTLConnectorNode->setInterval(interval);
      d->IGTLConnectorNode->setUseCompress(d->UseCompressCheckBox->isChecked());
      d->IGTLConnectorNode->setRequireConversion(false);
      d->IGTLConnectorNode->PushQuery(d->IGTLDataQueryNode);
    }
  }
  else
  {
    d->IGTLDataQueryNode->SetIGTLName("ColoredDepth");
    d->IGTLDataQueryNode->SetQueryType(d->IGTLDataQueryNode->TYPE_STOP);
    d->IGTLDataQueryNode->SetQueryStatus(d->IGTLDataQueryNode->STATUS_PREPARED);
    d->IGTLConnectorNode->PushQuery(d->IGTLDataQueryNode);
  }
}

//------------------------------------------------------------------------------
void qSlicerKinectTrackingModuleWidget::updateIGTLConnectorNode()
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  if (d->IGTLConnectorNode != NULL && d->IGTLRobotConnectorNode != NULL)
  {
    d->IGTLConnectorNode->DisableModifiedEventOn();
    d->IGTLConnectorNode->SetServerHostname(d->ConnectorHostAddressEdit->text().toStdString());
    d->IGTLConnectorNode->SetServerPort(d->ConnectorPortEdit->text().toInt());
    d->IGTLConnectorNode->DisableModifiedEventOff();
    d->IGTLConnectorNode->InvokePendingModifiedEvent();
    d->IGTLRobotConnectorNode->DisableModifiedEventOn();
    d->IGTLRobotConnectorNode->SetServerHostname(d->ConnectorHostAddressEdit_2->text().toStdString());
    d->IGTLRobotConnectorNode->SetServerPort(d->ConnectorPortEdit_2->text().toInt());
    d->IGTLRobotConnectorNode->DisableModifiedEventOff();
    d->IGTLRobotConnectorNode->InvokePendingModifiedEvent();
  }
}

void qSlicerKinectTrackingModuleWidget::importDataAndEvents()
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  if (d->StartVideoCheckBox->checkState() == Qt::CheckState::Checked)
  {
    vtkMRMLAbstractLogic* l = this->logic();
    vtkSlicerKinectTrackingLogic * igtlLogic = vtkSlicerKinectTrackingLogic::SafeDownCast(l);
    if (igtlLogic)
    {
      int64_t startTime = Connector::getTime();
      if (d->IGTLConnectorNode->STATE_CONNECTED)
      {
        igtlLogic->CallConnectorTimerHander();
        d->polydata = igtlLogic->GetPolyData();
        d->polyDataOverlay = igtlLogic->GetPolyDataOverlay();
        
      }
      else
      {
        d->polydata = igtlLogic->GetPolyData();
      }
      //-------------------
      // Convert the image in p_PixmapConversionBuffer to a QPixmap
      if (d->polydata && igtlLogic->GetFrame())
      {
        memcpy(d->RGBFrame, igtlLogic->GetFrame(), d->picWidth*d->picHeight*3);
        int64_t renderingTime = Connector::getTime();
        d->mapper->SetInputData(d->polydata);
        d->sliceMapper->SetInputData(d->polyDataOverlay);
        d->PolyDataRenderer->GetRenderWindow()->Render();
        d->graphicsView->setRGBFrame(d->RGBFrame);
        d->graphicsView->update();
        std::cerr<<"Rendering Time: "<<(Connector::getTime()-renderingTime)/1e6 << std::endl;
      }
      if(d->IGTLRobotConnectorNode->STATE_CONNECTED)
      {
        char str[30];
        sprintf(str, "%.1f %.1f %.1f",igtlLogic->targetInRobotCoord[0],igtlLogic->targetInRobotCoord[1],igtlLogic->targetInRobotCoord[2] );
        igtl::StringMessage::Pointer msg = igtl::StringMessage::New();
        msg->AllocatePack();
        msg->SetString(str);
        msg->Pack();
        d->IGTLRobotConnectorNode->SendData(msg->GetPackSize(), (unsigned char*)msg->GetPackPointer());
      }
    }
  }
}
