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

// Logic Include
#include "vtkSlicerKinectTrackingLogic.h"

// OpenIGTLinkIF MRML includes
#include "vtkMRMLIGTLConnectorNode.h"
#include "vtkIGTLToMRMLDepthVideo.h"
#include "vtkMRMLIGTLQueryNode.h"
#include <qMRMLNodeFactory.h>
#include <vtkMRMLModelNode.h>

//VTK include
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>

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
  vtkMRMLIGTLQueryNode * IGTLDataQueryNode;
  vtkIGTLToMRMLDepthVideo* converter;
  QTimer ImportDataAndEventsTimer;
  vtkSlicerKinectTrackingLogic * logic();
  vtkRenderer* activeRenderer;
  vtkRenderer*   PolyDataRenderer;
  vtkSmartPointer<vtkActor> PolyDataActor;
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkPolyDataMapper> mapper;
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
  QObject::connect(d->ConnectorStateCheckBox, SIGNAL(toggled(bool)),
                   this, SLOT(startCurrentIGTLConnector(bool)));
  QObject::connect(d->StartVideoCheckBox, SIGNAL(toggled(bool)),
                   this, SLOT(startVideoTransmission(bool)));
  QObject::connect(d->NodeSelector, SIGNAL(currentNodeChanged(vtkMRMLNode*)),
                   this, SLOT(UpdateTargetModel(vtkMRMLNode*)));
  qSlicerApplication *  app = qSlicerApplication::application();
  vtkRenderer* activeRenderer = app->layoutManager()->activeThreeDRenderer();
  d->PolyDataRenderer = activeRenderer;
  vtkRenderWindow* activeRenderWindow = activeRenderer->GetRenderWindow();
  d->polydata = vtkSmartPointer<vtkPolyData>::New();
  if (activeRenderer)
  {
    d->mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    d->mapper->SetInputData(d->polydata);
    d->PolyDataActor = vtkSmartPointer<vtkActor>::New();
    d->PolyDataActor->SetMapper(d->mapper);
    d->PolyDataRenderer->AddActor(d->PolyDataActor);
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
    d->IGTLDataQueryNode = vtkMRMLIGTLQueryNode::SafeDownCast(this->mrmlScene()->CreateNodeByClass("vtkMRMLIGTLQueryNode"));
    this->mrmlScene()->AddNode(d->IGTLConnectorNode); //node added cause the IGTLConnectorNode be initialized
    this->mrmlScene()->AddNode(d->IGTLDataQueryNode);
    d->converter = vtkIGTLToMRMLDepthVideo::New();
    d->converter->SetIGTLName("ColoredDepth");
    d->IGTLConnectorNode->RegisterMessageConverter(d->converter);
    qvtkReconnect( this->mrmlScene(), scene, vtkMRMLScene::NodeAddedEvent, this, SLOT( AddingTargetModel(vtkObject*,vtkObject*) ) );
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



void qSlicerKinectTrackingModuleWidget::AddingTargetModel(vtkObject* sceneObject, vtkObject* nodeObject)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  vtkMRMLScene* scene = vtkMRMLScene::SafeDownCast(sceneObject);
  if (!scene)
  {
    return;
  }
  
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
  }
}



void qSlicerKinectTrackingModuleWidget::UpdateTargetModel(vtkMRMLNode* selectedNode)
{
  Q_D(qSlicerKinectTrackingModuleWidget);
  vtkMRMLModelNode* node = vtkMRMLModelNode::SafeDownCast(selectedNode);
  if (node && node->GetPolyData())
  {
    vtkSlicerKinectTrackingLogic::SafeDownCast(d->logic())->ResetTargetModel(node->GetPolyData());
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
  if (d->IGTLConnectorNode != NULL)
  {
    d->IGTLConnectorNode->DisableModifiedEventOn();
    d->IGTLConnectorNode->SetServerHostname(d->ConnectorHostAddressEdit->text().toStdString());
    d->IGTLConnectorNode->SetServerPort(d->ConnectorPortEdit->text().toInt());
    d->IGTLConnectorNode->DisableModifiedEventOff();
    d->IGTLConnectorNode->InvokePendingModifiedEvent();
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
        d->polydata = igtlLogic->CallConnectorTimerHander();
      }
      else
      {
        d->polydata = igtlLogic->polyData;
      }
      //-------------------
      // Convert the image in p_PixmapConversionBuffer to a QPixmap
      if (d->polydata)
      {
        memcpy(d->RGBFrame, igtlLogic->GetFrame(), d->picWidth*d->picHeight*3);
        int64_t renderingTime = Connector::getTime();
        d->mapper->SetInputData(d->polydata);
        d->PolyDataRenderer->GetRenderWindow()->Render();
        d->graphicsView->setRGBFrame(d->RGBFrame);
        d->graphicsView->update();
        std::cerr<<"Rendering Time: "<<(Connector::getTime()-renderingTime)/1e6 << std::endl;
      }
    }
  }
}
