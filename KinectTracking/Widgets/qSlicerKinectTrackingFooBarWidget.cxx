/*==============================================================================

  Program: 3D Slicer

  Copyright (c) Kitware Inc.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc.
  and was partially funded by NIH grant 3P41RR013218-12S1

==============================================================================*/

// FooBar Widgets includes
#include "qSlicerKinectTrackingFooBarWidget.h"
#include "ui_qSlicerKinectTrackingFooBarWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_KinectTracking
class qSlicerKinectTrackingFooBarWidgetPrivate
  : public Ui_qSlicerKinectTrackingFooBarWidget
{
  Q_DECLARE_PUBLIC(qSlicerKinectTrackingFooBarWidget);
protected:
  qSlicerKinectTrackingFooBarWidget* const q_ptr;

public:
  qSlicerKinectTrackingFooBarWidgetPrivate(
    qSlicerKinectTrackingFooBarWidget& object);
  virtual void setupUi(qSlicerKinectTrackingFooBarWidget*);
};

// --------------------------------------------------------------------------
qSlicerKinectTrackingFooBarWidgetPrivate
::qSlicerKinectTrackingFooBarWidgetPrivate(
  qSlicerKinectTrackingFooBarWidget& object)
  : q_ptr(&object)
{
}

// --------------------------------------------------------------------------
void qSlicerKinectTrackingFooBarWidgetPrivate
::setupUi(qSlicerKinectTrackingFooBarWidget* widget)
{
  this->Ui_qSlicerKinectTrackingFooBarWidget::setupUi(widget);
}

//-----------------------------------------------------------------------------
// qSlicerKinectTrackingFooBarWidget methods

//-----------------------------------------------------------------------------
qSlicerKinectTrackingFooBarWidget
::qSlicerKinectTrackingFooBarWidget(QWidget* parentWidget)
  : Superclass( parentWidget )
  , d_ptr( new qSlicerKinectTrackingFooBarWidgetPrivate(*this) )
{
  Q_D(qSlicerKinectTrackingFooBarWidget);
  d->setupUi(this);
}

//-----------------------------------------------------------------------------
qSlicerKinectTrackingFooBarWidget
::~qSlicerKinectTrackingFooBarWidget()
{
}
