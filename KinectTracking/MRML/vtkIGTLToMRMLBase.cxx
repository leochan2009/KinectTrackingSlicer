/*==========================================================================

  Portions (c) Copyright 2008-2009 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Module:    $HeadURL: http://svn.slicer.org/Slicer3/trunk/Modules/OpenIGTLinkIF/vtkIGTLToMRMLBase.cxx $
  Date:      $Date: 2009-10-05 17:19:02 -0400 (Mon, 05 Oct 2009) $
  Version:   $Revision: 10576 $

==========================================================================*/

// OpenIGTLink includes
#include <igtlMessageBase.h>

// OpenIGTLinkIF MRML includes
#include "vtkIGTLToMRMLBase.h"

#include "vtkSlicerKinectTrackingLogic.h"

// VTK includes
#include <vtkObjectFactory.h>

// VTKSYS includes
#include <vtksys/SystemTools.hxx>

// STD includes
#include <string>

//---------------------------------------------------------------------------
vtkStandardNewMacro(vtkIGTLToMRMLBase);

//---------------------------------------------------------------------------
class vtkIGTLToMRMLBasePrivate
{
public:
  vtkIGTLToMRMLBasePrivate();
  ~vtkIGTLToMRMLBasePrivate();

  void SetKinectTrackingLogic(vtkSlicerKinectTrackingLogic* logic);
  vtkSlicerKinectTrackingLogic* GetKinectTrackingLogic();

protected:
  vtkSlicerKinectTrackingLogic* KinectTrackingLogic;
};

vtkIGTLToMRMLBasePrivate::vtkIGTLToMRMLBasePrivate()
{
  this->KinectTrackingLogic = NULL;
}

vtkIGTLToMRMLBasePrivate::~vtkIGTLToMRMLBasePrivate()
{
}

void vtkIGTLToMRMLBasePrivate::SetKinectTrackingLogic(vtkSlicerKinectTrackingLogic* logic)
{
  this->KinectTrackingLogic = logic;
}


vtkSlicerKinectTrackingLogic* vtkIGTLToMRMLBasePrivate::GetKinectTrackingLogic()
{
  return this->KinectTrackingLogic;
}


//---------------------------------------------------------------------------
vtkIGTLToMRMLBase::vtkIGTLToMRMLBase()
{
  this->CheckCRC = 1;
  this->Private = new vtkIGTLToMRMLBasePrivate;
}

//---------------------------------------------------------------------------
vtkIGTLToMRMLBase::~vtkIGTLToMRMLBase()
{
  if (this->Private)
    {
    delete this->Private;
    }
}

//---------------------------------------------------------------------------
void vtkIGTLToMRMLBase::PrintSelf(ostream& os, vtkIndent indent)
{
  this->vtkObject::PrintSelf(os, indent);
}


//---------------------------------------------------------------------------
void vtkIGTLToMRMLBase::SetKinectTrackingLogic(vtkSlicerKinectTrackingLogic* logic)
{
  if (this->Private)
    {
    this->Private->SetKinectTrackingLogic(logic);
    }
}


//---------------------------------------------------------------------------
vtkSlicerKinectTrackingLogic* vtkIGTLToMRMLBase::GetKinectTrackingLogic()
{
  if (this->Private)
    {
    return this->Private->GetKinectTrackingLogic();
    }
  else
    {
    return NULL;
    }
}

//---------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData>  vtkIGTLToMRMLBase::IGTLToMRML(igtl::MessageBase::Pointer buffer, vtkMRMLNode* node)
{
  if(buffer && node)
    {
      igtlUint32 second;
      igtlUint32 nanosecond;
      
      buffer->GetTimeStamp(&second, &nanosecond);
      
      std::stringstream ss;
      ss << second << nanosecond;
      
      node->SetAttribute("Timestamp",ss.str().c_str());
    }
  return 0;
}


//---------------------------------------------------------------------------
uint8_t * vtkIGTLToMRMLBase::IGTLToMRML(igtl::MessageBase::Pointer buffer)
{
  return NULL;
}
