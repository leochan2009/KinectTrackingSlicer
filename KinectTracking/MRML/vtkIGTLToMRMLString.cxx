/*==========================================================================

  Portions (c) Copyright 2008-2014 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See Doc/copyright/copyright.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer
  Contributor: Tamas Ungi at Queen's University.

==========================================================================*/

// OpenIGTLinkIF MRML includes
#include "vtkIGTLToMRMLString.h"
#include "vtkMRMLIGTLQueryNode.h"
#include "vtkMRMLTextNode.h"

// VTK includes
#include "vtkCommand.h"
#include "vtkIntArray.h"
#include "vtkObjectFactory.h"

//---------------------------------------------------------------------------
vtkStandardNewMacro(vtkIGTLToMRMLString);

//---------------------------------------------------------------------------
vtkIGTLToMRMLString
::vtkIGTLToMRMLString()
{
  this->IGTLNames.clear();
}

//---------------------------------------------------------------------------
vtkIGTLToMRMLString
::~vtkIGTLToMRMLString()
{
}

//---------------------------------------------------------------------------
void vtkIGTLToMRMLString::PrintSelf( ostream& os, vtkIndent indent )
{
  Superclass::PrintSelf(os, indent);
}

//---------------------------------------------------------------------------
vtkMRMLNode* vtkIGTLToMRMLString
::CreateNewNode( vtkMRMLScene* scene, const char* name )
{
  vtkSmartPointer< vtkMRMLTextNode > textNode = vtkSmartPointer< vtkMRMLTextNode >::New();
  textNode->SetName( name );
  textNode->SetDescription( "Created by OpenIGTLinkIF module" );

  scene->AddNode( textNode );

  return textNode;
}

//---------------------------------------------------------------------------
vtkIntArray* vtkIGTLToMRMLString
::GetNodeEvents()
{
  vtkIntArray* events;

  events = vtkIntArray::New();
  events->InsertNextValue( vtkCommand::ModifiedEvent );

  return events;
}

//---------------------------------------------------------------------------
int vtkIGTLToMRMLString
::MRMLToIGTL( unsigned long event, vtkMRMLNode* mrmlNode, int* size, void** igtlMsg )
{
  if ( mrmlNode == NULL )
    {
    vtkErrorMacro("vtkIGTLToMRMLString::MRMLToIGTL failed: invalid input MRML node");
    return 0;
    }

  const char* deviceName = NULL;
  const char* text = NULL;
  int encoding = vtkMRMLTextNode::ENCODING_US_ASCII;

  vtkMRMLTextNode* textNode = vtkMRMLTextNode::SafeDownCast( mrmlNode );
  vtkMRMLIGTLQueryNode* queryNode = vtkMRMLIGTLQueryNode::SafeDownCast( mrmlNode );
  if ( textNode != NULL && event == vtkCommand::ModifiedEvent)
    {
    deviceName = textNode->GetName();
    text = textNode->GetText();
    encoding = textNode->GetEncoding();
    }
  else if ( queryNode != NULL )
    {
    // Special case for STRING command handling.
    // The command is a regular STRING message with special device name (CMD_...).
    // Note that the query node has a name that matches the response node name (ACK_...),
    // as it is for detecting the arrival of the response.
    deviceName = queryNode->GetAttribute("CommandDeviceName");
    text = queryNode->GetAttribute("CommandString");
    }

  if (deviceName!=NULL && text!=NULL)
    {
    if (this->StringMsg.GetPointer()==NULL)
      {
      this->StringMsg = igtl::StringMessage::New();
      }
    this->StringMsg->SetDeviceName( deviceName );
    this->StringMsg->SetString( text );
    this->StringMsg->SetEncoding( encoding );
    this->StringMsg->Pack();
    *size = this->StringMsg->GetPackSize();
    *igtlMsg = (void*)this->StringMsg->GetPackPointer();
    return 1;
    }

  return 0;
}

float*  vtkIGTLToMRMLString::GetFloatValue( igtl::MessageBase::Pointer buffer)
{
  igtl::StringMessage::Pointer stringMessage;
  stringMessage = igtl::StringMessage::New();
  stringMessage->Copy( buffer );
  
  int c = stringMessage->Unpack( this->CheckCRC );
  
  if ( ! ( c & igtl::MessageHeader::UNPACK_BODY ) )
  {
    vtkErrorMacro( "Incoming IGTL string message failed CRC check!" );
    return 0;
  }
  return NULL;
}

