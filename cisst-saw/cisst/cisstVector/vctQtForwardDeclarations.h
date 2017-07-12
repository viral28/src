/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-04-16

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#pragma once
#ifndef _vctQtForwardDeclarations_h
#define _vctQtForwardDeclarations_h

template <class _elementType>
class vctQtWidgetDynamicVectorReadFloating;
template <class _elementType>
class vctQtWidgetDynamicVectorReadInteger;

typedef vctQtWidgetDynamicVectorReadFloating<double> vctQtWidgetDynamicVectorDoubleRead;
typedef vctQtWidgetDynamicVectorReadFloating<float> vctQtWidgetDynamicVectorFloatRead;
typedef vctQtWidgetDynamicVectorReadInteger<int> vctQtWidgetDynamicVectorIntRead;
typedef vctQtWidgetDynamicVectorReadInteger<unsigned int> vctQtWidgetDynamicVectorUIntRead;
typedef vctQtWidgetDynamicVectorReadInteger<bool> vctQtWidgetDynamicVectorBoolRead;

template <class _elementType>
class vctQtWidgetDynamicVectorWriteFloating;
template <class _elementType>
class vctQtWidgetDynamicVectorWriteInteger;

typedef vctQtWidgetDynamicVectorWriteFloating<double> vctQtWidgetDynamicVectorDoubleWrite;
typedef vctQtWidgetDynamicVectorWriteFloating<float> vctQtWidgetDynamicVectorFloatWrite;
typedef vctQtWidgetDynamicVectorWriteInteger<int> vctQtWidgetDynamicVectorIntWrite;
typedef vctQtWidgetDynamicVectorWriteInteger<unsigned int> vctQtWidgetDynamicVectorUIntWrite;


class vctQtWidgetDynamicMatrixDoubleRead;

class vctQtWidgetRotationDoubleRead;
class vctQtWidgetFrameDoubleRead;

#endif  // _vctQtForwardDeclarations_h
