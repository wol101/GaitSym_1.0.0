/*
 *  NamedObject.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Root object that allows some basic data storage such as naming

#include "NamedObject.h"

NamedObject::NamedObject()
{
#ifdef USE_OPENGL
    m_AxisSize[0] = m_AxisSize[1] = m_AxisSize[2] = 1;
    m_Colour.SetColour(1, 1, 1, 1);
#endif
}

NamedObject::~NamedObject()
{
}


