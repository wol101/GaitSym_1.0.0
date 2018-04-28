/*
 *  Driver.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat May 22 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 * Virtual class that all drivers descend from
 */

#ifndef Driver_h
#define Driver_h

#include "NamedObject.h"

class Muscle;

class Driver: public NamedObject
{
public:
    Driver() { m_Target = 0; };
    virtual ~Driver() {};
    
    void SetTarget(Muscle *muscle) { m_Target = muscle; };
    Muscle *GetTarget() { return m_Target; };
    
    virtual double GetValue(double time) = 0;

protected:

    Muscle *m_Target;
    
};

#endif
