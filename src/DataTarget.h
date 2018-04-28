/*
 *  DataTarget.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat May 22 2004.
 *  Copyright (c) 2004 Bill Sellers. All rights reserved.
 *
 */

#ifndef DataTarget_h
#define DataTarget_h

#include "NamedObject.h"

class Body;

class DataTarget: public NamedObject
{
public:
    DataTarget();
    ~DataTarget();
    
    enum DataType
    {
        XP,
        YP,
        ZP,
        Q0,
        Q1,
        Q2,
        Q3,
        XV,
        YV,
        ZV,
        XRV,
        YRV,
        ZRV
    };
    
    void SetValueDurationPairs(int size, dReal *valueDurationPairs);
    void SetWeight(dReal w) { m_Weight = w; };
    void SetTarget(Body *target) { m_Target = target; };
    void SetDataType(DataType dataType) { m_DataType = dataType; };
    
    bool GetTargetValue(dReal time, dReal *target, dReal *weight);
    Body *GetTarget() { return m_Target; };
    DataType GetDataType() { return m_DataType; };
    
protected:
        dReal *m_ValueList;
    dReal *m_DurationList;
    int m_ListLength;
    int m_LastIndex;
    dReal m_Weight;
    Body *m_Target;
    DataType m_DataType;
};

#endif

