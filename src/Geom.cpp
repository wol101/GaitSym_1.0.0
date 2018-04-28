/*
 *  Geom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Wrapper class to hold ODE geom

#include <ode/ode.h>

#include <iostream>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif


#include "Geom.h"
#include "PGDMath.h"
#include "DataFile.h"
#include "Simulation.h"
#include "Body.h"

// Simulation global
extern Simulation *gSimulation;



Geom::Geom()
{
    m_GeomID = 0;
    m_GeomTransformID = 0;
    m_GeomLocation = environment;
    m_CFM = -1; // < 0 is not used
    m_ERP = 2; // > 1 is not used
    m_Bounce = -1; // < 0 is not used
    m_Mu = dInfinity;
    m_Abort = false;
}

Geom::~Geom()
{
    if (m_GeomID) dGeomDestroy(m_GeomID);
    if (m_GeomTransformID) dGeomDestroy(m_GeomTransformID);    
}


// these functions set the geom position relative to its body
// by setting using the GeomTransfer as a container
void Geom::SetBody(dBodyID setBody)
{
    dGeomSetBody(m_GeomTransformID, setBody);
}

dBodyID Geom::GetBody()
{
    return dGeomGetBody(m_GeomTransformID);
}

void Geom::SetPosition (dReal x, dReal y, dReal z)
{
    dGeomSetPosition(m_GeomID, x, y, z); 
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void
Geom::SetPosition(const char *buf)
{
	int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos, result;
    dBodyID geomBody = dGeomGetBody(m_GeomTransformID);
    
    strcpy(lBuf, buf);
	int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
	if (count < 3) 
    {
        std::cerr << "Error in Geom::SetPosition\n";
        return; // error condition
    }
	
	if (isalpha((int)*lBufPtrs[0]) == 0) 
	{
		for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(geomBody, pos[0], pos[1], pos[2], result); // convert from world to body
        SetPosition(result[0], result[1], result[2]);
		return;
	}
	
	if (count < 4)
    {
        std::cerr << "Error in Geom::SetPosition\n";
        return; // error condition
    }
	Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
	if (theBody == 0) 
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(geomBody, pos[0], pos[1], pos[2], result); // convert from world to body
            SetPosition(result[0], result[1], result[2]);
            return;
        }
        else
        {
            std::cerr << "Error in Geom::SetPosition\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(geomBody, result[0], result[1], result[2], pos); // convert from world to body
    SetPosition(pos[0], pos[1], pos[2]);
}

const dReal *Geom::GetPosition()
{
    return dGeomGetPosition(m_GeomID); 
}

void Geom::SetQuaternion(dReal q0, dReal q1, dReal q2, dReal q3)
{
    dQuaternion q;
    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
    dGeomSetQuaternion(m_GeomID, q);
}

// parses the quaternion allowing a relative position specified by BODY ID
// note quaternion is (qs,qx,qy,qz)
// s x y z - world coordinates
// bodyName s x y z - position relative to bodyName local coordinate system
void
Geom::SetQuaternion(const char *buf)
{
	int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dQuaternion quaternion, q;
    
    strcpy(lBuf, buf);
	int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
	if (count < 4)     
    {
        std::cerr << "Error in Geom::SetQuaternion\n";
        return; // error condition
    }
    
    
	if (isalpha((int)*lBufPtrs[0]) == 0) 
	{
        dGeomGetQuaternion(m_GeomID, q); 
        pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
		for (i = 0; i < 4; i++) quaternion[i] = strtod(lBufPtrs[i], 0);
        pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        pgd::Quaternion qLocal = ~qBody * qWorld;
        SetQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
		return;
	}
	
	if (count < 5) 
    {
        std::cerr << "Error in Geom::SetQuaternion\n";
        return; // error condition
    }
	Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
	if (theBody == 0)
	{
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            dGeomGetQuaternion(m_GeomID, q); 
            pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
            for (i = 0; i < 4; i++) quaternion[i] = strtod(lBufPtrs[i + 1], 0);
            pgd::Quaternion qWorld(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            pgd::Quaternion qLocal = ~qBody * qWorld;
            SetQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
            return;
        }
        else
        {
            std::cerr << "Error in Geom::SetQuaternion\n";
            return; // error condition
        }
	}
    // first get world quaternion
    const dReal *q2 = theBody->GetQuaternion();
    pgd::Quaternion qBody1(q2[0], q2[1], q2[2], q2[3]);
    for (i = 0; i < 4; i++) quaternion[i] = strtod(lBufPtrs[i + 1], 0);
    pgd::Quaternion qBody2(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    pgd::Quaternion qWorld = qBody1 * qBody2;
    
    // then set the local quaternion
    dGeomGetQuaternion(m_GeomID, q); 
    pgd::Quaternion qBody(q[0], q[1], q[2], q[3]);
    pgd::Quaternion qLocal = ~qBody * qWorld;
    SetQuaternion(qLocal.n, qLocal.v.x, qLocal.v.y, qLocal.v.z);
}

void Geom::GetQuaternion(dQuaternion q)
{
    dGeomGetQuaternion(m_GeomID, q);
}

void Geom::SetSpringDamp(dReal springConstant, dReal dampingConstant, dReal integrationStep)
{
    m_ERP = integrationStep * springConstant/(integrationStep * springConstant + dampingConstant);
    m_CFM = 1/(integrationStep * springConstant + dampingConstant);
}

void Geom::SetSpringERP(dReal springConstant, dReal ERP, dReal integrationStep)
{
    m_ERP = ERP;
    m_CFM = ERP / (integrationStep * springConstant);
}
