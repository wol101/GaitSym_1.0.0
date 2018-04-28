/*
 *  TwoPointStrap.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <iostream>

#include <math.h>
#include <string.h>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif

#include "TwoPointStrap.h"
#include "Body.h"
#include "PGDMath.h"
#include "DataFile.h"
#include "Simulation.h"

#ifdef USE_OPENGL
extern int gDrawMuscles;
extern int gDrawMuscleForces;
#endif

// Simulation global
extern Simulation *gSimulation;

TwoPointStrap::TwoPointStrap()
{
    PointForce *origin = new PointForce();
    PointForce *insertion = new PointForce();
    m_PointForceList.push_back(origin);
    m_PointForceList.push_back(insertion);
}

TwoPointStrap::~TwoPointStrap()
{
}

void TwoPointStrap::SetOrigin(Body *body, dVector3 point)
{
    m_OriginBody = body;
    m_PointForceList[0]->body = m_OriginBody;
    memcpy(m_Origin, point, sizeof(m_Origin));
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void
TwoPointStrap::SetOrigin(Body *body, const char *buf)
{
	int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos, result;
    
    strcpy(lBuf, buf);
	int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
	if (count < 3) 
    {
        std::cerr << "Error in TwoPointStrap::SetOrigin\n";
        return; // error condition
    }
    
	
	if (isalpha((int)*lBufPtrs[0]) == 0) 
	{
		for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetOrigin(body, result);
		return;
	}
	
	if (count < 4) 
    {
        std::cerr << "Error in TwoPointStrap::SetOrigin\n";
        return; // error condition
    }
	Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
	if (theBody == 0)
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetOrigin(body, result);
            return;
        }
        else
        {
            std::cerr << "Error in TwoPointStrap::SetOrigin\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetOrigin(body, pos);
}

void TwoPointStrap::SetInsertion(Body *body, dVector3 point)
{
    m_InsertionBody = body;
    m_PointForceList[1]->body = m_InsertionBody;
    memcpy(m_Insertion, point, sizeof(m_Insertion));
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void
TwoPointStrap::SetInsertion(Body *body, const char *buf)
{
	int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos, result;
    
    strcpy(lBuf, buf);
	int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
	if (count < 3) 
    {
        std::cerr << "Error in TwoPointStrap::SetInsertion\n";
        return; // error condition
    }
	
	if (isalpha((int)*lBufPtrs[0]) == 0) 
	{
		for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetInsertion(body, result);
		return;
	}
	
	if (count < 4)
    {
        std::cerr << "Error in TwoPointStrap::SetInsertion\n";
        return; // error condition
    }
	Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
	if (theBody == 0) 
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetInsertion(body, result);
            return;
        }
        else
        {
            std::cerr << "Error in TwoPointStrap::SetInsertion\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetInsertion(body, pos);
}

void TwoPointStrap::Calculate(dReal deltaT)
{
    PointForce *theOrigin = m_PointForceList[0];
    PointForce *theInsertion = m_PointForceList[1];    
    
    // calculate the world positions
    dBodyGetRelPointPos(m_OriginBody->GetBodyID(), m_Origin[0], m_Origin[1], m_Origin[2], 
                        theOrigin->point);
    dBodyGetRelPointPos(m_InsertionBody->GetBodyID(), m_Insertion[0], m_Insertion[1], m_Insertion[2], 
                        theInsertion->point);
    
    // calculate the vector from the origin to the insertion
    dVector3 line;
    line[0] = theInsertion->point[0] - theOrigin->point[0];
    line[1] = theInsertion->point[1] - theOrigin->point[1];
    line[2] = theInsertion->point[2] - theOrigin->point[2];
    
    // calculate the length and velocity
    m_LastLength = m_Length;
    m_Length = sqrt(line[0]*line[0] + line[1]*line[1] + line[2]*line[2]);
    if (deltaT != 0.0) m_Velocity = (m_Length - m_LastLength) / deltaT;
    else m_Velocity = 0;
    
    // normalise the direction vector
    line[0] /= m_Length;
    line[1] /= m_Length;
    line[2] /= m_Length;
    
    memcpy(theOrigin->vector, line, sizeof(theOrigin->vector));
    
    // simply reverse the direction for the insertion
    line[0] = -line[0];
    line[1] = -line[1];
    line[2] = -line[2];
    
    memcpy(theInsertion->vector, line, sizeof(theInsertion->vector));
}

#ifdef USE_OPENGL
void
TwoPointStrap::Draw()
{
    // draw strap force as a single line
    
    if (gDrawMuscles)
    {
        pgd::Vector path[2];
        path[0].x = m_PointForceList[0]->point[0];
        path[0].y = m_PointForceList[0]->point[1];
        path[0].z = m_PointForceList[0]->point[2];
        path[1].x = m_PointForceList[1]->point[0];
        path[1].y = m_PointForceList[1]->point[1];
        path[1].z = m_PointForceList[1]->point[2];
        GLUtils::DrawPath(path, 2, m_Radius, m_Colour);
         
        if (gDrawMuscleForces)
        {
            GLUtils::DrawCylinder(m_PointForceList[0]->point, m_PointForceList[0]->vector, m_Tension * m_ForceScale, m_ForceRadius, m_ForceColour);
            GLUtils::DrawCylinder(m_PointForceList[1]->point, m_PointForceList[1]->vector, m_Tension * m_ForceScale, m_ForceRadius, m_ForceColour);
        }
    }
}
#endif



