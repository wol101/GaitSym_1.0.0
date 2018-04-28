/*
 *  ThreePointStrap.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>

#include <math.h>
#include <string.h>
#include <iostream>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif

#include "ThreePointStrap.h"
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

ThreePointStrap::ThreePointStrap(): TwoPointStrap()
{
    PointForce *midPoint = new PointForce();
    m_PointForceList.push_back(midPoint);
}

ThreePointStrap::~ThreePointStrap()
{
}

void ThreePointStrap::SetMidpoint(Body *body, dVector3 point)
{
    m_MidpointBody = body;
    m_PointForceList[2]->body = m_MidpointBody;
    memcpy(m_Midpoint, point, sizeof(m_Midpoint));
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void
ThreePointStrap::SetMidpoint(Body *body, const char *buf)
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
        std::cerr << "Error in ThreePointStrap::SetMidpoint\n";
        return; // error condition
    }
	
	if (isalpha((int)*lBufPtrs[0]) == 0) 
	{
		for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
        SetMidpoint(body, result);
		return;
	}
	
	if (count < 4) 
    {
        std::cerr << "Error in ThreePointStrap::SetMidpoint\n";
        return; // error condition
    }
	Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
	if (theBody == 0) 
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dBodyGetPosRelPoint(body->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from world to body
            SetMidpoint(body, result);
            return;
        }
        else
        {
            std::cerr << "Error in ThreePointStrap::SetMidpoint\n";
            return; // error condition
        }
    }
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos(theBody->GetBodyID(), pos[0], pos[1], pos[2], result); // convert from body to world
    dBodyGetPosRelPoint(body->GetBodyID(), result[0], result[1], result[2], pos); // convert from world to body
    SetMidpoint(body, pos);
}

void ThreePointStrap::Calculate(dReal deltaT)
{
    PointForce *theOrigin = m_PointForceList[0];
    PointForce *theInsertion = m_PointForceList[1];    
    PointForce *theMidpoint = m_PointForceList[2];    
    
    // calculate the world positions
    dBodyGetRelPointPos(m_OriginBody->GetBodyID(), m_Origin[0], m_Origin[1], m_Origin[2], 
                        theOrigin->point);
    dBodyGetRelPointPos(m_InsertionBody->GetBodyID(), m_Insertion[0], m_Insertion[1], m_Insertion[2], 
                        theInsertion->point);
    dBodyGetRelPointPos(m_MidpointBody->GetBodyID(), m_Midpoint[0], m_Midpoint[1], m_Midpoint[2], 
                        theMidpoint->point);
    
    // calculate the two vectors from the endpoints to the midpoint
    dVector3 line1, line2;
    line1[0] = theMidpoint->point[0] - theOrigin->point[0];
    line1[1] = theMidpoint->point[1] - theOrigin->point[1];
    line1[2] = theMidpoint->point[2] - theOrigin->point[2];
    line2[0] = theMidpoint->point[0] - theInsertion->point[0];
    line2[1] = theMidpoint->point[1] - theInsertion->point[1];
    line2[2] = theMidpoint->point[2] - theInsertion->point[2];
    
    // calculate the length and velocity
    m_LastLength = m_Length;
    dReal length1 = sqrt(line1[0]*line1[0] + line1[1]*line1[1] + line1[2]*line1[2]);
    dReal length2 = sqrt(line2[0]*line2[0] + line2[1]*line2[1] + line2[2]*line2[2]);
    m_Length = length1 + length2;
    if (deltaT != 0.0) m_Velocity = (m_Length - m_LastLength) / deltaT;
    else m_Velocity = 0;
        
    // normalise the direction vectors
    line1[0] /= length1;
    line1[1] /= length1;
    line1[2] /= length1;
    line2[0] /= length2;
    line2[1] /= length2;
    line2[2] /= length2;
        
    // now calculate the midpoint vector
    dVector3 midpoint;
    midpoint[0] = -line2[0] - line1[0];
    midpoint[1] = -line2[1] - line1[1];
    midpoint[2] = -line2[2] - line1[2];
    
    // set the origin, insertion and midpoint vectors
    memcpy(theOrigin->vector, line1, sizeof(theOrigin->vector));
    memcpy(theInsertion->vector, line2, sizeof(theInsertion->vector));
    memcpy(theMidpoint->vector, midpoint, sizeof(theMidpoint->vector));
}

#ifdef USE_OPENGL
void
ThreePointStrap::Draw()
{
    // draw strap force as a single line
    
    if (gDrawMuscles)
    {
        pgd::Vector path[3];
        path[0].x = m_PointForceList[0]->point[0];
        path[0].y = m_PointForceList[0]->point[1];
        path[0].z = m_PointForceList[0]->point[2];
        path[1].x = m_PointForceList[2]->point[0];
        path[1].y = m_PointForceList[2]->point[1];
        path[1].z = m_PointForceList[2]->point[2];
        path[2].x = m_PointForceList[1]->point[0];
        path[2].y = m_PointForceList[1]->point[1];
        path[2].z = m_PointForceList[1]->point[2];
        GLUtils::DrawPath(path, 3, m_Radius, m_Colour);
        
        if (gDrawMuscleForces)
        {
            GLUtils::DrawCylinder(m_PointForceList[0]->point, m_PointForceList[0]->vector, m_Tension * m_ForceScale, m_ForceRadius, m_ForceColour, true);
            GLUtils::DrawCylinder(m_PointForceList[1]->point, m_PointForceList[1]->vector, m_Tension * m_ForceScale, m_ForceRadius, m_ForceColour, true);
            GLUtils::DrawCylinder(m_PointForceList[2]->point, m_PointForceList[2]->vector, m_Tension * m_ForceScale, m_ForceRadius, m_ForceColour, true);
        }
    }
}
#endif

