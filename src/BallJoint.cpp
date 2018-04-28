/*
 *  BallJoint.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/12/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif

#include <ode/ode.h>

#include "BallJoint.h"
#include "DataFile.h"
#include "Body.h"
#include "Simulation.h"
#include "DebugControl.h"

#ifdef USE_OPENGL
#include "GLUtils.h"
#include "FacetedSphere.h"
extern int gAxisFlag;
extern std::string gOBJName;
#endif

// Simulation global
extern Simulation *gSimulation;

BallJoint::BallJoint(dWorldID worldID)
{
    m_JointID = dJointCreateBall(worldID, 0);
    dJointSetData(m_JointID, this);
    
    dJointSetFeedback(m_JointID, &m_JointFeedback); 
}

void BallJoint::SetBallAnchor (dReal x, dReal y, dReal z)
{
    dJointSetBallAnchor(m_JointID, x, y, z); 
}

// parses the position allowing a relative position specified by BODY ID
// x y z - world coordinates
// bodyName x y z - position relative to bodyName local coordinate system
void
BallJoint::SetBallAnchor(const char *buf)
{
	int i;
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dVector3 pos;
    
    strcpy(lBuf, buf);
	int count = DataFile::ReturnTokens(lBuf, lBufPtrs, l);
	if (count < 3) 
    {
        std::cerr << "Error in BallJoint::SetBallAnchor\n";
        return; // error condition
    }
	
	if (isalpha((int)*lBufPtrs[0]) == 0) 
	{
		for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i], 0);
        dJointSetBallAnchor(m_JointID, pos[0], pos[1], pos[2]);
		return;
	}
	
	if (count < 4) 
    {
        std::cerr << "Error in BallJoint::SetBallAnchor\n";
        return; // error condition
    }
	Body *theBody = gSimulation->GetBody(lBufPtrs[0]);
	if (theBody == 0) 
    {
        if (strcmp(lBufPtrs[0], "World") == 0)
        {
            for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
            dJointSetBallAnchor(m_JointID, pos[0], pos[1], pos[2]);
            return;
        }
        else
        {
            std::cerr << "Error in BallJoint::SetBallAnchor\n";
            return; // error condition
        }
    }
    dVector3 result;
    for (i = 0; i < 3; i++) pos[i] = strtod(lBufPtrs[i + 1], 0);
    dBodyGetRelPointPos (theBody->GetBodyID(), pos[0], pos[1], pos[2], result);
    dJointSetBallAnchor(m_JointID, result[0], result[1], result[2]);
}

void BallJoint::GetBallAnchor(dVector3 result)
{
    dJointGetBallAnchor(m_JointID, result);
}

void BallJoint::GetBallAnchor2(dVector3 result)
{
    dJointGetBallAnchor2(m_JointID, result);
}

#ifdef USE_OPENGL
void BallJoint::Draw()
{
    if (gAxisFlag) 
    {
        dVector3 anchor;
        dJointGetBallAnchor(m_JointID, anchor);
        gOBJName = m_Name;
        // and draw the sphere
        const static int kLevels = 3;
        dReal radius = m_AxisSize[0] / 10;
        FacetedSphere sphere(radius, kLevels);
        sphere.SetColour(m_Colour);
        sphere.SetDisplayPosition(anchor[0], anchor[1], anchor[2]);
        // sphere.SetDisplayRotation(r);
        sphere.Draw();
    }
}
#endif
