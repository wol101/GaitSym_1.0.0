/*
 *  SphereGeom.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 05/12/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <string>

#include "SphereGeom.h"
#include "FacetedSphere.h"

#ifdef USE_OPENGL
extern int gDrawGeomsFlag;
extern std::string gOBJName;
#endif

SphereGeom::SphereGeom(dSpaceID space, dReal radius)
{
    // create the geom
    m_GeomID = dCreateSphere(0, radius); 
    dGeomSetData(m_GeomID, this);
    
    // and now create the Geometry Transform Class
    m_GeomTransformID = dCreateGeomTransform(space);
    dGeomSetData(m_GeomTransformID, this);
    dGeomTransformSetGeom(m_GeomTransformID, m_GeomID);
}

#ifdef USE_OPENGL
void SphereGeom::Draw()
{
    if (gDrawGeomsFlag)
    {
        gOBJName = m_Name;
        
        const dReal *bodyRotation = dGeomGetRotation(m_GeomTransformID);
        const dReal *cylinderRelPosition = dGeomGetPosition(m_GeomID);
        const dReal *cylinderRelRotation = dGeomGetRotation(m_GeomID);
        
        dVector3 p;
        dMatrix3 r;
        
        // get the cylinder position in world coordinates
        dBodyGetRelPointPos(dGeomGetBody(m_GeomTransformID), cylinderRelPosition[0],cylinderRelPosition[1], cylinderRelPosition[2], p);
        
        //combine the body rotation with the cylinder rotation to get combined rotation from world coordinates
        dMultiply0(r, bodyRotation, cylinderRelRotation, 3, 3, 3);
        
        // get the radius
        dReal radius = dGeomSphereGetRadius (m_GeomID);
        
        // and draw the sphere
        const static int kLevels = 3;
        FacetedSphere sphere(radius, kLevels);
        sphere.SetColour(m_Colour);
        sphere.SetDisplayPosition(p[0], p[1], p[2]);
        sphere.SetDisplayRotation(r);
        sphere.Draw();
    }
    
}
#endif
