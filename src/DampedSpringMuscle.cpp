/*
 *  DampedSpringMuscle.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// DampedSpringMuscle - implementation of a damped spring strap force

#include <ode/ode.h>

#include "Strap.h"
#include "DampedSpringMuscle.h"
#include "DebugControl.h"

// constructor

DampedSpringMuscle::DampedSpringMuscle(Strap *strap): Muscle(strap)
{
    m_Damping = 0;
    m_SpringConstant = 0;
    m_UnloadedLength = 0;
    m_Activation = 0;
    m_Area = 1;
}

// destructor
DampedSpringMuscle::~DampedSpringMuscle()
{
}

dReal DampedSpringMuscle::GetElasticEnergy()
{
    dReal delLen = m_Strap->GetLength() - m_UnloadedLength;
    if (delLen < 0) return 0;
    
    // difference between these two values is the amount of energy lost by damping
    // std::cerr << 0.5 * m_Strap->GetTension() * delLen << "\n";
    // std::cerr << 0.5 * m_SpringConstant * m_Area * delLen * delLen / m_UnloadedLength << "\n";
    
    return 0.5 * m_SpringConstant * m_Area * delLen * delLen / m_UnloadedLength;
}


// update the tension depending on length and velocity
// activation is used as a linear multiplier
void DampedSpringMuscle::SetActivation(dReal activation, dReal duration)
{
    m_Activation = activation;
    
    // calculate strain
    dReal elasticStrain = (m_Strap->GetLength() - m_UnloadedLength) / m_UnloadedLength;
    
    // calculate stress
    dReal elasticStress = elasticStrain * m_SpringConstant;
    
    // calculate damping
    dReal relativeVelocity = m_Strap->GetVelocity() / m_UnloadedLength;
    dReal dampingStress = relativeVelocity * m_Damping;

    // now calculate tension
    // NB. tension is negative when muscle shortening
    dReal tension = (elasticStress + dampingStress) * m_Area * m_Activation;
    
    // stop any pushing
    if (tension < 0) tension = 0;
    m_Strap->SetTension(tension);
    
    if (gDebug == DampedSpringDebug)
        *gDebugStream << "DampedSpringMuscle::UpdateTension m_Name " << m_Name << " m_Strap->GetLength() " << m_Strap->GetLength() << " m_UnloadedLength " << m_UnloadedLength
            << " m_SpringConstant " << m_SpringConstant << " m_Strap->GetVelocity() " << m_Strap->GetVelocity()
            << " m_Damping " << m_Damping << " tension " << tension << "\n";
}



