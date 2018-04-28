/*
 *  MAMuscle.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// MAMuscle - implementation of an Minetti & Alexander style
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a parallel spring element

#include <ode/ode.h>

#include "Strap.h"
#include "MAMuscle.h"
#include "DebugControl.h"

// constructor

MAMuscle::MAMuscle(Strap *strap): Muscle(strap)
{
    m_VMax = 0;
    m_F0 = 0;
    m_K = 0;
    m_Alpha = 0;
}

// destructor
MAMuscle::~MAMuscle()
{
}

// set the proportion of muscle fibres that are active
// calculates the tension in the strap

void MAMuscle::SetAlpha(dReal alpha)
{
    dReal fCE, fPE;
    dReal v, fFull;
    
    if (alpha < 0) m_Alpha = 0;
    else
    {
        if (alpha > 1.0) m_Alpha = 1.0;
        else m_Alpha = alpha;
    }
    
    // m_Velocity is negative when muscle shortening
    // we need the sign the other way round
    v = -m_Strap->GetVelocity();

    // limit v
    if (v > m_VMax) v = m_VMax;
    else if (v < -m_VMax) v = -m_VMax;

    if (v < 0)
    {
        fFull = m_F0 * (1.8 - 0.8 * ((m_VMax + v) / (m_VMax - (7.56 / m_K) * v)));
    }
    else
    {
        fFull = m_F0 * (m_VMax - v) / (m_VMax + (v / m_K));
    }

    // now set the tension as a proportion of fFull
    fCE = m_Alpha * fFull;
    m_Strap->SetTension(fCE);

    if (gDebug == MAMuscleDebug)
    {
        *gDebugStream << "MAMuscle::SetAlpha " << m_Name <<
        " alpha " << alpha <<
        " m_Alpha " << m_Alpha <<
        " m_F0 " << m_F0 <<
        " m_VMax " << m_VMax <<
        " m_Velocity " << m_Strap->GetVelocity() <<
        " fFull " << fFull <<
        " m_Length " << m_Strap->GetLength() <<
        " fCE " << fCE <<
        " fPE " << fPE <<
        "\n";
    }
}

// calculate the metabolic power of the muscle

dReal MAMuscle::GetMetabolicPower()
{
    // m_Velocity is negative when muscle shortening
    // we need the sign the other way round
    dReal relV = -m_Strap->GetVelocity() / m_VMax;
    
    // limit relV
    if (relV > 1) relV = 1;
    else if (relV < -1) relV = -1;
    
    dReal relVSquared = relV * relV;
    dReal relVCubed = relVSquared * relV;
    
    dReal sigma = (0.054 + 0.506 * relV + 2.46 * relVSquared) /
        (1 - 1.13 * relV + 12.8 * relVSquared - 1.64 * relVCubed);
    
    if (gDebug == MAMuscleDebug)
    {
        *gDebugStream << "MAMuscle::GetMetabolicPower " << m_Name <<
        " m_Alpha " << m_Alpha <<
        " m_F0 " << m_F0 <<
        " m_VMax " << m_VMax <<
        " m_Velocity " << m_Strap->GetVelocity() <<
        " sigma " << sigma <<
        " power " << m_Alpha * m_F0 * m_VMax * sigma << "\n";
    }
    return (m_Alpha * m_F0 * m_VMax * sigma);
}



