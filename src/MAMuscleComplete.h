/*
 *  MAMuscleComplete.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 03/03/2007.
 *  Copyright 2006 Bill Sellers. All rights reserved.
 *
 */

// MAMuscleComplete - implementation of an Minetti & Alexander style 
// muscle based on the StrapForce class

// Minetti & Alexander, J. theor Biol (1997) 186, 467-476

// Added extra terms to allow a serial and parallel spring element
// plus length tension stuff too


#ifndef MAMuscleComplete_h
#define MAMuscleComplete_h

#ifdef USE_GSL

#include "Muscle.h"
#include <gsl/gsl_roots.h>

// this struct contains all the paramers required for the CalculateForceError function
struct CalculateForceErrorParams
{
    // input parameters
    dReal spe; // slack length parallel element (m)
    dReal epe; // elastic constant parallel element (N/m)
    dReal sse; // slack length serial element (m)
    dReal ese; // elastic constant serial element (N/m)
    dReal k; // shape constant
    dReal vmax; // maximum shortening velocity (m/s)
    dReal fmax; // maximum isometric force (N)
    dReal width; // relative width of length/tension peak
    dReal alpha; // proportion of muscle activated
    dReal timeIncrement; // inegration step size for simulation (s)
    dReal len; // length of the whole element (m)
    dReal lastlpe; // last calculated lpe value (m)
    
    // output parameters
	dReal fce; // contractile force (N)
    dReal lpe; // contractile and parallel length (m)
    dReal fpe; // parallel element force (N)
    dReal lse; // serial length (m)
    dReal fse; // serial element force (N)
    dReal vce; // contractile element velocity (m/s)
    dReal targetFce; // fce calculated from elastic elements (N)
    dReal f0; // length corrected fmax (N)
};


class Strap;
class MAMuscle;
class DampedSpringMuscle;
class SimpleStrap;
class Filter;

class MAMuscleComplete : public Muscle
{
public:
    
    MAMuscleComplete(Strap *strap);
    ~MAMuscleComplete();
    
    void SetSerialElasticProperties(dReal springConstant, dReal unloadedLength); 
    void SetParallelElasticProperties(dReal springConstant, dReal unloadedLength);
    void SetMuscleProperties(dReal vMax, dReal Fmax, dReal K, dReal Width);
    void SetActivationKinetics(bool activationKinetics) { m_ActivationKinetics = activationKinetics; };

    virtual dReal GetMetabolicPower();

    virtual void SetActivation(dReal activation, dReal timeIncrement);
    virtual dReal GetActivation() { return m_Stim; };
    
    dReal GetFCE() { return m_Params.fce; };
    dReal GetLPE() { return m_Params.lpe; };
    dReal GetFPE() { return m_Params.fpe; };
    dReal GetLSE() { return m_Params.lse; };
    dReal GetFSE() { return m_Params.fse; };
    dReal GetVCE() { return m_Params.vce; };
    dReal GetVPE() { return m_Params.vce; };
    dReal GetVSE() { return GetVelocity() - m_Params.vce; };
    dReal GetESE() { if (m_Params.lse > m_Params.sse) return (0.5 * SQUARE(m_Params.lse - m_Params.sse) * m_Params.ese); else return 0; }; // energy serial element
    dReal GetEPE() { if (m_Params.lpe > m_Params.spe) return (0.5 * SQUARE(m_Params.lpe - m_Params.spe) * m_Params.epe); else return 0; }; // energy parallel element
    dReal GetPSE() { return GetVSE() * -m_Params.fse; }; // power serial element
    dReal GetPPE() { return GetVPE() * -m_Params.fpe; }; // power parallel element
    dReal GetPCE() { return GetVCE() * -m_Params.fce; }; // power contractile element
        
protected:

    dReal m_Stim;
    bool m_ActivationKinetics;
    
    CalculateForceErrorParams m_Params;
    gsl_root_fsolver *m_gsl_root_fsolver;
    dReal m_Tolerance;
};






#endif // USE_GSL

#endif // MAMuscleComplete_h
