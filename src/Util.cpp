/*
 *  Util.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 12/07/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include <ode/ode.h>
#include <math.h>
#include <vector>
#include <string>

#include "Util.h"

// Euler decomposition examples from Geometric Tools for Computer Graphics, Scheider & Eberly 2003

// matrix format is:
// [r00 r01 r02]
// [r10 r11 r12]
// [r20 r21 r22]

// ode format definition
#define r00 mRot[0*4 + 0]
#define r01 mRot[0*4 + 1]
#define r02 mRot[0*4 + 2]
#define r10 mRot[1*4 + 0]
#define r11 mRot[1*4 + 1]
#define r12 mRot[1*4 + 2]
#define r20 mRot[2*4 + 0]
#define r21 mRot[2*4 + 1]
#define r22 mRot[2*4 + 2]

void Util::EulerDecompositionXYZ(const dReal *mRot, dReal& thetaX, dReal& thetaY, dReal& thetaZ)
{
    thetaY = asin(r02); 
    if ( thetaY < M_PI/2 ) 
    { 
        if ( thetaY > -M_PI/2 ) 
        { 
            thetaX = atan2(-r12,r22); 
            thetaZ = atan2(-r01,r00); 
        } 
        else 
        { 
            // not a unique solution 
            thetaX = -atan2(r10,r11); 
            thetaZ = 0; 
        } 
    } 
    else 
    { 
        // not a unique solution 
        thetaX = atan2(r10,r11); 
        thetaZ = 0; 
    } 
}

void Util::EulerDecompositionXZY(const dReal *mRot, dReal& thetaX, dReal& thetaY, dReal& thetaZ)
{
    thetaZ = asin(-r01); 
    if ( thetaZ < M_PI/2 ) 
    { 
        if ( thetaZ > -M_PI/2 ) 
        { 
            thetaX = atan2(r21,r11); 
            thetaY = atan2(r02,r00); 
        } 
        else 
        { 
            // not a unique solution 
            thetaX = -atan2(-r20,r22); 
            thetaY = 0; 
        } 
    } 
    else 
    { 
        // not a unique solution 
        thetaX = atan2(-r20,r22); 
        thetaY = 0; 
    } 
}

void Util::EulerDecompositionYXZ(const dReal *mRot, dReal& thetaX, dReal& thetaY, dReal& thetaZ)
{
    thetaX = asin(-r12); 
    if ( thetaX < M_PI/2 ) 
    { 
        if ( thetaX > -M_PI/2 ) 
        { 
            thetaY = atan2(r02,r22); 
            thetaZ = atan2(r10,r11); 
        } 
        else 
        { 
            // not a unique solution 
            thetaY = -atan2(-r01,r00); 
            thetaZ = 0; 
        } 
    } 
    else 
    { 
        // not a unique solution 
        thetaY = atan2(-r01,r00); 
        thetaZ = 0; 
    } 
}

void Util::EulerDecompositionYZX(const dReal *mRot, dReal& thetaX, dReal& thetaY, dReal& thetaZ)
{
    thetaZ = asin(r10); 
    if ( thetaZ < M_PI/2 ) 
    { 
        if ( thetaZ > -M_PI/2 ) 
        { 
            thetaY = atan2(-r20,r00); 
            thetaX = atan2(-r12,r11); 
        } 
        else 
        { 
            // not a unique solution 
            thetaY = -atan2(r21,r22); 
            thetaX = 0; 
        } 
    } 
    else 
    { 
        // not a unique solution 
        thetaY = atan2(r21,r22); 
        thetaX = 0; 
    } 
}

void Util::EulerDecompositionZXY(const dReal *mRot, dReal& thetaX, dReal& thetaY, dReal& thetaZ)
{
    thetaX = asin(r21); 
    if ( thetaX < M_PI/2 ) 
    { 
        if ( thetaX > -M_PI/2 ) 
        { 
            thetaZ = atan2(-r01,r11); 
            thetaY = atan2(-r20,r22); 
        } 
        else 
        { 
            // not a unique solution 
            thetaZ = -atan2(r02,r00); 
            thetaY = 0; 
        } 
    } 
    else 
    { 
        // not a unique solution 
        thetaZ = atan2(r02,r00); 
        thetaY = 0; 
    } 
}
 
void Util::EulerDecompositionZYX(const dReal *mRot, dReal& thetaX, dReal& thetaY, dReal& thetaZ)
{
    thetaY = asin(-r20); 
    if ( thetaY < M_PI/2 ) 
    { 
        if ( thetaY > -M_PI/2 ) 
        { 
            thetaZ = atan2(r10,r00); 
            thetaX = atan2(r21,r22); 
        } 
        else 
        { 
            // not a unique solution 
            thetaZ = -atan2(-r01,r02); 
            thetaX = 0; 
        } 
    } 
    else 
    { 
        // not a unique solution 
        thetaZ = atan2(-r01,r02); 
        thetaX = 0; 
    } 
}

void Util::Tokenizer(const char *constbuf, std::vector<std::string> &tokens, char *stopList)
{
    char *localBuf = (char *)malloc(strlen(constbuf) + 1);
    char *ptr = localBuf;
    strcpy(ptr, constbuf);
    char *qp;
    char byte;
    char oneChar[2] = {0, 0};
    if (stopList == 0) stopList = "{};,=:&|!()+-/*[]'<>^";
     
     while (*ptr)
     {
         // find non-whitespace
         if (*ptr < 33)
         {
             ptr++;
             continue;
         }
         
         // is it in stoplist
         if (strchr(stopList, *ptr))
         {
             oneChar[0] = *ptr;
             tokens.push_back(oneChar);
             ptr++;
             continue;
         }
         
         // is it a double quote?
         if (*ptr == '"')
         {
             ptr++;
             qp = strchr(ptr, '"');
             if (qp)
             {
                 *qp = 0;
                 tokens.push_back(ptr);
                 *qp = '"';
                 ptr = qp + 1;
                 continue;
             }
         }
         
         qp = ptr;
         while (*qp >= 33 && strchr(stopList, *qp) == 0 && *qp != '"') 
         {
             qp++;
         }
         byte = *qp;
         *qp = 0;
         tokens.push_back(ptr);
         if (byte == 0) break;
         *qp = byte;
         ptr = qp;
     }
     
     free(localBuf);
}





