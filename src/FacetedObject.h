/*
 *  FacettedObject.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 13/09/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef FacetedObject_h
#define FacetedObject_h

#include "NamedObject.h"

class Vertex;
class Face;
class DataFile;
class ostringstream;

struct StridedVertex {
    dReal Vertex[3];
    // Userdata
};

struct StridedTri {
    int Indices[3];
    // Userdata
};

class FacetedObject: public NamedObject
{
        public:
        FacetedObject();
        ~FacetedObject();
        
        bool ParseOBJFile(const char *filename);
        bool ParseOBJFile(DataFile &theFile);
        
        virtual void WritePOVRay(std::ostringstream &theString);
        virtual void WriteOBJFile(std::ostringstream &out);
    
        int AddVertex(Vertex vertex, bool fast = true);
        int GetNumVertices()
        {
            return mNumVertices;
        };
        Vertex *GetVertex(int i)
        {
            if (i < 0 || i >= mNumVertices) throw (__LINE__);
            return mVertexList[i];
        }
    
        void AddFace(Face *face);
        void AddFace(Vertex vertex[], int numVertices, bool fast = true);
        int GetNumFaces()
        {
            return mNumFaces;
        };
        Face *GetFace(int i)
        {
            if (i < 0 || i >= mNumFaces) throw (__LINE__);
            return mFaceList[i];
        };
    
        void ClearLists();
        void Concatenate(FacetedObject *object);
        
        virtual void Draw();
        void SetDisplayPosition(dReal x, dReal y, dReal z);
        void SetDisplayRotation(const dMatrix3 R, bool fast = true);
        void SetDisplayRotationFromQuaternion(const dQuaternion q, bool fast = true);
        void SetDisplayRotationFromAxis(dReal x, dReal y, dReal z, bool fast = true);
        void SetDrawClockwise(bool v) { mDrawClockwise = v; };
        void SetBadMesh(bool v) { mBadMesh = v; };
        
        void CalculateNormals();
    
        // static utilities
        static void ComputeFaceNormal(const Vertex *v1, 
            const Vertex *v2, const Vertex *v3, dReal normal[3]);
    
        // manipulation functions
        void Move(dReal x, dReal y, dReal z);
        void Scale(dReal x, dReal y, dReal z);
        void Mirror(bool x, bool y, bool z);
        void SwapAxes(int axis1, int axis2);
        void RotateAxes(int axis0, int axis1, int axis2);
    
        // reporting functions
        void Stats();
        
        // utility
        void Triangulate();
    
        // ODE link
        void CalculateTrimesh(StridedVertex **vertices, int *numVertices, int *vertexStride, StridedTri **tris, int *numTris, int *triStride);
        void CalculateMassProperties(dMass *m, dReal density, bool clockwise);

            
    protected:
                    
        int mNumVertices;
        int mNumVerticesAllocated;
        Vertex **mVertexList;
        int mNumFaces;
        int mNumFacesAllocated;
        Face **mFaceList;
        bool mDrawClockwise;
        bool mBadMesh;
        dReal m_VertexMatchTolerance;
        bool m_UseRelativeOBJ;
        
        dVector3 m_DisplayPosition; 
        dMatrix3 m_DisplayRotation;
    
        static const int kAllocationIncrement = 1000;
    };

#endif
