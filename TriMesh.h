#ifndef TRIMESH_H
#define TRIMESH_H

#include "types.h"
#include <vector>

class TriMesh
{
public:
    TriMesh() { }

    TriMesh(const TriMesh& mesh)
    : mPoints(mesh.mPoints)
    , mFaceToVert(mesh.mFaceToVert)
    { }

    TriMesh& operator = (const TriMesh& mesh)
    {
        mPoints = mesh.mPoints;
        mFaceToVert = mesh.mFaceToVert;
        return *this;
    }

    void clear();
    void normalize();
    void read(const char* filename);

    inline unsigned numVerts() const { return mPoints.size(); }
    inline unsigned numFaces() const { return mFaceToVert.size(); }
    inline const Vec3& getVertPos(unsigned vert) const { return mPoints[vert]; }

    void getFaceVerts(unsigned face, std::vector<unsigned>& verts) const;
    unsigned getVertIndexInFace(unsigned face, unsigned vert) const;
    unsigned getFaceVert(unsigned face, unsigned index) const;
    double computeFaceArea(unsigned face) const;
    Vec3 computeFaceNormal(unsigned face) const;
    double computeTotalArea() const;
    double computeMaxEdgeLength() const;
    void computeVertNormals(std::vector<Vec3>& vertNormal) const;
    double computeEdgeLength(unsigned vert0, unsigned vert1) const;

private:
    std::vector<Vec3> mPoints;
    std::vector< std::vector<unsigned> > mFaceToVert;
};

#endif // TRIMESH_H

