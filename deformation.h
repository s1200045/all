#ifndef DEFORMATION_H
#define DEFORMATION_H
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cassert>
#include <limits>
#include <set>
#include "Viewer.h"
#include "TriMesh.h"
#include <utility>
#include "types.h"
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
class deformation{
  deformation(TriMesh* mesh,std::set<unsigned> selectedVert){
    mesh=mesh;
    selectedVert=selectedVert;
  }

  public:
    typedef std::pair<Point, Vector> PointVectorPair;
    std::vector<Vec3> vertNormal;
  std::vector<PointVectorPair> points;
  TriMesh* mesh;
  std::set<unsigned> selectedVert;
  double distance,thr=1,d=1/13,alpha=1.0,disp;
  void selectedVertDeformation();

};
#endif
