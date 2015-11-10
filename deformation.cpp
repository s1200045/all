#include "deformation.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <math.h>
#include <utility>

void deformation::selectedVertDeformation()
{
  std::set<unsigned>::const_iterator it = selectedVert.begin();
      Vec3 p_center = mesh->getVertPos(*it);
   mesh->computeVertNormals(vertNormal);
   for(unsigned i=0;i<mesh->numVerts();i++){
     Vec3 p_neighbor = mesh->getVertPos(i);
     distance=sqrt(pow((p_center.x-p_neighbor.x),2)+pow((p_center.y-p_neighbor.y),2)+pow((p_center.z-p_neighbor.z),2));
     if(distance>thr)disp=0;
     else disp=d*exp(-alpha*pow(distance,2));
     Point p(p_neighbor.x+disp*vertNormal[i].x,p_neighbor.y+disp*vertNormal[i].y,p_neighbor.z+disp*vertNormal[i].z);
     Vector v(vertNormal[i].x,vertNormal[i].y,vertNormal[i].z);
     points.push_back(std::make_pair(p,v));
   }
   
   
   const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
   CGAL::pca_estimate_normals(points.begin(), points.end(), CGAL::First_of_pair_property_map<PointVectorPair>(),CGAL::Second_of_pair_property_map<PointVectorPair>(),nb_neighbors);
   
   std::vector<PointVectorPair> ::iterator unoriented_points_begin =
     CGAL::mst_orient_normals(points.begin(), points.end(),
			      CGAL::First_of_pair_property_map<PointVectorPair>(),
			      CGAL::Second_of_pair_property_map<PointVectorPair>(),
			      nb_neighbors);
   points.erase(unoriented_points_begin, points.end());
}
    
