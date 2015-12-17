#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <eigen/Eigen/Core>
#include "hrbf_core.h"
#include "hrbf_phi_funcs.h"
#include "CIsoSurface.h"
#include "Vectors.h"
#include <utility>
#include <fstream>
#include <CGAL/compute_average_spacing.h>
#include <AntTweakBar.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/trace.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/trace.h>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cassert>
#include <limits>
#include "Viewer.h"
#include "Image.h"
#include "TriMesh.h"

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>

#else
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>

#endif

int  Viewer::windowSize[2] = { 800, 800 };
bool Viewer::renderWireframe = false;
bool Viewer::renderSelected = false;

std::set<unsigned> Viewer::selectedVert;
TriMesh* Viewer::meshPtr = 0;
int    Viewer::verbose = 0;

GLuint Viewer::surfaceDL = 0;
Shader Viewer::shader;
Camera Viewer::camera;

//added
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Eigen::Matrix<double,3,1> Vector3;
bool flag=0;
CIsoSurface <double> *ciso = new CIsoSurface <double> ();
TriMesh Viewer::marching;
TwBar *myBar;
typedef CGAL::Poisson_reconstruction_function<Kernel> Poisson_reconstruction_function;
typedef Kernel::FT FT;
typedef CGAL::Surface_mesh_default_triangulation_3 STr;
typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<STr> C2t3;
typedef Kernel::Sphere_3 Sphere;
typedef CGAL::Implicit_surface_3<Kernel, Poisson_reconstruction_function> Surface_3;

typedef CGAL::Polyhedron_3<Kernel> Polyhedron;


void
Viewer::keyboard(unsigned char c, int /*x*/, int /*y*/)
{
    switch(c)
    {
        case 27:
            exit(0);
            break;
        case 'w':
            renderWireframe = !renderWireframe;
            updateDisplayList();
            break;
        case 's':
            renderSelected = !renderSelected;
            std::cout<<renderSelected<<std::endl;
            updateDisplayList();
            break;
        case 'r':
            clearData();
            updateDisplayList();
            break;
            
        case '/':
            takeScreenshot();
            break;
        default:
            break;
    }
}
   
void 
Viewer::init(int argc, char** argv)
{
    initGLUT(argc, argv);
    initGLSL();
    setGL();
    updateDisplayList();
   // TwInit(TW_OPENGL, NULL);
    glutMainLoop();
}
   
void 
Viewer::initGLUT(int argc, char** argv)
{
    glutInitWindowSize(windowSize[0], windowSize[1]);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInit(&argc, argv);
    glutCreateWindow("Pick Test");

    glutMouseFunc   (Viewer::mouse   );
    glutKeyboardFunc(Viewer::keyboard);
    glutSpecialFunc (Viewer::special );
    glutMotionFunc  (Viewer::motion  );
    glutDisplayFunc (Viewer::display );
    glutIdleFunc    (Viewer::idle    );
}
   
void 
Viewer::initGLSL()
{
    shader.loadVertex  ("vertex.glsl"  );
    shader.loadFragment("fragment.glsl");
}
   
void 
Viewer::special(int i, int /*x*/, int /*y*/)
{
    switch (i)
    {
        case GLUT_KEY_UP:
            camera.zoomIn();
            break;
        case GLUT_KEY_DOWN:
            camera.zoomOut();
            break;
        case 27:
            exit(0);
        break;
        default:
            break;
    }
}

void 
Viewer::mouse(int button, int state, int x, int y)
{

    if ((glutGetModifiers() & GLUT_ACTIVE_SHIFT) && (state == GLUT_UP))
    {
        pickVertex(x, y);
        return;
    }
    camera.mouse(button, state, x, y);
}

void 
Viewer::motion(int x, int y)
{
    camera.motion(x, y);
}

void 
Viewer::idle()
{
    glutPostRedisplay();
}

void 
Viewer::display()
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    shader.enable();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    double aspect = double(viewport[2]) / double(viewport[3]);
    const double fovy = 50.;
    const double clipNear = .01;
    const double clipFar = 1000.;
    gluPerspective(fovy, aspect, clipNear, clipFar);
      
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    Quaternion    eye(0., 0., 0.,-2.5*camera.zoom());
    Quaternion center(0., 0., 0., 0. );
    Quaternion     up(0., 0., 1., 0. );
    
    gluLookAt(   eye.x,    eye.y,    eye.z,
              center.x, center.y, center.z,
                  up.x,     up.y,     up.z );
    
    GLint uniformEye = glGetUniformLocation(shader, "eye");
    Quaternion r = camera.computeCurrentRotation();
    eye = r.conjugate() * eye * r;
    glUniform3f(uniformEye, eye.x, eye.y, eye.z);
      
    GLint uniformLight = glGetUniformLocation( shader, "light" );
    Quaternion light(0., -1., 1., -10.);
    light = r.conjugate() * light * r;
    glUniform3f(uniformLight, light.x, light.y, light.z);

    camera.setGLModelView();
    
    callDisplayList();

    shader.disable();
    
    
    //
    /*
    Quaternion    eye(0., 0., 0.,-2.5*camera.zoom());
    Quaternion center(0., 0., 0., 0. );
    Quaternion     up(0., 0., 1., 0. );
    GLint uniformEye = glGetUniformLocation(shader, "eye");
    Quaternion r = camera.computeCurrentRotation();
    eye = r.conjugate() * eye * r;
    glUniform3f(uniformEye, eye.x, eye.y, eye.z);
      
    GLint uniformLight = glGetUniformLocation( shader, "light" );
    Quaternion light(0., -1., 1., -10.);
    light = r.conjugate() * light * r;
    glUniform3f(uniformLight, light.x, light.y, light.z);

    glTranslated(0.0, 0.0, -3.0);
    glutSolidSphere(1.0, 16, 16);
    
    shader.disable();
    */
    //
    
        glutSwapBuffers();
    
}
   
void 
Viewer::updateDisplayList()
{
    if (surfaceDL)
    {
        glDeleteLists(surfaceDL, 1);
        surfaceDL = 0;
    }

    surfaceDL = glGenLists(1);
    glNewList(surfaceDL, GL_COMPILE);
    drawScene();
    glEndList();
}
   
void Viewer::setGL()
{
    glClearColor( 1., 1., 1., 1. );
    setLighting();
}
   
void Viewer::setLighting()
{
    GLfloat position[4] = { 20., 30., 40., 0. };
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHT0);

    GLfloat  diffuse[4] = { .8, .5, .3, 1. };
    GLfloat specular[4] = { .3, .3, .3, 1. };
    GLfloat  ambient[4] = { .2, .2, .5, 1. };
      
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   diffuse );
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   ambient );
    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 16.     );
}
   
void 
Viewer::callDisplayList()
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glCallList(surfaceDL);
    glPopAttrib();
}
   
void 
Viewer::drawScene()
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    //glEnable(GL_POLYGON_OFFSET_FILL);
    //glPolygonOffset(1., 1.);
    
   // drawPolygons();

    glDisable(GL_POLYGON_OFFSET_FILL);
    if (renderWireframe) drawWireframe();
    if (renderSelected ) drawSelectedVerts();
    //drawSelectedVerts();
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1., 1.);
    
    drawPolygons();

    glPopAttrib();
}

void 
Viewer::drawPolygons()
{
       // if(flag==0){
            glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);

    std::vector<Vec3> vertNormal;
    meshPtr->computeVertNormals(vertNormal);

    glBegin(GL_TRIANGLES);
    for (unsigned face = 0; face < meshPtr->numFaces(); ++face) 
    {
        if (renderWireframe) 
        {
            Vec3 n = meshPtr->computeFaceNormal(face);
            glNormal3d(n.x, n.y, n.z);
        }

        std::vector<unsigned> fVerts;
        meshPtr->getFaceVerts(face, fVerts);
        assert(fVerts.size() == 3);

        for (unsigned i = 0; i < 3; ++i) 
        {
            unsigned vert = fVerts[i];
            Vec3 p = meshPtr->getVertPos(vert);
            if (!renderWireframe) glNormal3d(vertNormal[vert].x,vertNormal[vert].y, vertNormal[vert].z);

            double alpha = 0.5;
            glColor3d(alpha, alpha, alpha);
            glVertex3d(p.x, p.y, p.z);
        }
    }
            glEnd();
    glDisable(GL_COLOR_MATERIAL);
//}
       //else if(flag==1){
           /*
            glEnable(GL_LIGHTING);
            glEnable(GL_COLOR_MATERIAL);
              glBegin(GL_TRIANGLES);
            
            for (unsigned i = 0; i < ciso->m_nTriangles; ++i)
            {
                    glNormal3f(ciso->m_pvec3dNormals[ciso->m_piTriangleIndices[i]][0],ciso->m_pvec3dNormals[ciso->m_piTriangleIndices[i]][1], ciso->m_pvec3dNormals[ciso->m_piTriangleIndices[i]][2]);
                float alpha = 0.5;
                glColor3f(alpha, alpha, alpha);
                glVertex3f(ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i*3]][0], ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i*3]][1], ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i*3]][2]);
                glVertex3f(ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i*3+1]][0], ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i*3+1]][1], ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i*3+1]][2]);
                glVertex3f(ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i*3+2]][0], ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i*3+2]][1], ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i*3+2]][2]);
                
                
                    std::cout<<ciso->m_piTriangleIndices[i]<<" "<< ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i]][1]<<" " <<ciso->m_ppt3dVertices[ciso->m_piTriangleIndices[i]][2]<<std::endl;
            }
            glEnd();
            
            glDisable(GL_COLOR_MATERIAL);
                 std::cout<<"drawed"<<std::endl;
        }*/
       /*    glEnable(GL_LIGHTING);
           glEnable(GL_COLOR_MATERIAL);
           
           std::vector<Vec3> vertNormal;
           marching.computeVertNormals(vertNormal);
       
           glBegin(GL_TRIANGLES);
           for (unsigned face = 0; face < marching.numFaces(); ++face)
           {
               
               if (renderWireframe)
               {
                   Vec3 n = marching.computeFaceNormal(face);
                   glNormal3d(n.x, n.y, n.z);
               }
               
               std::vector<unsigned> fVerts;
               marching.getFaceVerts(face, fVerts);
               assert(fVerts.size() == 3);
               
               for (unsigned i = 0; i < 3; ++i)
               {
                   unsigned vert = fVerts[i];
                   Vec3 p = marching.getVertPos(vert);
                   std::cout<<vert<<std::endl;
                   
                   if (!renderWireframe) glNormal3d(vertNormal[vert].x,vertNormal[vert].y, vertNormal[vert].z);
                   
                   double alpha = 0.5;
                   glColor3d(alpha, alpha, alpha);
                   glVertex3d(p.x, p.y, p.z);
               }
           }
           glEnd();
           glDisable(GL_COLOR_MATERIAL);
       }*/

}
void
Viewer::drawWireframe()
{
    shader.disable();   
    glDisable(GL_LIGHTING);
    glColor4f(0., 0., 0., 1.);

    glBegin(GL_LINES);
    for (unsigned face = 0; face < meshPtr->numFaces(); ++face) 
    {
        std::vector<unsigned> fVerts;
        meshPtr->getFaceVerts(face, fVerts);
        assert(fVerts.size() == 3);

        for (unsigned i = 0; i < 3; ++i) 
        {
            Vec3 p0 = meshPtr->getVertPos(fVerts[(i+1)%3]);
            Vec3 p1 = meshPtr->getVertPos(fVerts[(i+2)%3]);
            glVertex3d(p0.x, p0.y, p0.z);
            glVertex3d(p1.x, p1.y, p1.z);
        }
    }
    glEnd();
}
   
void 
Viewer::drawSelectedVerts( )
{
  
    
    shader.disable();
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glEnable(GL_COLOR_MATERIAL);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glColor3f(0.0, 0.5, 0.5);
    double h = 0.5 * meshPtr->computeMaxEdgeLength();
    for (std::set<unsigned>::const_iterator 
        it = selectedVert.begin(); it != selectedVert.end(); ++it) 
    {
        Vec3 p = meshPtr->getVertPos(*it);
std::cout<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
        glPushMatrix();
        glTranslated(p.x, p.y, p.z);
        //glutSolidSphere(h, 10, 10);
	selectedVertDeformation(p.x, p.y, p.z);

        glPopMatrix();
    }

    glPopAttrib();
}

void 
Viewer::drawVerts()
{
    for (unsigned i = 0; i < meshPtr->numVerts(); ++i)
    {
        Vec3 p = meshPtr->getVertPos(i);
        glLoadName(i);
        glBegin(GL_POINTS);
        glVertex3d(p.x, p.y, p.z);
        glEnd();
    }
}

void 
Viewer::pickVertex(int x, int y)
{
  std::cout<<"called"<<std::endl;
    int width  = glutGet(GLUT_WINDOW_WIDTH );
    int height = glutGet(GLUT_WINDOW_HEIGHT);
    if (x < 0 || x >= width || y < 0 || y >= height) return;

    int bufSize = meshPtr->numVerts();
    GLuint* buf = new GLuint[bufSize];
    glSelectBuffer(bufSize, buf);

    GLint viewport[4];
    GLdouble projection[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    glRenderMode(GL_SELECT);
    glInitNames();
    glPushName(0);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluPickMatrix(x, viewport[3]-y, 10, 10, viewport);
    glMultMatrixd(projection);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    drawVerts();
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    long hits = glRenderMode(GL_RENDER);
    std::cout<<hits<<std::endl;
    int index = -1;
    double min_z = 1.0e100;
    for( long i = 0; i < hits; ++i ) 
    {
        double distance = buf[4*i + 1];
        if (distance < min_z) 
        {
            index = buf[4*i + 3];  
            min_z = distance;
        }
    }
    delete[] buf;
    if (index < 0) return;

    std::set<unsigned>::iterator it = selectedVert.find(index);
    /*if (it == selectedVert.end())
        selectedVert.insert(index);
    else
        selectedVert.erase(it);*/
    selectedVert.clear();
    selectedVert.insert(index);
    updateDisplayList();
}

void
Viewer::clearData()
{
    selectedVert.clear();
}

void
Viewer::pickCenter()
{
    unsigned index = meshPtr->numVerts();
    double minVal = std::numeric_limits<int>::max();
    for (unsigned vert = 0; vert < meshPtr->numVerts(); ++vert) 
    {
        Vec3 p = meshPtr->getVertPos(vert);
        double val = p.norm();
        if (val < minVal) 
        {
            minVal = val;
            index = vert;
        }
    }
    selectedVert.insert(index);    
}


void Viewer::takeScreenshot()
{
    static int index = 0;
  
    GLint view[4];
    glGetIntegerv(GL_VIEWPORT, view);
    int w = view[2];
    int h = view[3];

    // get pixels
    Image image(w, h);
    glReadPixels(0, 0, w, h, GL_BGR, GL_FLOAT, &image(0,0));

    std::stringstream filename;
    filename << "snapshot" << index << ".tga";
    image.write(filename.str().c_str());
    std::cout << "snapshot " << index << " taken" << std::endl;

    index++;
}
//added
void createGrid(const Vector3& leftCorner,
                const Vector3& rightCorner,
                unsigned int nx, unsigned int ny, unsigned int nz,
                std::vector<Vector3>& grid)
{
    double dx = (rightCorner[0] - leftCorner[0]) / nx;
    double dy = (rightCorner[1] - leftCorner[1]) / ny;
    double dz = (rightCorner[2] - leftCorner[2]) / nz;
    
    double currentX = leftCorner[0];
    double currentY = leftCorner[1];
    double currentZ = leftCorner[2];
    
    for (unsigned int zsub = 0; zsub < nz; ++zsub) {
        currentY = leftCorner[1];
        for (unsigned int ysub = 0; ysub < ny; ++ysub) {
            currentX = leftCorner[0];
            for (unsigned int xsub = 0; xsub < nx; ++xsub) {
                grid.push_back(Vector3(currentX, currentY, currentZ));
                currentX = currentX + dx;
            }
            currentY = currentY + dy;
        }
        currentZ = currentZ + dz;
    }
}

void createVTKFile(const std::string& outFileName,
                   unsigned int nx, unsigned int ny, unsigned int nz,
                   const std::vector<Vector3>& grid,
                   const std::vector<double>& data)
{
    std::ofstream out(outFileName.c_str());
    
    // header
    out << "# vtk DataFile Version 3.0" << std::endl;
    out << "vtk output" << std::endl;
    out << "ASCII" << std::endl;
    out << "DATASET STRUCTURED_GRID" << std::endl;
    out << "DIMENSIONS " <<
    nx << " " <<
    ny << " " <<
    nz << std::endl;
    out << "POINTS " << nx*ny*nz << " double" << std::endl;
    
    // structured grid
    std::vector<Vector3>::const_iterator it;
    for (it = grid.begin(); it != grid.end(); ++it) {
        Vector3 curr = *it;
        out << curr[0] << " " << curr[1] << " " << curr[2] << std::endl;
    }
    out << std::endl;
    
    // data
    // header
    out << std::endl;
    out << "POINT_DATA " << nx*ny*nz << std::endl;
    out << "SCALARS Density double" << std::endl;
    out << "LOOKUP_TABLE default" << std::endl;
    
    // data
    std::vector<double>::const_iterator datait;
    for (datait = data.begin(); datait != data.end(); ++datait) {
        out << *datait << std::endl;
    }
    
    out << std::endl;
    
    out.close();
}

void createOFFFile(const std::string& outFileName,std::vector<Vector3>& p, std::vector<Vector3>& n)
{
    std::vector<Vector3>::const_iterator it;
    std::vector<Vector3>::const_iterator it2;
        std::ofstream out(outFileName.c_str());
    it2=n.begin();
    for (it = p.begin(); it != p.end(); ++it) {
        Vector3 curr = *it;
        Vector3 curr2= *it2;
        out << curr[0] << " " << curr[1] << " " << curr[2] <<" "<<curr2[0]<<" "<<curr2[1]<<" "<<curr2[2]<< std::endl;
        it2++;
    }

    out.close();
}

void createOFFFile2(const std::string& outFileName){
    std::ofstream out(outFileName.c_str());
    out<<"OFF"<<std::endl;
    out<<ciso->m_nVertices<<" "<<ciso->m_nTriangles<<" "<<0<<std::endl;
    for(int i=0;i<ciso->m_nVertices;i++){
        out<<ciso->m_ppt3dVertices[i][0]<<" "<<ciso->m_ppt3dVertices[i][1]<<" "<<ciso->m_ppt3dVertices[i][2]<<std::endl;
    }
    for(int i=0;i<ciso->m_nTriangles;i++){
        out<<"3"<<" "<<ciso->m_piTriangleIndices[i*3]<<" "<<ciso->m_piTriangleIndices[i*3+1]<<" "<<ciso->m_piTriangleIndices[i*3+2]<<std::endl;
    }
    
}
/*void Wendland_gradphi(Vector3 p,std::vector<Vector3>& centers,double rho,std::vector<double>& g){
    for(int i=0;i<centers.size();i++){
        double r=sqrt(pow((p(0)-centers[i](0)),2)+pow((p(1)-centers[i](1)),2)+pow((p(2)-centers[i](2)),2));
        std::cout<<r<<std::endl;
        if (r <= rho&&(p(0)!=centers[i](0)+p(1)!=centers[i](1)+p(2)!=centers[i](2))!=0){
            g[i*3]=-20.0/(rho*rho)*(p(0)-centers[i](0))*((1.0-r/rho)*(1.0-r/rho)*(1.0-r/rho));
            g[i*3+1]=-20.0/pow(rho,2)*(p(1)-centers[i](1))*((1.0-r/rho)*(1.0-r/rho)*(1.0-r/rho));
            g[i*3+2]=-20.0/pow(rho,2)*(p(2)-centers[i](2))*((1.0-r/rho)*(1.0-r/rho)*(1.0-r/rho));

        }
    }
 //std::cout<<"p "<<p(0)<<" "<<p(1)<<" "<<p(2)<<std::endl;
}
void evalHRBF_closed(std::vector<Vector3>& pts,std::vector<Vector3>& normals, std::vector<Vector3> &centers,double rho,double eta,std::vector<double>& d){
    
    for(int k=0;k<pts.size();k++){
        d[k]=0;
    }
        std::vector<double> g(centers.size()*3);
    for(int j=0;j<pts.size();j++){
        Vector3 x = pts[j];
    std::cout<<"pts "<<pts[j](0)<<" "<<pts[j](1)<<" "<<pts[j](2)<<std::endl;
        Wendland_gradphi(x, centers, rho,g);
        for(int i=0;i<centers.size();i++){
            double rho2=pow(rho,2);
            Vector3 nr;

            nr(0)=rho2/(20.0+eta*rho2)*normals[i](0);
            nr(1)=rho2/(20.0+eta*rho2)*normals[i](1);
            nr(2)=rho2/(20.0+eta*rho2)*normals[i](2);
            double dt=nr(0)*g[i*3]+nr(1)*g[i*3+1]+nr(2)*g[i*3+2];
            //std::cout<<dt<<std::endl;
            d[j]=d[j]-dt;
          
        }
        //std::cout<<"d="<<d[j]<<std::endl;
        g.clear();
    }
}
*/
void Wendland_gradphi(Vector3 p, std::vector<Vector3>& centers,
                      double rho, std::vector<double>& g)
{
    for(int i=0;i<centers.size();i++){
        g[i*3] = 0.0;
        g[i*3+1] = 0.0;
        g[i*3+2] = 0.0;
        
        double r=sqrt(pow((p(0)-centers[i](0)),2)+pow((p(1)-centers[i](1)),2)+pow((p(2)-centers[i](2)),2));
        
        if (r <= rho&&(p(0)!=centers[i](0)||p(1)!=centers[i](1)||p(2)!=centers[i](2))!=0){
            g[i*3]=-20.0/pow(rho,2)*(p(0)-centers[i](0))*pow((1.0-r/rho),3);
            g[i*3+1]=-20.0/pow(rho,2)*(p(1)-centers[i](1))*pow((1.0-r/rho),3);
            g[i*3+2]=-20.0/pow(rho,2)*(p(2)-centers[i](2))*pow((1.0-r/rho),3);
        }
    }
}


void evalHRBF_closed(std::vector<Vector3>& pts, std::vector<Vector3>& normals,
                     std::vector<Vector3> &centers, double rho, double eta,
                     std::vector<double>& d)
{
    for(int k=0;k<pts.size();k++){
        d.push_back(0);
    }
    
    std::vector<double> g(centers.size()*3);
    
    for(int j=0;j<pts.size();j++){
        Vector3 x = pts[j];
        
        Wendland_gradphi(x, centers, rho,g);
        for(int i=0;i<centers.size();i++){
            double rho2=pow(rho,2);
            Vector3 nr;
            
            nr(0)=rho2/(20.0+eta*rho2)*normals[i](0);
            nr(1)=rho2/(20.0+eta*rho2)*normals[i](1);
            nr(2)=rho2/(20.0+eta*rho2)*normals[i](2);
            
            double dt=nr(0)*g[i*3]+nr(1)*g[i*3+1]+nr(2)*g[i*3+2];
            
            d[j]=d[j]-dt;
            
        }
    }
}

void Viewer::selectedVertDeformation(double selected_x,double selected_y,double selected_z)
 {
     typedef std::pair<Point, Vector> PointVectorPair;
     std::vector<Vec3> vertNormal, deformationPoints;
     std::vector<PointVectorPair> points;
   meshPtr->computeVertNormals(vertNormal);
   double distance,thr=0.1,d=0.1,alpha=200.0,disp;
   for(unsigned i=0;i<meshPtr->numVerts();i++){
     Vec3 p_neighbor = meshPtr->getVertPos(i);
     distance=sqrt(pow((selected_x-p_neighbor.x),2)+pow((selected_y-p_neighbor.y),2)+pow((selected_z-p_neighbor.z),2));
     if(distance>thr)disp=0;
      else disp=d*exp(-alpha*pow(distance,2));
     //Vec3 tmp(p.x+disp*vertNormal[i].x,p.y+disp*vertNormal[i].y,p.z+disp*vertNormal[i].z);
     //deformationPoints.push_back(tmp);
       Point p(p_neighbor.x+disp*vertNormal[i].x,p_neighbor.y+disp*vertNormal[i].y,p_neighbor.z+disp*vertNormal[i].z);
       Vector v(vertNormal[i].x,vertNormal[i].y,vertNormal[i].z);
                //vertex.push_back(std::make_pair(tmp,vertNormal[i]));
       points.push_back(std::make_pair(p,v));
   }
 

 const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
 CGAL::pca_estimate_normals(points.begin(), points.end(), CGAL::First_of_pair_property_map<PointVectorPair>(),CGAL::Second_of_pair_property_map<PointVectorPair>(),nb_neighbors);
 
std::vector<PointVectorPair> ::iterator unoriented_points_begin =
CGAL::mst_orient_normals(points.begin(), points.end(),
                         CGAL::First_of_pair_property_map<PointVectorPair>(),
                         CGAL::Second_of_pair_property_map<PointVectorPair>(),
                         nb_neighbors);

     
     std::vector<Vector3> points2;
     std::vector<Vector3> normals2;
     
     for(unsigned i=0;i<meshPtr->numVerts();i++){
 


     Vector3 p_tmp(points[i].first.x(), points[i].first.y(), points[i].first.z());
         Vector3 n_tmp(points[i].second.x(), points[i].second.y(), points[i].second.z());
         
         points2.push_back(p_tmp);
         normals2.push_back(n_tmp);
         
     }
     points.erase(unoriented_points_begin, points.end());
     createOFFFile("test.xyz",points2,normals2);
     std::vector<Vector3> structuredGrid;
     Vector3 leftCorner(-1.5,-1.5,-1.5);
     Vector3 rightCorner(1.5,1.5,1.5);
     unsigned int subx = 64;
     unsigned int suby = 64;
     unsigned int subz = 64;
     createGrid(leftCorner, rightCorner, subx, suby, subz, structuredGrid);
     //HRBF_fit<double, 3, Rbf_pow3<double> > hrbf;
    // hrbf.hermite_fit(points2, normals2);
     
     
     // Evaluate on the grid
     std::vector<double> results(structuredGrid.size());
     
   /* for (size_t i = 0; i < structuredGrid.size(); ++i) {
         results[i] = hrbf.eval(structuredGrid[i]);
     }*/
     
  
     double eta=4000;
     const unsigned int nb_neighbors2 = 6; // 1 ring
     double averagespacing = CGAL::compute_average_spacing(points.begin(), points.end(),CGAL::First_of_pair_property_map<PointVectorPair>(),nb_neighbors2);
    
     //evalHRBF_closed(structuredGrid,normals2,points2,averagespacing*10,eta,results);*/
     float dx = (float)(rightCorner[0] - leftCorner[0]) / subx;
     float dy = (float)(rightCorner[1] - leftCorner[1]) / suby;
     float dz = (float)(rightCorner[2] - leftCorner[2]) / subz;

     double *resultarray= new double[results.size()];
     for(int i=0;i<results.size();i++){
         resultarray[i]=results[i];
     }
   //  ciso->GenerateSurface(resultarray, 0, subx-1, suby-1, subz-1, dx, dy, dz);
     
     //poisson reconstruction
     double sm_angle = 20.0;
     double sm_radius = 30;
     double sm_distance = 0.375;
     Poisson_reconstruction_function function(points.begin(), points.end(),CGAL::First_of_pair_property_map<PointVectorPair>(),CGAL::Second_of_pair_property_map<PointVectorPair>());
     Point inner_point = function.get_inner_point();
     Sphere bsphere = function.bounding_sphere();
     FT radius = std::sqrt(bsphere.squared_radius());
     FT sm_sphere_radius = 5.0 * radius;
     FT sm_dichotomy_error = sm_distance*averagespacing/1000.0; // Dichotomy error must be << sm_distance
     Surface_3 surface(function,
                       Sphere(inner_point,sm_sphere_radius*sm_sphere_radius),
                       sm_dichotomy_error/sm_sphere_radius);
     

     
     CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,sm_radius*averagespacing,sm_distance*averagespacing);
     STr tr;
     C2t3 c2t3(tr);
     CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                             surface,                              // implicit surface
                             criteria,                             // meshing criteria
                             CGAL::Manifold_with_boundary_tag());  // require manifold mesh
     std::ofstream out("poisson.off");
     Polyhedron output_mesh;
     CGAL::output_surface_facets_to_polyhedron(c2t3, output_mesh);
     out << output_mesh;
     
     delete[] resultarray;
     //createVTKFile("test.vtk", subx, suby, subz, structuredGrid, results);

   //createOFFFile2("test2.off");
 //std::cout<<ciso->m_nVertices<<std::endl;
     //std::cout<<(double)ciso->m_ppt3dVertices[0][0]<<std::endl;
     delete(meshPtr);
     meshPtr=new TriMesh;
     //meshPtr->setData(ciso->m_ppt3dVertices,ciso->m_nVertices,ciso->m_piTriangleIndices,ciso->m_nTriangles);
     meshPtr->normalize();
     glDisable(GL_COLOR_MATERIAL);
     std::cout<<"drawed"<<std::endl;

 }


