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
TriMesh*        Viewer::meshPtr = 0;
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
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1., 1.);
    drawPolygons();
    glDisable(GL_POLYGON_OFFSET_FILL);
    if (renderWireframe) drawWireframe();
    if (renderSelected ) drawSelectedVerts();
    //drawSelectedVerts();
    glPopAttrib();
}

void 
Viewer::drawPolygons()
{
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
    if (it == selectedVert.end())
        selectedVert.insert(index);
    else
        selectedVert.erase(it);
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

void Viewer::selectedVertDeformation(double selected_x,double selected_y,double selected_z)
 {
     typedef std::pair<Point, Vector> PointVectorPair;
     std::vector<Vec3> vertNormal, deformationPoints;
     std::vector<PointVectorPair> points;
   //std::vector<vec3Pair> vertex;
   meshPtr->computeVertNormals(vertNormal);
   double distance,thr=1,d=1.0/13.0,alpha=1.0,disp;
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
points.erase(unoriented_points_begin, points.end());
     
     std::vector<Vector3> points2;
     std::vector<Vector3> normals2;
     
     for(unsigned i=0;i<meshPtr->numVerts();i++){
 


     Vector3 p_tmp(points[i].first.x(), points[i].first.y(), points[i].first.z());
         Vector3 n_tmp(points[i].second.x(), points[i].second.y(), points[i].second.z());
         
         points2.push_back(p_tmp);
         normals2.push_back(n_tmp);
         
     }
     std::vector<Vector3> structuredGrid;
     Vector3 leftCorner(-1.5,-1.5,-1.5);
     Vector3 rightCorner(1.5,1.5,1.5);
     unsigned int subx = 20;
     unsigned int suby = 20;
     unsigned int subz = 20;
     createGrid(leftCorner, rightCorner, subx, suby, subz, structuredGrid);
     HRBF_fit<double, 3, Rbf_pow3<double> > hrbf;
     hrbf.hermite_fit(points2, normals2);
     
     
     // Evaluate on the grid
     std::vector<double> results(structuredGrid.size());
     for (size_t i = 0; i < structuredGrid.size(); ++i) {
         results[i] = hrbf.eval(structuredGrid[i]);
         std::cout<<results[i]<<std::endl;
     }
     float dx = (float)(rightCorner[0] - leftCorner[0]) / subx;
     float dy = (float)(rightCorner[1] - leftCorner[1]) / suby;
     float dz = (float)(rightCorner[2] - leftCorner[2]) / subz;
     CIsoSurface <short> *ciso = new CIsoSurface <short> ();

     
 }


