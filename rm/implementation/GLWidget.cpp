/*
 * Implementation file for CG2EX2 OpenGL QT widget.
 */

#include <rm/interface/GLWidget.hpp>
#include <glm/interface/gtc/type_ptr.hpp>

#include <GL/glu.h>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	#include <time.h>
#else
	#include <sys/time.h>
#endif

#include <qnumeric.h>
#include <QMouseEvent>
#include <QFileDialog>
#include <QMessageBox>
#include <QElapsedTimer>

#include <iostream>
#include <cmath>
#include <algorithm>

#ifdef REPORT_LEVEL
#undef REPORT_LEVEL
#endif

#define REPORT_LEVEL 1
#include <util/interface/Debug.hpp>

/////////////////////////////////////////////////////////////////////////

GLWidget::GLWidget(QWidget * parent):
  QGLWidget(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer | QGL::Rgba | QGL::AlphaChannel | QGL::DirectRendering), parent),
  m_queryPoint(0.0f),
  m_radius(0.0f),
  m_camera(Point3f(0.0f, 0.0f, -1.5f), Point3f(0.0f), Point3f(0.0f, 1.0f, 0.0f)),
  m_rotation(0.0f),
  m_lightPos(-1.0f, -1.0f, -10.0f, 1.0),
  m_lightAngle(0.0f),
  m_lightAmbient(0.4, 0.4, 0.4, 1.0),
  m_lightDiffuse(0.8, 0.8, 0.8, 1.0),
  m_lightSpecular(0.2, 0.2, 0.2, 1.0),
  m_drawPoints(true),
  m_drawGrid(false),
  m_drawWLS(false),
  m_drawRM(false),
  m_drawRMPoints(100000),
  m_immediateMode(false),
  m_recomputeWLS(true),
  m_enableGPU(false) {

	setFocusPolicy(Qt::StrongFocus);
	setMouseTracking(true);
}

/////////////////////////////////////////////////////////////////////////

void GLWidget::initializeGL() {
	/// Light

	glLightfv(GL_LIGHT0, GL_AMBIENT, glm::value_ptr(m_lightAmbient));
	glLightfv(GL_LIGHT0, GL_DIFFUSE, glm::value_ptr(m_lightDiffuse));
	glLightfv(GL_LIGHT0, GL_SPECULAR, glm::value_ptr(m_lightSpecular));
	glLightfv(GL_LIGHT0, GL_POSITION, glm::value_ptr(m_lightPos));

	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);

	// enable normalization of vertex normals
	glEnable(GL_NORMALIZE);

	// smooth surfaces
	glShadeModel(GL_SMOOTH);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	glMatrixMode(GL_MODELVIEW);

	setAutoBufferSwap(true);

	glClearColor(0.0,0.0,0.0,1.0);
}

void GLWidget::resizeGL(int w, int h) {
	w = w & ~1;
	h = h & ~1;
	
	glViewport(0, 0, (GLint)w, (GLint)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70.0, (float) w / (float) h, 1.0, 100.0);

	updateGL();
}

void GLWidget::paintGL() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(m_camera.eye.x, m_camera.eye.y, m_camera.eye.z, 
			  m_camera.center.x, m_camera.center.y, m_camera.center.z,
			  m_camera.up.x, m_camera.up.y, m_camera.up.z);

	// rotate object
	glRotatef(m_rotation.x, 1.0f, 0.0f, 0.0f);
	glRotatef(m_rotation.y, 0.0f, 1.0f, 0.0f);
	
	// rotate light
	glPushMatrix();
	glTranslatef(m_surface.center().x, m_surface.center().y, m_surface.center().z);
	glRotatef(m_lightAngle, 0.0f, 1.0f, 0.0f);
	glm::vec4 light = m_lightPos - glm::vec4(m_surface.center(), 1.0f);
	glLightfv(GL_LIGHT0, GL_POSITION, glm::value_ptr(light));
	glPopMatrix();

	if (m_drawPoints) {
		m_surface.draw();
	}
	
	if (m_drawGrid) {
		if (m_surface.size()) {
			m_surface.grid().draw();
		}
	}
	
	if (m_drawWLS) {
		util::Grid3D& grid = m_surface.grid();
		Point3i dims = grid.dimensions();
		
		glPointSize(5.0f);
		glBegin(GL_POINTS);
		for (unsigned z = 0; z < dims.z; z++) {
			for (unsigned y = 0; y < dims.y; y++) {
				for (unsigned x = 0; x < dims.x; x++) {
					const Point3f& p = grid.getGridPoint(x, y, z);
					float v = grid.value(x, y, z);
					if (v < 0.0f) {
						glColor3f(1.0f, 0.0f, 0.0f);
					} else if (v == 0.0f) {
						glColor3f(0.0f, 1.0f, 0.0f);
					} else {
						glColor3f(0.0f, 0.0f, 1.0f);
					}
					glVertex3f(p.x, p.y, p.z);
				}
			}
		}
		glEnd();
		glPointSize(1.0f);
	}
	
	if (m_drawRM) {
		if (m_rayMarchPoints.size()) {
			unsigned step = m_rayMarchPoints.size() / m_drawRMPoints;
			if (!step) {
				step = 1;
			}
			
			glBegin(GL_POINTS);
			glColor3f(0.0f, 0.0f, 1.0f);
			for (unsigned i = 0; i < m_rayMarchPoints.size(); i += step) {
				Point3f color;
				
				const Point3f& p = m_rayMarchPoints[i].first;
				const Point3f& n = m_rayMarchPoints[i].second;
				
				_shade(p, n, color);
				
				glColor3f(color.x, color.y, color.z);
				glVertex3f(p.x, p.y, p.z);
			}
			glEnd();
		}
	}
}

/////////////////////////////////////////////////////////////////////////

void GLWidget::mouseMoveEvent(QMouseEvent * event) {
	float dx = (event->x() - m_lastPosition.x()) / 10.0f;
	float dy = (event->y() - m_lastPosition.y()) / 10.0f;

	if (event->buttons() == Qt::LeftButton) {
		if (event->modifiers() & Qt::ShiftModifier) {
			m_lightAngle += dx;
		} else {
			m_rotation += glm::vec2(8.0f * dy, 8.0f * dx);
			
			if (m_enableGPU && m_immediateMode && m_drawRM) {
				raymarch();
			}
		}
		updateGL();
	}

	m_lastPosition = event->pos();
}

void GLWidget::mousePressEvent(QMouseEvent * event) {
	if (event->button() != Qt::NoButton) {
		m_lastPosition = event->pos();
		updateGL();
	}
}

void GLWidget::wheelEvent(QWheelEvent* event) {
	float delta = (float)event->delta();
	util::BoundingBox box = m_surface.boundingBox();
	float span = box.extents()[box.dominantAxis()];
	m_camera.eye.z += span/delta;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////

void GLWidget::loadFile() {
	QString qfile = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.*)"));
	std::string filename = qfile.toAscii().data();
    const bool readNormalizedModel = true;

	m_surface.readOFF(filename, readNormalizedModel);
	m_recomputeWLS = true;
	
	// update camera position based on model
	float xSpan = m_surface.boundingBox().extents().x;
	m_camera.eye.z = -5.0f * (xSpan / (tan(80.0f / 180.0f * 3.14159f)));
    
	updateGL();
}

void GLWidget::wls() {
	if (m_recomputeWLS) {
		float time = 0.0f;
		GET_TIME( computeWLS() )
		report("GLWidget::wls(): WLS calculation took " << time / 1000.0f << " ms");
		m_recomputeWLS = false;
	}
}

void GLWidget::raymarch() {
	report("GLWidget::raymarch()");
	
	float time = 0.0f;
	
	// weighted least squares
	if (m_recomputeWLS) {
		float time = 0.0f;
		GET_TIME( computeWLS() )
		report("GLWidget::raymarch(): WLS calculation took " << time / 1000.0f << " ms");
		m_recomputeWLS = false;
	}
	
	if (m_enableGPU) {
		
		unsigned width, height;
		glm::vec3 origin;
		PointVector directions;
		
		// generate ray directions, origin, and width and height of the viewport
		{
			float time = 0.0f;
			GET_TIME( generateRaysGPU(origin, directions, width, height) )
			report("GLWidget::raymarch(): Ray generation took " << time / 1000.0f << " ms");
		}
		
		// trace rays
		{
			float time = 0.0f;
			GET_TIME( traceRaysGPU(origin, directions, width, height) )
			report("GLWidget::raymarch(): Ray tracing took " << time / 1000.0f << " ms");
		}
		
		
	} else {
		util::RayVector rays;
		
		// generate rays
		{
			float time = 0.0f;
			GET_TIME( generateRaysCPU(rays) )
			report("GLWidget::raymarch(): Ray generation took " << time / 1000.0f << " ms");
		}
		
		// trace rays
		{
			float time = 0.0f;
			GET_TIME( traceRaysCPU(rays) )
			report("GLWidget::raymarch(): Ray tracing took " << time / 1000.0f << " ms");
		}
	}
	
	report("GLWidget::raymarch(): Completed, found " << m_rayMarchPoints.size() << " intersection points");
	updateGL();
}

void GLWidget::computeWLS() {
	report("GLWidget::computeWLS()");
	
	util::Grid3D& grid = m_surface.grid();
	Point3i dims = grid.dimensions();
	
	#pragma omp parallel for schedule(dynamic)
	for (unsigned z = 0; z < dims.z; z++) {
		for (unsigned y = 0; y < dims.y; y++) {
			for (unsigned x = 0; x < dims.x; x++) {
				const Point3f& p = grid.getGridPoint(x, y, z);
				util::ImplicitSurface::IndexVector indices = m_surface.collectInRadius(p, m_radius);
				PointVector points, normals;
				std::vector<float> values;
				m_surface.getPoints(indices, points);
				m_surface.getValues(indices, values);
				m_surface.getNormals(indices, normals);
				
				float sum = 0.0f;
				Point3f normal(0.0f);
				unsigned nonZeroNormals = 0;
				for (unsigned i = 0; i < points.size(); i++) {
					float w = util::wendland(glm::distance(p, points[i]), m_radius);
					sum += w * values[i];
					if (normals[i] != Point3f(0.0f)) {
						normal += w * normals[i];
						nonZeroNormals++;
					}
				}
				if (points.size()) {
					sum /= points.size();
				}
				if (nonZeroNormals) {
					normal = normal / (float)nonZeroNormals;
				}
				
				grid.value(x, y, z) = sum;
				grid.normal(x, y, z) = normal;
			}
		}
	}	
}

void GLWidget::traceRaysGPU(const glm::vec3& _origin, const PointVector& _directions, unsigned _width, unsigned _height) {
	report("GLWidget::traceRaysGPU()");
	m_surface.intersectGPU(_origin, _directions, _width, _height, m_rayMarchPoints);
}

void GLWidget::generateRaysGPU(glm::vec3& _origin, PointVector& _directions, unsigned& _width, unsigned& _height) {
	report("GLWidget::generateRaysGPU()");

	// retrieve rendering matrices
	int viewport[4];
	double projection[16], modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
	int w = viewport[2], h = viewport[3];

	_directions.resize(4);

	// retrieve camera location
	Point3f origin;
	origin.x = -(modelview[0] * modelview[12] + modelview[1] * modelview[13] + modelview[2] * modelview[14]);
	origin.y = -(modelview[4] * modelview[12] + modelview[5] * modelview[13] + modelview[6] * modelview[14]);
	origin.z = -(modelview[8] * modelview[12] + modelview[9] * modelview[13] + modelview[10] * modelview[14]);

	// retrieve points in four corners of the viewport
	glm::dvec3 topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint;
	gluUnProject(0, h, 1.0, modelview, projection, viewport,
				&topLeftPoint[0],&topLeftPoint[1],&topLeftPoint[2]);
	gluUnProject(w, h, 1.0, modelview, projection, viewport,
				&topRightPoint[0],&topRightPoint[1],&topRightPoint[2]);
	gluUnProject(0, 0, 1.0, modelview, projection, viewport,
				&bottomLeftPoint[0],&bottomLeftPoint[1],&bottomLeftPoint[2]);
	gluUnProject(w, 0, 1.0, modelview, projection, viewport,
				&bottomRightPoint[0],&bottomRightPoint[1],&bottomRightPoint[2]);
	
	// return directions
	_directions[0] = glm::normalize(glm::vec3(topLeftPoint) - origin);
	_directions[1] = glm::normalize(glm::vec3(topRightPoint) - origin);
	_directions[2] = glm::normalize(glm::vec3(bottomLeftPoint) - origin);
	_directions[3] = glm::normalize(glm::vec3(bottomRightPoint) - origin);
	
	// return origin, width, height
	_origin = origin;
	_width = w;
	_height = h;
}

void GLWidget::traceRaysCPU(const util::RayVector& _rays) {
	report("GLWidget::traceRaysCPU()");
	m_surface.intersectCPU(_rays, m_rayMarchPoints);
}

static glm::vec3 bivariateBilerp(const glm::vec3& tl, const glm::vec3& tr, const glm::vec3& bl, const glm::vec3& br, float x, float y) {
	glm::vec3 top = (1-x) * tl + x * tr;
	glm::vec3 bottom = (1-x) * bl + x * br;
	return ((1-y) * top + y * bottom);
}

void GLWidget::generateRaysCPU(util::RayVector& _rays) {
	report("GLWidget::generateRaysCPU()");

	// retrieve rendering matrices
	int viewport[4];
	double projection[16], modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
	int w = viewport[2], h = viewport[3];

	_rays.resize(w*h);

	// retrieve camera location
	Point3f origin;
	origin.x = -(modelview[0] * modelview[12] + modelview[1] * modelview[13] + modelview[2] * modelview[14]);
	origin.y = -(modelview[4] * modelview[12] + modelview[5] * modelview[13] + modelview[6] * modelview[14]);
	origin.z = -(modelview[8] * modelview[12] + modelview[9] * modelview[13] + modelview[10] * modelview[14]);

	// retrieve points in four corners of the viewport
	glm::dvec3 topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint;
	gluUnProject(0, h, 1.0, modelview, projection, viewport,
				&topLeftPoint[0],&topLeftPoint[1],&topLeftPoint[2]);
	gluUnProject(w, h, 1.0, modelview, projection, viewport,
				&topRightPoint[0],&topRightPoint[1],&topRightPoint[2]);
	gluUnProject(0, 0, 1.0, modelview, projection, viewport,
				&bottomLeftPoint[0],&bottomLeftPoint[1],&bottomLeftPoint[2]);
	gluUnProject(w, 0, 1.0, modelview, projection, viewport,
				&bottomRightPoint[0],&bottomRightPoint[1],&bottomRightPoint[2]);
	
	// compute directions
	glm::vec3 rayDirTopLeft = glm::vec3(topLeftPoint) - origin;
	glm::vec3 rayDirTopRight = glm::vec3(topRightPoint) - origin;
	glm::vec3 rayDirBottomLeft = glm::vec3(bottomLeftPoint) - origin;
	glm::vec3 rayDirBottomRight = glm::vec3(bottomRightPoint) - origin;

	// generate rays using bivariate bilerps between directions in each pixel
	for (int y = 0; y < h; y++) {
		
		float diffY = (float)y / (float)(h-1);
		for (int x = 0; x < w; x++) {
			util::Ray& ray = _rays[y*w+x];
			
			float diffX = (float)x / (float)(w-1);
			
			ray.o = origin;
			ray.d = glm::normalize(bivariateBilerp(rayDirTopLeft, rayDirTopRight, rayDirBottomLeft, rayDirBottomRight, diffX, diffY));
		}
	}
	
	report("GLWidget::generateRays(): Top left ray dir = " << _rays[0].d);
	report("GLWidget::generateRays(): Top right ray dir = " << _rays[w-1].d);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

// implements blinn-phong shading model
void GLWidget::_shade(const Point3f& p, const Point3f& n, Point3f& color) const {
	color = Point3f(1.0f, 0.0f, 0.0f);
	return;
	
	double modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

	// retrieve camera location
	Point3f cam;
	cam.x = -(modelview[0] * modelview[12] + modelview[1] * modelview[13] + modelview[2] * modelview[14]);
	cam.y = -(modelview[4] * modelview[12] + modelview[5] * modelview[13] + modelview[6] * modelview[14]);
	cam.z = -(modelview[8] * modelview[12] + modelview[9] * modelview[13] + modelview[10] * modelview[14]);
	
	// get incident directions
	glm::vec3 L = glm::normalize(glm::vec3(m_lightPos) - p);
	glm::vec3 H = glm::normalize(glm::normalize(cam-p) + L);
	float NdotH = glm::dot(n, H);

	// ambient color
	glm::vec3 amb(m_lightAmbient);

	// diffuse color
	float LdotN = std::max(glm::dot(L, n), 0.0f);
	glm::vec3 diff = glm::vec3(m_lightDiffuse) * LdotN;

	// specular component, scale by 4.0 to get visible highlights
	float HdotN = std::max(glm::dot(H, n), 0.0f);
	glm::vec3 spec = glm::vec3(m_lightSpecular) * (float)pow(HdotN, 4.0f);

	color = glm::clamp(amb + diff + spec, 0.0f, 1.0f);
}
