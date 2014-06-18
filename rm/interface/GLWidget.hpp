/*
 * Header file for CG2EX2 OpenGL QT widget.
 */

#ifndef EX2_GLWIDGET_H
#define EX2_GLWIDGET_H

#include <QGLWidget>

#include <util/interface/Util.hpp>
#include <util/interface/ImplicitSurface.hpp>
#include <util/interface/Camera.hpp>
#include <util/interface/Ray.hpp>

class GLWidget : public QGLWidget {
	Q_OBJECT
public:
	explicit GLWidget(QWidget * parent = 0);
	
public:
	void mouseMoveEvent(QMouseEvent* event);
	void mousePressEvent(QMouseEvent* event);

	void wheelEvent(QWheelEvent* event);

protected:
	virtual void initializeGL();
	virtual void resizeGL(int w, int h);
	virtual void paintGL();

protected:	
	void computeWLS();
	void generateRays(util::RayVector& _rays);
	void traceRays(const util::RayVector& _rays);
	
protected:
	void _shade(const Point3f& p, const Point3f& n, Point3f& color) const;
	
protected:
	Point3f m_queryPoint;
	float m_radius;
	
	util::ImplicitSurface m_surface;
	
	PointNormalData m_rayMarchPoints;
	
protected:
	Camera m_camera;
	QPoint m_lastPosition;
	glm::vec2 m_rotation;
	
	glm::vec4 m_lightPos;
	float m_lightAngle;
	glm::vec4 m_lightAmbient;
	glm::vec4 m_lightDiffuse;
	glm::vec4 m_lightSpecular;
	
	bool m_drawPoints;
	bool m_drawGrid;
	bool m_drawWLS;
	bool m_drawRM;
	
	bool m_recomputeWLS;
	
	bool m_enableGPU;
	
signals:

public slots:
	void setTreeLeafSize(int size) { m_surface.setTreeLeafSize(size); }
	void setTreeMaxDepth(int depth) { m_surface.setTreeMaxDepth(depth); }

	void setGridDimX(int dim) { m_surface.setGridDimX(dim); m_recomputeWLS = true; updateGL(); }
	void setGridDimY(int dim) { m_surface.setGridDimY(dim); m_recomputeWLS = true; updateGL(); }
	void setGridDimZ(int dim) { m_surface.setGridDimZ(dim); m_recomputeWLS = true; updateGL(); }

	void setRadius(double radius) { m_radius = radius; m_recomputeWLS = true; }
	void setRMSteps(int steps) { m_surface.setRayMarchingSteps(steps); }

	void loadFile();

	void setRenderPoints(bool enable) { m_drawPoints = enable; updateGL(); }
	void setRenderGrid(bool enable) { m_drawGrid = enable; updateGL(); }
	void setRenderWLS(bool enable) { m_drawWLS = enable; updateGL(); }
	void setRenderRM(bool enable) { m_drawRM = enable; updateGL(); }
	
	void enableCPUDevice() { m_enableGPU = false; m_recomputeWLS = true; }
	void enableGPUDevice() { m_enableGPU = true; m_recomputeWLS = true; }
	
	void wls();	
	void raymarch();
};

#endif // GLWIDGET_H
