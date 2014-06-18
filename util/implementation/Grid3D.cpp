/*
 * Source file for Grid3D class.
 */

#include <util/interface/Grid3D.hpp>

#include <GL/glew.h>

#ifdef REPORT_LEVEL
#undef REPORT_LEVEL
#endif

#define REPORT_LEVEL 1
#include <util/interface/Debug.hpp>

util::Grid3D::Grid3D(const Point3f& _min, const Point3f& _max, const Point3i& _dims):
	m_dims(_dims), m_bounds(_min, _max) {
	
	if (!(m_dims == Point3i(0))) {
		build();
	}
	
}

util::Grid3D::Grid3D(const Point3f& _min, const Point3f& _max, unsigned _x, unsigned _y, unsigned _z):
	m_dims(_x, _y, _z), m_bounds(_min, _max) {
	
	if (!(m_dims == Point3i(0))) {
		build();
	}
}

util::Grid3D::~Grid3D() {
	m_gridPoints.clear();
	m_gridData.clear();
}
		
Point3f util::Grid3D::getGridPoint(const Point3i& idx) const {
	return getGridPoint(idx.x, idx.y, idx.z);
}
Point3f util::Grid3D::getGridPoint(unsigned x, unsigned y, unsigned z) const {
	return m_gridPoints[z * (m_dims.x * m_dims.y) + y * m_dims.x + x];
}

float& util::Grid3D::value(const Point3i& idx) {
	return value(idx.x, idx.y, idx.z);
}
float& util::Grid3D::value(unsigned x, unsigned y, unsigned z) {
	return m_gridData[z * (m_dims.x * m_dims.y) + y * m_dims.x + x].value;
}

const float& util::Grid3D::value(const Point3i& idx) const {
	return value(idx.x, idx.y, idx.z);
}
const float& util::Grid3D::value(unsigned x, unsigned y, unsigned z) const {
	return m_gridData[z * (m_dims.x * m_dims.y) + y * m_dims.x + x].value;
}

Point3f& util::Grid3D::normal(const Point3i& idx) {
	return normal(idx.x, idx.y, idx.z);
}
Point3f& util::Grid3D::normal(unsigned x, unsigned y, unsigned z) {
	return m_gridData[z * (m_dims.x * m_dims.y) + y * m_dims.x + x].normal;
}
const Point3f& util::Grid3D::normal(const Point3i& idx) const {
	return normal(idx.x, idx.y, idx.z);
}
const Point3f& util::Grid3D::normal(unsigned x, unsigned y, unsigned z) const {
	return m_gridData[z * (m_dims.x * m_dims.y) + y * m_dims.x + x].normal;
}

void util::Grid3D::draw() const {	
	glBegin(GL_LINES);
	glColor3f(0.0f, 1.0f, 0.0f);
	
	// draw all the X-Y slices
	for (unsigned z = 0; z < m_dims.z; z++) {
		for (unsigned y = 0; y < m_dims.y; y++) {
			Point3f start = getGridPoint(0, y, z);
			glVertex3f(start.x, start.y, start.z);
			glVertex3f(start.x + m_bounds.extents().x, start.y, start.z);
		}
		
		for (unsigned x = 0; x < m_dims.x; x++) {
			Point3f start = getGridPoint(x, 0, z);
			glVertex3f(start.x, start.y, start.z);
			glVertex3f(start.x, start.y + m_bounds.extents().y, start.z);
		}
	}
	
	// draw Z lines
	for (unsigned y = 0; y < m_dims.y; y++) {
		for (unsigned x = 0; x < m_dims.x; x++) {
			Point3f start = getGridPoint(x, y, 0);
			glVertex3f(start.x, start.y, start.z);
			glVertex3f(start.x, start.y, start.z + m_bounds.extents().z);
		}
	}
	
	glEnd();
}
		
void util::Grid3D::build() {
	m_gridPoints.clear();
	m_gridPoints.resize(m_dims.x * m_dims.y * m_dims.z);
	m_gridData.resize(m_dims.x * m_dims.y * m_dims.z);
	
	float stepX = m_bounds.extents().x / (m_dims.x-1);
	float stepY = m_bounds.extents().y / (m_dims.y-1);
	float stepZ = m_bounds.extents().z / (m_dims.z-1);
	
	for (unsigned z = 0; z < m_dims.z; z++) {
		for (unsigned y = 0; y < m_dims.y; y++) {
			for (unsigned x = 0; x < m_dims.x; x++) {
				Point3f p(m_bounds.min().x + x * stepX, m_bounds.min().y + y * stepY, m_bounds.min().z + z * stepZ);
				m_gridPoints[z * (m_dims.x * m_dims.y) + y * m_dims.x + x] = p;
			}
		}
	}
}