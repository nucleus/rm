 /*
 * Source file for BezierCurve class.
 */

#include <GL/glew.h>

#include <util/interface/Bezier.hpp>

#include <cassert>
#include <cmath>

static unsigned fac(unsigned n) {
	static unsigned table[] = {1, 1, 2, 6, 24, 120, 720, 5040};
	return table[n];
}

static float bernstein(unsigned n, unsigned i, float t) {
	float binom = fac(n) / (fac(n-i) * fac(i));
	if (i == 0) { 
		return binom * pow(1-t, n);
	} else if (i == n) {
		return binom * pow(t, i);
	} else {
		return binom * pow(t, i) * pow(1-t, n-i);
	}
}

util::CubicBezierCurve::CubicBezierCurve(): m_degree(4), m_rasterized(false) {
	m_controlPoints.resize(m_degree);
	m_controlPoints[0] = Point2f(-0.75f, -0.25f);
	m_controlPoints[1] = Point2f(-0.25f, 0.25f);
	m_controlPoints[2] = Point2f(0.25f, 0.25f);
	m_controlPoints[3] = Point2f(0.75f, -0.25f);
}

util::CubicBezierCurve::~CubicBezierCurve() {}

void util::CubicBezierCurve::rasterize(unsigned k, RasterizationMode mode) {
	switch (mode) {
	case Mode_direct:
		_rasterizeDirect(k); break;
	case Mode_casteljau:
		_rasterizeCasteljau(k); break;
	default:
		break;
	}
	
	m_rasterized = true;
}

void util::CubicBezierCurve::setControlPoint(unsigned i, const Point2f& p) {
	assert(i < m_degree);
	m_controlPoints[i] = p;
	m_rasterized = false;
}

Point2f util::CubicBezierCurve::getControlPoint(unsigned i) const {
	assert(i < m_degree);
	return m_controlPoints[i];
}

Point2f& util::CubicBezierCurve::operator[](unsigned i) {
	assert(i < m_degree);
	m_rasterized = false;
	return m_controlPoints[i];
}

const Point2f& util::CubicBezierCurve::operator[](unsigned i) const {
	assert(i < m_degree);
	return m_controlPoints[i];
}

unsigned util::CubicBezierCurve::getClosestControlPointIdx(const Point2f& point) const {
	float distance = std::numeric_limits<float>::max();
	unsigned idx = 0;
	for (unsigned i = 0; i < m_controlPoints.size(); i++) {
		float tmp = glm::distance2(m_controlPoints[i], point);
		if (tmp < distance) {
			distance = tmp;
			idx = i;
		}
	}
	return idx;
}

void util::CubicBezierCurve::_rasterizeDirect(unsigned k) {
	m_rasterizedPoints.resize(k);
	
	float step = 1.0 / (float)k;
	float t = 0.0f;
	for (unsigned i = 0; i < k; i++, t += step) {
		Point2f point(0.0f);
		for (unsigned j = 0; j < m_degree; j++) {
			float bern = bernstein(m_degree-1, j, t);
			point += bern * m_controlPoints[j];
		}
		m_rasterizedPoints[i] = point;
	}
}

void util::CubicBezierCurve::_rasterizeCasteljau(unsigned k) {

}

void util::CubicBezierCurve::draw() {
	if (!m_rasterized) {
		rasterize();
	}

	glPointSize(3.0);
	glBegin(GL_POINTS);
	for (unsigned i = 0; i < m_controlPoints.size(); i++) {
		glVertex2f(m_controlPoints[i].x, m_controlPoints[i].y);
	}
	glEnd();

	glPointSize(0.3);
	glBegin(GL_POINTS);
	for (unsigned i = 0; i < m_rasterizedPoints.size(); i++) {
		glVertex2f(m_rasterizedPoints[i].x, m_rasterizedPoints[i].y);
	}
	glEnd();
}

 
