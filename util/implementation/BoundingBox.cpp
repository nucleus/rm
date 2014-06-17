/*
 * Implementation file for geometric bounding box class.
 */

#include <algorithm>
#include <GL/glew.h>

#include <util/interface/BoundingBox.hpp>

util::BoundingBox::BoundingBox(): m_min(0.0f), m_max(0.0f) {
	
}

util::BoundingBox::BoundingBox(glm::vec3 _min, glm::vec3 _max): m_min(_min), m_max(_max) {
	
}

void util::BoundingBox::extend(const glm::vec3& p) {
	FOR_EACH_AXIS {
		if (p[axis] < m_min[axis]) { m_min[axis] = p[axis]; }
		if (p[axis] > m_max[axis]) { m_max[axis] = p[axis]; }
	}
}

void util::BoundingBox::extend(const BoundingBox& box) {
	FOR_EACH_AXIS {
		m_max[axis] = std::max(box.max()[axis], m_max[axis]);
		m_min[axis] = std::min(box.min()[axis], m_min[axis]);
	}
}

bool util::BoundingBox::contains(const glm::vec3& p) const {
	FOR_EACH_AXIS {
		if (!(p[axis] > m_min[axis] && p[axis] < m_max[axis])) { return false; }
	}
	return true;
}

Axis util::BoundingBox::dominantAxis() const {
	glm::vec3 diff = extents();
	if (diff.x > diff.y) {
		if (diff.x > diff.z) { return X; } else { return Z; }
	} else {
		if (diff.y > diff.z) { return Y; } else { return Z; }
	}
}

void util::BoundingBox::split(Axis axis, float pos, BoundingBox& left, BoundingBox& right) const {
	left.min() = m_min;
	left.max() = m_max;
	left.max()[axis] = pos;
	
	right.max() = m_max;
	right.min() = m_min;
	right.min()[axis] = pos;
}

void util::BoundingBox::draw() const {
	glBegin(GL_LINES);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_min.y, m_min.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_max.y, m_min.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_min.y, m_min.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_min.y, m_max.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_min.y, m_min.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_min.y, m_min.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_max.y, m_max.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_min.y, m_max.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_max.y, m_max.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_max.y, m_max.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_max.y, m_max.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_max.y, m_min.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_max.y, m_min.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_max.y, m_max.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_max.y, m_min.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_max.y, m_min.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_min.y, m_min.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_min.y, m_max.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_min.y, m_min.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_max.y, m_min.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_max.y, m_max.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_min.y, m_max.z);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_min.x, m_min.y, m_max.z);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(m_max.x, m_min.y, m_max.z);
	glEnd();
}

std::ostream& operator<<(std::ostream& out, const util::BoundingBox& box) {
	out << "BoundingBox(min=" << box.min() << ",max=" << box.max() << ")";
	return out;
}
