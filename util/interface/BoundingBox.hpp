/*
 * Header file for geometric bounding box class.
 */

#ifndef BOUNDING_BOX_HPP
#define BOUNDING_BOX_HPP

#include <iostream>

#include <glm/interface/glm.hpp>
#include <util/interface/Util.hpp>

namespace util {

class BoundingBox {
public:
	BoundingBox();
	BoundingBox(glm::vec3 _min, glm::vec3 _max);
	
public:
	glm::vec3& min() { return m_min; }
	const glm::vec3& min() const { return m_min; }
	
	glm::vec3& max() { return m_max; }
	const glm::vec3& max() const { return m_max; }
	
	glm::vec3 extents() const { return m_max - m_min; }
	glm::vec3 center() const { return (m_max + m_min) / 2.0f; }
	
public:
	void extend(const glm::vec3& p);	
	void extend(const BoundingBox& box);
	
	bool contains(const glm::vec3& p) const;
	
	Axis dominantAxis() const;
	
	void split(Axis axis, float pos, BoundingBox& left, BoundingBox& right) const;
	
public:
	void draw() const;

protected:
	glm::vec3 m_min, m_max;
};

}

std::ostream& operator<<(std::ostream& out, const util::BoundingBox& box);

#endif
