/*
 * Header file for utility classes and functions.
 */

#ifndef UTIL_HPP
#define UTIL_HPP

#include <algorithm>
#include <iostream>
#include <vector>
#include <set>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	#include <time.h>
	#include <windows.h>
#else
	#include <sys/time.h>
#endif

#define GLM_FORCE_RADIANS

#include <glm/interface/glm.hpp>
#include <glm/interface/gtc/type_precision.hpp>

#define FOR_EACH_AXIS for(int axis = 0; axis < 3; axis++)

enum Axis {
	X = 0,
	Y,
	Z,
	W
};

typedef glm::vec3 Point3f;
typedef glm::vec3 Point3d;
typedef glm::i32vec3 Point3i;

typedef glm::vec3 Point2f;
typedef glm::i32vec2 Point2i;

typedef std::vector< Point3f > PointVector;
typedef std::set< Point3f > PointSet;

typedef std::vector< Point3f* > PointPtrVector;

typedef glm::vec3 Color3f;
typedef glm::vec4 Color4f;

typedef std::vector< Color3f > ColorVector;

namespace util {
	struct Indents {
		Indents(unsigned _indents): indents(_indents) {}
		unsigned indents;
	};
	
	struct PointDimComparator {
		PointDimComparator(Axis axis): m_axis(axis) {}
		bool operator()(Point3f* const & lhs, Point3f* const & rhs) {
			return (*lhs)[m_axis] < (*rhs)[m_axis];
		}
		Axis m_axis;
	};
	
	Point3f median(PointPtrVector& _data, Axis axis);
	
	std::string toString(Axis axis);

	double elapsedTime(timeval& start, timeval& end);
	
	float wendland(float d, float h);
}

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)

#define GET_TIME(_OP) { SYSTEMTIME start, end; \
	GetSystemTime(&start); \
	_OP; \
	GetSystemTime(&end); \
	time = (end.wSecond-start.wSecond)+(end.wMilliseconds-start.wMilliseconds)/1000000.0f; \
	}

#else

#define GET_TIME(_OP) { timeval start, end; \
	gettimeofday(&start, 0); \
	_OP; \
	gettimeofday(&end, 0); \
	time = util::elapsedTime(start, end); \
	}

#endif

std::ostream& operator<<(std::ostream& out, const glm::vec3& vec);
std::ostream& operator<<(std::ostream& out, const util::Indents& indents);

#endif
