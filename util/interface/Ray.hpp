#ifndef RAY_HPP
#define RAY_HPP

#include <util/interface/Util.hpp>

namespace util {
	
    class Ray {
    public:
        Ray(): o(0.0f), d(0.0f) {}
        Ray(const Point3f& _o, const glm::vec3& _d): o(_o), d(_d) {}

	public:
		Point3f operator()(float t) const {
			return o + t * d;
		}
		
	public:
		Point3f o, d;
    };
	
	typedef std::vector<Ray> RayVector;
}

#endif // GRID_H
