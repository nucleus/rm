#ifndef RAY_HPP
#define RAY_HPP

#include <util/interface/Util.hpp>

namespace util {
	
    class Ray {
    public:
        Ray(): o(0.0f), d(0.0f), tNear(0.0f), tFar(std::numeric_limits<float>::max()) {}
        Ray(const Point3f& _o, const glm::vec3& _d): o(_o), d(_d), tNear(0.0f), tFar(std::numeric_limits<float>::max()) {}

	public:
		Point3f operator()(float t) const {
			if (t < tNear || t > tFar) {
				return Point3f(0.0f);
			}
			return o + t * d;
		}
		
	public:
		Point3f o, d;
		float tNear, tFar;
    };
	
	typedef std::vector<Ray> RayVector;
}

#endif // GRID_H
