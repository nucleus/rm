/*
 * Header file for Camera class.
 */

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <util/interface/Util.hpp>

class Camera {
public:
	Camera(): eye(0.0f), center(0.0f), up(0.0f) {}
	Camera(const Point3f& _eye, const Point3f& _center, const Point3f& _up): eye(_eye), center(_center), up(_up) {}
	
public:
	Point3f eye, center, up;
};

#endif