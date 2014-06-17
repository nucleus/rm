/*
 * Header file for Grid3D class.
 */

#ifndef GRID_3D_HPP
#define GRID_3D_HPP

#include <util/interface/BoundingBox.hpp>
#include <util/interface/Util.hpp>

namespace util {
	
	class Grid3D {
	public:
		struct Voxel3D {
			Voxel3D(): coords(0.0f), normal(0.0f), value(0.0f) {}
			Voxel3D(const Point3f& _p): coords(_p), normal(0.0f), value(0.0f) {}
			Voxel3D(const Point3f& _p, float _v): coords(_p), normal(0.0f), value(_v) {}
			Voxel3D(const Point3f& _p, const Point3f& _n, float _v): coords(_p), normal(_n), value(_v) {}
			
			Point3f coords;
			Point3f normal;
			float value;
		};
		
	public:
		Grid3D(const Point3f& _min, const Point3f& _max, const Point3i& _dims);
		
		Grid3D(const Point3f& _min, const Point3f& _max, unsigned _x, unsigned _y, unsigned _z);
		
		~Grid3D();
		
	public:
		Point3f getGridPoint(const Point3i& idx) const;
		Point3f getGridPoint(unsigned x, unsigned y, unsigned z) const;
		
		float& value(const Point3i& idx);
		float& value(unsigned x, unsigned y, unsigned z);
		const float& value(const Point3i& idx) const;
		const float& value(unsigned x, unsigned y, unsigned z) const;
		
		Point3f& normal(const Point3i& idx);
		Point3f& normal(unsigned x, unsigned y, unsigned z);
		const Point3f& normal(const Point3i& idx) const;
		const Point3f& normal(unsigned idx, unsigned y, unsigned z) const;
		
		Point3i dimensions() const;
		
		size_t size() const { return m_grid.size(); }
		
	public:
		void draw() const;
		
	protected:
		void build();
		
	protected:
		Point3i m_dims;
		
		BoundingBox m_bounds;
		
		std::vector< Voxel3D > m_grid;
	};

}
#endif