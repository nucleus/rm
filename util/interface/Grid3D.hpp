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
			Voxel3D(): normal(0.0f), value(0.0f) {}
			Voxel3D(float _v): normal(0.0f), value(_v) {}
			Voxel3D(const Point3f& _n, float _v):  normal(_n), value(_v) {}
			
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
		
		Point3i dimensions() const { return m_dims; }
		size_t size() const { return m_gridPoints.size(); }
		const BoundingBox& bounds() const { return m_bounds; }
		
		// for CUDA interoperability
		const Voxel3D* data() const { return &m_gridData[0]; }
		
	public:
		void draw() const;
		
	protected:
		void build();
		
	protected:
		Point3i m_dims;
		
		BoundingBox m_bounds;
		
		// std::vector< Voxel3D > m_grid;
		
		std::vector< Point3f > m_gridPoints;
		std::vector< Voxel3D > m_gridData;
	};

}
#endif