#ifndef APPROXSURFACE_H
#define APPROXSURFACE_H

#include <eigen/Dense>
#include <util/interface/Util.hpp>
#include <util/interface/PointCloud.hpp>
#include <util/interface/Grid.hpp>

namespace util {
    class ApproxSurface {

    public:
        // Constructor
        ApproxSurface();

        // Functions
        void create(Grid* grid, PointCloud* pointCloud);
		void updated();
        void setRadius(float radius);
        void setK(int k);
        void setRenderWLS(bool enable);
        void setRenderBezier(bool enable);
        void setRenderMLS(bool enable);

        // OpenGL
        void draw() const;
		
		void clear() {
			m_grid.clear();
			m_pointCloud = NULL;
			m_vecBB.clear();
			m_vecB.clear();
			m_vecF.clear();
			m_WLSValues.clear();
			m_BezierValues.clear();
			m_BezierNormals.clear();
			m_MLSNormals.clear();
		}

    protected:
        void wls();
        void bezier();
        void mls();
        Point3f decasteljau(PointVector points, bool calcNormals, float t);

    protected:
        Grid m_grid;
        PointCloud* m_pointCloud;
        float m_radius;
        int m_k;
        std::vector<Eigen::MatrixXf> m_vecBB;
        std::vector<Eigen::VectorXf> m_vecB;
        std::vector<float> m_vecF;
        Grid m_WLSValues;
        Grid m_BezierValues;
        Grid m_BezierNormals;
        Grid m_MLSValues;
        Grid m_MLSNormals;
        bool m_renderWLS;
        bool m_renderBezier;
        bool m_renderMLS;
    };
}

#endif // APPROXSURFACE_H
