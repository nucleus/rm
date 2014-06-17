#include <util/interface/ApproximationSurface.hpp>
#include <GL/glew.h>
#include <glm/interface/glm.hpp>
#include <glm/interface/gtx/norm.hpp>

util::ApproxSurface::ApproxSurface() : m_radius(0.1), m_k(4)
{
    m_pointCloud = NULL;
}

void util::ApproxSurface::create(Grid* grid, PointCloud* pointCloud)
{
	if (m_pointCloud) {
		clear();
	}
	
    m_grid.create(pointCloud->boundingBox().min(), pointCloud->boundingBox().max(), grid->rows(), grid->cols());
    m_pointCloud = pointCloud;

    updated();
}

void util::ApproxSurface::updated()
{
	/// WLS
    wls();

    /// BEZIER
    bezier();

    /// MLS
	mls();
}

void util::ApproxSurface::setRadius(float radius)
{
    m_radius = radius;
    if (m_grid.size() > 0 && m_pointCloud != 0)
    {
        updated();
    }
}

void::util::ApproxSurface::setK(int k)
{
    m_k = k;
    if (m_grid.size() > 0 && m_pointCloud != 0)
	{
        bezier();
		mls();
	}
}

void util::ApproxSurface::setRenderWLS(bool enable)
{
    m_renderWLS = enable;
}

void util::ApproxSurface::setRenderBezier(bool enable)
{
    m_renderBezier = enable;
}

void util::ApproxSurface::setRenderMLS(bool enable)
{
    m_renderMLS = enable;
}

void util::ApproxSurface::wls()
{
    m_WLSValues = m_grid;

    /// Create b(x) and b(x)b(x).T
	const PointVector& allPoints = m_pointCloud->points();

	m_vecB.resize(allPoints.size());
	m_vecBB.resize(allPoints.size());
	m_vecF.resize(allPoints.size());
	
	for (unsigned i = 0; i < allPoints.size(); i++)
    {
        const Point3f& p = allPoints[i];

        Eigen::VectorXf vec(6);
        vec << 1, p.x, p.y, glm::pow(p.x, 2.0f), p.x*p.y, glm::pow(p.y, 2.0f);
        m_vecB[i] = vec;
        m_vecBB[i] = vec * vec.transpose();
        m_vecF[i] = p.z;
    }

    /// Calculate z values with Weighted Least Squares

    /// Loop through all points in the grid

    float minZ = m_pointCloud->boundingBox().min().z;
    float maxZ = m_pointCloud->boundingBox().max().z;
    float stepZ = (float)(maxZ-minZ)/9;

    for (size_t row=0; row<m_grid.rows(); row++)
    {
        for (size_t col=0; col<m_grid.cols(); col++)
        {
            Point3f pointGrid = m_grid.at(row,col);
            PointCloud::IndexVector indices = m_pointCloud->collectInRadius(pointGrid, m_radius, util::PointCloud::Mode_naive_2d);
			
            /// Create Matrix for solving
            Eigen::VectorXf sumB = Eigen::VectorXf::Zero(6);
            Eigen::MatrixXf sumBB = Eigen::MatrixXf::Zero(6,6);
            Eigen::VectorXf c = Eigen::VectorXf::Zero(6);

            for (int i=0; i<indices.size(); i++)
            {
                /// Wendland
                float dist = glm::distance2(glm::vec2(pointGrid.x, pointGrid.y), glm::vec2(m_vecB[indices[i]][1], m_vecB[indices[i]][2]));
                float w = wendland(dist, m_radius);
                /// Sum
                sumB += w * m_vecB[indices[i]] * m_vecF[indices[i]];
                sumBB += w * m_vecBB[indices[i]];
            }
            if (indices.size() > 0)
            {
                c = sumBB.fullPivLu().solve(sumB);
            }
            Eigen::VectorXf vec(6);
            vec << 1, pointGrid.x, pointGrid.y, glm::pow(pointGrid.x, 2.0f), pointGrid.x*pointGrid.y, glm::pow(pointGrid.y, 2.0f);
			
            m_WLSValues.setZat(row, col, vec.dot(c));
        }
    }

}

void util::ApproxSurface::bezier()
{
    if (m_k < 1) {
       m_k=1;
	}

    int rows = m_grid.rows();
    int cols = m_grid.cols();

    m_BezierNormals.create(m_pointCloud->boundingBox().min(), m_pointCloud->boundingBox().max(), m_k* rows, m_k* cols);
    m_BezierValues.create(m_pointCloud->boundingBox().min(), m_pointCloud->boundingBox().max(), m_k* rows, m_k* cols);

    Point3f bezierSurface[rows*m_k][cols*m_k];
	
    for (size_t x=0; x<rows; x++)
    {
        PointVector rowVec;
        for (size_t y=0; y<cols; y++)
        {
            rowVec.push_back( Point3f( m_WLSValues.at(x, y) ) );
        }

        for (size_t y=0; y<cols*m_k; y++) {
            bezierSurface[x*m_k][y] = decasteljau(rowVec, false, (float)y/((cols*m_k)-1));
        }
    }
    
    for (int y=0; y<cols*m_k; y++)
    {
        PointVector colVec;
        for (size_t x=0; x<rows; x++)
        {
            colVec.push_back( bezierSurface[x*m_k][y] );
        }
        for (int x=0; x<rows*m_k; x++)
        {
            Point3f p = decasteljau(colVec, false, (float)x/((rows*m_k)-1));
            m_BezierValues.setZat(x, y, p.z);
        }
    }
    
    for (int y=0; y<cols*m_k; y++)
    {
        PointVector colVec, rowVec;
        
        for (int x=0; x<rows*m_k; x++)
        {
			PointVector colVec, rowVec;
			for (int i = 0; i < rows*m_k; i++) {
				colVec.push_back( m_BezierValues.at(i,y) );
			}
			for (int i = 0; i < cols*m_k; i++) {
				rowVec.push_back( m_BezierValues.at(x,i) );
			}
			Point3f tangY = decasteljau(colVec, true, (float)x/((rows*m_k)-1));
			Point3f tangX = decasteljau(rowVec, true, (float)y/((cols*m_k)-1));
            Point3f normal = glm::normalize( glm::cross( tangX, tangY ) );
            m_BezierNormals.setAt(x, y, normal);
        }
    }
}

void util::ApproxSurface::mls()
{
	if (m_k < 1)
       m_k=1;

	int rows = m_grid.rows();
    int cols = m_grid.cols();

    m_MLSValues.create(m_pointCloud->boundingBox().min(), m_pointCloud->boundingBox().max(), m_k* rows, m_k* cols);
    m_MLSNormals.create(m_pointCloud->boundingBox().min(), m_pointCloud->boundingBox().max(), m_k* rows, m_k* cols);

	/// Create b(x) and b(x)b(x).T
    const PointVector& allPoints = m_pointCloud->points();
	m_vecB.resize(allPoints.size());
	m_vecBB.resize(allPoints.size());
	m_vecF.resize(allPoints.size());	
	
    for (unsigned i = 0; i < allPoints.size(); i++)
    {
        const Point3f& p = allPoints[i];

        Eigen::VectorXf vec(6);
        vec << 1, p.x, p.y, glm::pow(p.x, 2.0f), p.x*p.y, glm::pow(p.y, 2.0f);
        m_vecB[i] = vec;
        m_vecBB[i] = vec * vec.transpose();
        m_vecF[i] = p.z;
    }

    float minZ = m_pointCloud->boundingBox().min().z;
    float maxZ = m_pointCloud->boundingBox().max().z;
    float stepZ = (float)(maxZ-minZ)/9;

    for (size_t row=0; row<rows*m_k; row++)
    {
        for (size_t col=0; col<cols*m_k; col++)
        {
            Point3f pointGrid = m_MLSValues.at(row,col);
            PointCloud::IndexVector indices = m_pointCloud->collectInRadius(pointGrid, m_radius, util::PointCloud::Mode_naive_2d);

            /// Create Matrix for solving
            Eigen::VectorXf sumB = Eigen::VectorXf::Zero(6);
            Eigen::MatrixXf sumBB = Eigen::MatrixXf::Zero(6,6);
            Eigen::VectorXf c = Eigen::VectorXf::Zero(6);

            for (int i=0; i<indices.size(); i++)
            {
                /// Wendland
                float dist = glm::distance2(glm::vec2(pointGrid.x, pointGrid.y), glm::vec2(m_vecB[indices[i]][1], m_vecB[indices[i]][2]));
                float w = wendland(dist, m_radius);
                /// Sum
                sumB += w * m_vecB[indices[i]] * m_vecF[indices[i]];
                sumBB += w * m_vecBB[indices[i]];
            }
            if (indices.size() > 0)
            {
                c = sumBB.fullPivLu().solve(sumB);
            }
            Eigen::VectorXf vec(6);
            vec << 1, pointGrid.x, pointGrid.y, glm::pow(pointGrid.x, 2.0f), pointGrid.x*pointGrid.y, glm::pow(pointGrid.y, 2.0f);
            m_MLSValues.setZat(row, col, vec.dot(c));		

			float nx = -(c(1)+2*c(3)*pointGrid.x+c(4)*pointGrid.y);
			float ny = -(c(2)+2*c(5)*pointGrid.y+c(4)*pointGrid.x);
			float nz = -1;

			m_MLSNormals.setAt(row, col, Point3f(nx,ny,nz));

		}
    }

}

Point3f util::ApproxSurface::decasteljau(PointVector points, bool calcNormals, float t)
{
	size_t n = points.size();	
	
	for (unsigned j = 1; j < n; j++) {
		if (calcNormals && j == n-2) {
			return points[0] - points[1];
		}
		
		for (unsigned i = 0; i <= n-1-j; i++) {
			points[i] = (1-t) * points[i] + t * points[i+1];
		}
	}
	return points[0];
}

void util::ApproxSurface::draw() const
{
    if (m_renderWLS)
    {
        glColor3f( 0.2f, 0.2f, 1.f );
        int rows = m_grid.rows();
        int cols = m_grid.cols();

        for (size_t row=0; row<rows-1; row++)
        {
            glBegin(GL_TRIANGLE_STRIP);

            Point3f p0(0.0f, 0.0f, 0.0f);
            Point3f p1(0.0f, 0.0f, 0.0f);
            Point3f p2(0.0f, 0.0f, 0.0f);
            Point3f normal(0.0f, 0.0f, 0.0f);

            for (size_t col=0; col<cols; col++)
            {
                if (row % 2 == 0)
                {
                    p1 = m_WLSValues.at( row, col );
                    glVertex3f( p1.x, p1.y, p1.z );

                    p2 = m_WLSValues.at( row+1, col );
                    glVertex3f( p2.x, p2.y, p2.z );
                }
                else
                {
                    p2 = m_WLSValues.at( row, cols-(col+1) );
                    glVertex3f( p2.x, p2.y, p2.z );

                    p1 = m_WLSValues.at( row+1, cols-(col+1) );
                    glVertex3f( p1.x, p1.y, p1.z );
                }

                if (row > 0)
                {
                    normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
                }

                glNormal3f(normal.x, normal.y, normal.z);
                p0 = p2;
            }

            glEnd();
        }
    }

    if(m_renderBezier)
    {
        glColor3f( 0.2f, 0.2f, 1.f );

        int rows = m_BezierValues.rows();
        int cols = m_BezierValues.cols();

        for (size_t row=0; row<rows-1; row++)
        {
            glBegin(GL_QUAD_STRIP);

            Point3f p(0.0f, 0.0f, 0.0f);
            Point3f n(0.0f, 0.0f, 0.0f);

            for (size_t col=0; col<cols; col++)
            {
                if (row % 2 == 0)
                {
                    p = m_BezierValues.at(row, col);
                    glVertex3f( p.x, p.y, p.z );
                    n = m_BezierNormals.at(row, col);
                    glNormal3f(n.x, n.y, n.z);

                    p = m_BezierValues.at(row+1, col);
                    glVertex3f( p.x, p.y, p.z );
                    n = m_BezierNormals.at(row+1, col);
                    glNormal3f( n.x, n.y, n.z );
                }
                else
                {
                    p = m_BezierValues.at(row, cols-(col+1));
                    glVertex3f( p.x, p.y, p.z );
                    n = m_BezierNormals.at(row, cols-(col+1));
                    glNormal3f( n.x, n.y, n.z );

                    p = m_BezierValues.at(row+1, cols-(col+1));
                    glVertex3f( p.x, p.y, p.z );
                    n = m_BezierNormals.at(row+1, cols-(col+1));
                    glNormal3f( n.x, n.y, n.z );
                }
            }

            glEnd();
        }
    }

	if(m_renderMLS)
	{

       glColor3f( 0.2f, 0.2f, 1.f );

        int rows = m_MLSValues.rows();
        int cols = m_MLSValues.cols();

        for (size_t row=0; row<rows-1; row++)
        {
            glBegin(GL_QUAD_STRIP);

            Point3f p(0.0f, 0.0f, 0.0f);
            Point3f n(0.0f, 0.0f, 0.0f);

            for (size_t col=0; col<cols; col++)
            {
                if (row % 2 == 0)
                {
                    p = m_MLSValues.at(row, col);
                    glVertex3f( p.x, p.y, p.z );
                    n = m_MLSNormals.at(row, col);
                    glNormal3f(n.x, n.y, n.z);

                    p = m_MLSValues.at(row+1, col);
                    glVertex3f( p.x, p.y, p.z );
                    n = m_MLSNormals.at(row+1, col);
                    glNormal3f( n.x, n.y, n.z );
                }
                else
                {
                    p = m_MLSValues.at(row, cols-(col+1));
                    glVertex3f( p.x, p.y, p.z );
                    n = m_MLSNormals.at(row, cols-(col+1));
                    glNormal3f( n.x, n.y, n.z );

                    p = m_MLSValues.at(row+1, cols-(col+1));
                    glVertex3f( p.x, p.y, p.z );
                    n = m_MLSNormals.at(row+1, cols-(col+1));
                    glNormal3f( n.x, n.y, n.z );
                }
            }

            glEnd();
        }
	}
}

