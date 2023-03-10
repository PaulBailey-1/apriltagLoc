#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "Point.h"

class Plane {
public:

    Plane();

    void fill(Point[] points);
    void fit();

private:

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::ModelCoefficients::Ptr m_coefficients;
    pcl::PointIndices::Ptr m_inliers;
    pcl::SACSegmentation<pcl::PointXYZ> m_seg;

};