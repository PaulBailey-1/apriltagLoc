
#include "Plane.h"

Plane::Plane() {
    m_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    m_coefficients = new pcl::ModelCoefficients;
    m_inliers = new pcl::PointIndices;

    m_cloud->width  = 4;
    m_cloud->height = 1;
    m_cloud->points.resize (cloud->width * cloud->height);

    m_seg.setOptimizeCoefficients(true);
    m_seg.setModelType(pcl::SACMODEL_PLANE);
    m_seg.setMethodType(pcl::SAC_RANSAC);
    m_seg.setDistanceThreshold(0.05);
}

void Plane::fill(Point[] points) {
    for (int i = 0; i < 4; i++) {
        m_cloud->points[i].x = points.x;
        m_cloud->points[i].y = points.y;
        m_cloud->points[i].z = points.z;
    }
}

void Plane::fit() {
    seg.setInputCloud (m_cloud);
    seg.segment (*inliers, *coefficients);
}
