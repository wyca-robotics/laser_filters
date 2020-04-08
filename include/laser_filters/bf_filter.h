#ifndef BF_FILTER_H
#define BF_FILTER_H

#include "ros/ros.h"

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

// PCL include

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cstdlib>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

struct L_points {
  geometry_msgs::Point p;
  double range;
  double intensity;
};

namespace laser_filters {
  class LaserScanBilateralFilter : public filters::FilterBase<sensor_msgs::LaserScan>
  {

  public:
    double _sigmaI, _sigmaS;
    double distance_threshold;
    int _nb_check_point;
    double _radius;

    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> _ddr;
    bool configure() {
      ros::NodeHandle nh_("~/BilateralFilter");
      _ddr = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);


      getParam("sigmaI", _sigmaI);
      getParam("sigmaS", _sigmaS);
      getParam("nbCheckPoint", _nb_check_point);
      getParam("radius", _radius);

      _ddr->registerVariable<double>("SigmaI", &_sigmaI, "sigmaI", 0, 10);
      _ddr->registerVariable<double>("SigmaS", &_sigmaS, "sigmaS", 0, 10);
      _ddr->registerVariable<double>("Radius", &_radius, "radius", 0, 1);
      _ddr->registerVariable<int>("nbCheckPoint", &_nb_check_point, "nbCheckPoint", 0, 100);

      _ddr->publishServicesTopics();

      return true;
    }

    double distance_function(double distance) {
      return 1 / (1 + distance);
    }

    double gaussian(float x, double sigma) {
        return exp(-(pow(x, 2))/(2 * pow(sigma, 2))) / (2 * M_PI * pow(sigma, 2));
    }

    virtual ~LaserScanBilateralFilter(){}

    int getStartInt(int j) {
      return (j > _nb_check_point) ? j - _nb_check_point : 0;
    }

    int getStopInt(int j, int size) {
      return (j + _nb_check_point < size) ? j + _nb_check_point : size;
    }

    sensor_msgs::LaserScan bilateralFilterOctree(sensor_msgs::LaserScan input_scan) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
      // Populate point cloud with LaserScan data
      for(int i = 0; i < input_scan.ranges.size(); i++) {
        pcl::PointXYZ point;
        point.x = input_scan.ranges[i] * cos(input_scan.angle_min + input_scan.angle_increment*i);
        point.y = input_scan.ranges[i] * sin(input_scan.angle_min + input_scan.angle_increment*i);
        point.z = 0;
        pointcloud->points.push_back(point);
      }

      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(double(0.05));
      octree.setInputCloud(pointcloud);
      octree.addPointsFromInputCloud();

      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      double distance_function_res, range_function_res;

      sensor_msgs::LaserScan filtered_scan = input_scan;
      for(std::size_t p = 0 ; p < pointcloud->size() ; p++) {
        double w, Wp = 0;
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        if(octree.radiusSearch(pointcloud->at(p), _radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
          for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
            distance_function_res = distance_function(sqrt(pointRadiusSquaredDistance[i]));
            range_function_res = gaussian(sqrt(pointRadiusSquaredDistance[i]), _sigmaS);
            w = distance_function_res * input_scan.ranges[p];
            filtered_scan.ranges[p] += sqrt( pow((pointcloud->at(pointIdxRadiusSearch[i]).x ), 2)
                                             + pow((pointcloud->at(pointIdxRadiusSearch[i]).y), 2)) * w;
            Wp += w;
          }
        }
        filtered_scan.ranges[p] /= Wp;
      }
      return filtered_scan;

    }

    sensor_msgs::LaserScan bilateralFilter(const sensor_msgs::LaserScan& input_scan) {
      sensor_msgs::LaserScan filtered_scan = input_scan;
      L_points point;
      std::vector<L_points> points;
      // Get x and y coordinates
      for(int i = 0; i < input_scan.ranges.size(); i++) {
        point.p.x = input_scan.ranges[i] * cos(input_scan.angle_min + input_scan.angle_increment*i);
        point.p.y = input_scan.ranges[i] * sin(input_scan.angle_min + input_scan.angle_increment*i);
        point.range = input_scan.ranges[i];
        point.intensity = input_scan.intensities[i];
        points.push_back(point);
      }

      double distance;
      double range_function_res, distance_function_res;
      for (int i = 0; i < points.size(); i++) {
        // Apply the filter for one point
        int j = 0;
        double w, Wp=0;
        for(j = getStartInt(j) ; j < getStopInt(j, input_scan.ranges.size())  ; j++) {
          // We first check if the points are close enough to used in the BF
          distance = sqrt( pow((points[i].p.x -  points[j].p.x), 2) + pow((points[i].p.y -  points[j].p.y), 2));
          if(distance < _radius) {
            distance_function_res = distance_function(distance);
            range_function_res = gaussian(distance, _sigmaS);
            w = distance_function_res * input_scan.ranges[i];
            filtered_scan.ranges[i] += points[j].range * w;
            Wp += w;
          }
        }
        filtered_scan.ranges[i] /= Wp;
      }
      return filtered_scan;
      ROS_INFO("Loop finished");
    }

    bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& output_scan) {
      output_scan = input_scan;
      output_scan = bilateralFilterOctree(input_scan);
    }
  };
};

#endif // BF_FILTER_H
