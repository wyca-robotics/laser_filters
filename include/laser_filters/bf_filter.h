#ifndef BF_FILTER_H
#define BF_FILTER_H

#include "ros/ros.h"

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>


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

      _ddr->registerVariable<double>("Sigma i", &_sigmaI, "sigmaI");
      _ddr->registerVariable<double>("Sigma S", &_sigmaS, "sigmaS");
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

    bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan) {
      filtered_scan = input_scan;

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

      double w, Wp=0;
      double distance;
      double range_function_res, distance_function_res;
      for (int i = 0; i < points.size(); i++) {
        // Apply the filter for one point
        for(int j = (i > _nb_check_point) ? i - _nb_check_point : 0 ; j < (i + _nb_check_point < input_scan.ranges.size()) ? i + _nb_check_point : input_scan.ranges.size() ; j++) {
          // We first check if the points are close enough to used in the BF
          distance = sqrt( pow((points[i].p.x -  points[j].p.x), 2) + pow((points[i].p.y -  points[j].p.y), 2));
          if(distance < _radius) {
            distance_function_res = distance_function(distance);
            w * distance_function_res * range_function_res;
            filtered_scan.ranges[i] += points[i].range;
            Wp += w;
          }
          filtered_scan.ranges[i] /= Wp;
        }
      }

    }
  };
};

#endif // BF_FILTER_H
