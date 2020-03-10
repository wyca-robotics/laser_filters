#ifndef WYCA_SHADOW_FILTER_H
#define WYCA_SHADOW_FILTER_H

#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>


namespace laser_filters
{
class LaserScanWycaShadowFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  int _window_size;
  double _angular_distance_threshold, _range_percentage;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> _ddr;


  bool configure()
  {
    ros::NodeHandle nh_("~/WycaShadowFilter");

    _ddr = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);

    getParam("window_size", _window_size);
    getParam("angular_distance_threshold", _angular_distance_threshold);
    getParam("range_percentage", _range_percentage);

    _ddr->registerVariable<int>("Window_size", &_window_size, "Window size", 0, 20);
    _ddr->registerVariable<double>("Angular_distance_threshold", &_angular_distance_threshold, "Angular distance threshold", 0.0, 0.05);
    _ddr->registerVariable<double>("Range_percentage", &_range_percentage, "Normal threshold", 0.0, 1.0);
    _ddr->publishServicesTopics();

    return true;
  }

  ~LaserScanWycaShadowFilter() {

  }

  // Check point direction return true if the range are evoling in a single direction, return false otherwise
  bool checkPointDirection(std::vector<double> points) {
    double base_sign = points[0] - points[1];

    for(int i = 1 ; i < points.size() - 1; i++) {
      if(std::signbit(points[i] - points[i+1]) != std::signbit(base_sign)) {
        return false;
      }
    }
    return true;
  }

  // Check point range
  bool checkPointRange(std::vector<double> points) {

    double range_max = *std::max_element(points.begin(), points.end());
    for(int i = 0 ; i < points.size() - 1 ; i++) {
      if(abs(points[i] - points[i+1]) > range_max*_range_percentage) {
        return false;
      }
    }
    return true;
  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan) {

    double first_x, first_y;
    double last_x, last_y;

    double pts_deleted=0;

    std::pair<double, double> point_vec, lidar_vec;

    double x_div, y_div;

    filtered_scan = input_scan;

    for(int i = 0 ; i < input_scan.ranges.size() ; i+=_window_size) {

      std::vector<double> current_window(input_scan.ranges.begin() + i, input_scan.ranges.begin() + i + _window_size + 1);

      if(checkPointDirection(current_window)) {// && checkPointRange(current_window)) {
        first_x = input_scan.ranges[i] * cos(input_scan.angle_min + i * input_scan.angle_increment);
        first_y = input_scan.ranges[i] * sin(input_scan.angle_min + i * input_scan.angle_increment);

        last_x = input_scan.ranges[i + _window_size] * cos(input_scan.angle_min + (i + _window_size) * input_scan.angle_increment);
        last_y = input_scan.ranges[i + _window_size] * sin(input_scan.angle_min + (i + _window_size) * input_scan.angle_increment);

        point_vec = std::make_pair<double, double>(last_y - first_y, last_x - first_x);
        lidar_vec = std::make_pair<double, double>(last_y - 0, last_x - 0);

        x_div = abs(point_vec.second/lidar_vec.second);
        y_div = abs(point_vec.first/lidar_vec.first);


//        ROS_INFO_STREAM("div : " << x_div/y_div);
        if((abs(x_div / y_div) > 0.0) && (abs(x_div / y_div) < 10.0)) {
          for(int j = 0 ; j < _window_size ; j++) {
            pts_deleted++;
            filtered_scan.ranges[i+j] = filtered_scan.range_max + 1;
            filtered_scan.intensities[i+j] = 0;
          }
        } else {
          ROS_INFO_STREAM("Div : " << abs(x_div / y_div));
        }
      }
    }

//    ROS_INFO_STREAM("Points taken away : " << pts_deleted);
    return true;
  }
};
}

#endif // WYCA_SHADOW_FILTER_H
