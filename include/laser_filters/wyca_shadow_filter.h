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
  double _min_threshold, _max_threshold;
  ros::Publisher deleted_scan_pub;

  bool _use_range, _use_direction;

  bool configure()
  {
    ros::NodeHandle nh_("~/WycaShadowFilter");

    deleted_scan_pub = nh_.advertise<sensor_msgs::LaserScan>("scan_deleted", 10);
    _ddr = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);

    getParam("window_size", _window_size);
    getParam("angular_distance_threshold", _angular_distance_threshold);
    getParam("range_percentage", _range_percentage);
    getParam("max_threshold", _max_threshold);
    getParam("min_threshold", _min_threshold);
    getParam("use_range", _use_range);
    getParam("use_direction", _use_direction);

    _ddr->registerVariable<int>("Window_size", &_window_size, "Window size", 0, 20);
    _ddr->registerVariable<double>("Angular_distance_threshold", &_angular_distance_threshold, "Angular distance threshold", 0.0, 0.05);
    _ddr->registerVariable<double>("Range_percentage", &_range_percentage, "Normal threshold", 0.0, 1.0);
    _ddr->registerVariable<double>("Min_threshold", &_min_threshold, "Min threshold", 0.0, 10.0);
    _ddr->registerVariable<double>("Max_threshold", &_max_threshold, "Max threshold", 0.0, 10.0);
    _ddr->registerVariable<bool>("Use_range", &_use_range, "Use range conditon");
    _ddr->registerVariable<bool>("Use_direction", &_use_direction, "Use direction condition");
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
    double point_x=0, point_y=0;

    double pts_deleted=0;

    std::pair<double, double> point_vec, lidar_vec;

    double x_div, y_div;

    sensor_msgs::LaserScan deleted_scan;

    filtered_scan = input_scan;
    deleted_scan = input_scan;
    deleted_scan.ranges.clear();
    deleted_scan.ranges.clear();

    int if_in=0, else_in=0;

    for(int i = 0 ; i < input_scan.ranges.size() ; i+=_window_size) {

      std::vector<double> current_window(input_scan.ranges.begin() + i, input_scan.ranges.begin() + i + _window_size + 1);

      if(((checkPointRange(current_window)) && _use_range && checkPointDirection(current_window) && _use_direction) ||
         (checkPointRange(current_window) && _use_range && !_use_direction) ||
         (checkPointDirection(current_window) && _use_direction && !_use_range)) {
        first_x = input_scan.ranges[i] * cos(input_scan.angle_min + i * input_scan.angle_increment);
        first_y = input_scan.ranges[i] * sin(input_scan.angle_min + i * input_scan.angle_increment);

        last_x = input_scan.ranges[i + _window_size] * cos(input_scan.angle_min + (i + _window_size) * input_scan.angle_increment);
        last_y = input_scan.ranges[i + _window_size] * sin(input_scan.angle_min + (i + _window_size) * input_scan.angle_increment);

        point_vec = std::make_pair<double, double>(last_x - first_x, last_y - first_y);

        for(int j = 0 ; j < _window_size ; j++) {
          point_x += input_scan.ranges[i + j] * cos(input_scan.angle_min + (i + j) * input_scan.angle_increment);
          point_y += input_scan.ranges[i + j] * sin(input_scan.angle_min + (i + j) * input_scan.angle_increment);
        }

        point_x /= _window_size;
        point_y /= _window_size;

        lidar_vec = std::make_pair<double, double>(point_x - 0, point_y - 0);


        x_div = abs(point_vec.first/lidar_vec.first);
        y_div = abs(point_vec.second/lidar_vec.second);


//        ROS_INFO_STREAM("div : " << x_div/y_div);
        if((abs(x_div / y_div) > _min_threshold) && (abs(x_div / y_div) < _max_threshold)) {
          for(int j = 0 ; j < _window_size ; j++) {
            if_in++;
            deleted_scan.ranges.push_back(filtered_scan.ranges[i+j]);
            deleted_scan.intensities.push_back(filtered_scan.intensities[i+j]);
            filtered_scan.ranges[i+j] = filtered_scan.range_max + 1;
            filtered_scan.intensities[i+j] = 0;
          }
        } else {
          for(int j = 0 ; j < _window_size ; j++) {
//            else_in++;
            deleted_scan.ranges.push_back(deleted_scan.range_max + 1);
            deleted_scan.intensities.push_back(0);
          }
        }
      } else {
        for(int j = 0 ; j < _window_size ; j++) {
//          else_in++;
          deleted_scan.ranges.push_back(deleted_scan.range_max + 1);
          deleted_scan.intensities.push_back(0);
        }
      }
    }

//    ROS_INFO_STREAM("If " << if_in << ", else " << else_in);
    deleted_scan_pub.publish(deleted_scan);
    return true;
  }
};
}

#endif // WYCA_SHADOW_FILTER_H
