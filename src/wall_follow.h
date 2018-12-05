/* Author: Shamsudeen A Sodangi */

#ifndef wall_follow_H
#define wall_follow_H
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


class WallFollow{

	public:
	WallFollow (std::string, double, double, double, double, double, sensor_msgs::LaserScan*, ros::Publisher*);
	void zeroRange();
	void start();
	void align();
	void emergencyStop();
	void checkAlignment();
	void printTry();
	void printAligned();
	
	private:
	bool aligned, align_rotate, align_active;
	bool print_try, print_aligned;
	int scan_id;
	double max_range, sensor_range, closest_point;
	static const float pi = 22/7;
	double error_ratio, hold_distance, del_theta, width;
	std::string direction; 
	sensor_msgs::LaserScan* pscans;
	float range[7];
	float stop_scan[3];
	float hold_range[7];
	geometry_msgs::Twist vel;
	double v_max;
	double rot_v_max;
	ros::Publisher* publisher;
	
	void getScans(sensor_msgs::LaserScan* scans, double del_theta);

	
	
	

};

#endif