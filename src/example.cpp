#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "/home/sas2/cws/src/wall_follow/src/wall_follow.cpp"

sensor_msgs::LaserScan l_scans;
sensor_msgs::LaserScan* pl_scans  = &l_scans;

void laserCallback(const sensor_msgs::LaserScan msg){
	*pl_scans = msg;
	ros::spinOnce();
}


int main(int argc, char** argv) {
	std::cout<<"start";
	ros::init(argc, argv, "wall_f");
	ros::NodeHandle n;
	
	ros::Publisher lsp = n.advertise<geometry_msgs::Twist>("cmd_vel", 500);
	ros::Subscriber laser_sub= n.subscribe("scan", 500 ,laserCallback);
	
	WallFollow wf("left", 2.2, 6, 1, 0.5, 0.5, pl_scans, &lsp);
	ros::Rate loop_rate(30);	
	while (ros::ok){
		wf.start();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}