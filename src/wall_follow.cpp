/* Author: Shamsudeen A Sodangi */

#include "wall_follow.h"

WallFollow::WallFollow (std::string direction, double hold_distance, double sensor_range, double width, 
						double v_max, double rot_v_max, sensor_msgs::LaserScan* pscans, ros::Publisher* publisher){
	del_theta = 0.1;
	aligned = false;
	align_active = false;
	align_rotate = false;
	double min_hold = 1.5; //TODO define min
	double max_hold = 6;
	if (sensor_range > hold_distance){
		this->sensor_range = sensor_range;
	}
	else std::cout<<"sensor range must be greater than hold range \n";
	if (width > 0){
		this->width = width;
	}
	else std::cout<<"width of robot footprint must be greater than zero \n";
	if (hold_distance > min_hold || hold_distance < max_hold){
		this->hold_distance = hold_distance;
	}
	else std::cout<<"hold_distance in constructor out of bounds \n";
	if (direction == "left" || direction == "right"){
		this->direction = direction;
	}
	else {
		std::cout<<"invalid direction in constructor. Use 'left' or 'right' \n";
		std::cout<<"WallFollow(direction, hold distance, width, sensor_range, max lin vel, max rotational vel, laser scans pointer, publisher) \n";
	}
	if(v_max > 0) this->v_max = v_max;
	else {
		std::cout<<"maximum velocity in constructor must be greater than zero, default of one set \n";
		this->v_max = 1;
	}
	if(rot_v_max > 0) this->rot_v_max = rot_v_max;
	else {
		std::cout<<"maximum rotational velocity in constructor must be greater than zero, default of one set \n";
		std::cout<<"WallFollow(direction, hold distance, width, sensor_range, max lin vel, max rotational vel, laser scans pointer, publisher) \n";
		this->v_max = 1;
	}
	this->pscans = pscans;
	this->publisher = publisher;
	
	hold_range[0] = hold_distance;
	hold_range[1] = hold_distance / cos(pi/8);
	hold_range[2] = hold_distance / cos(pi/4);
	hold_range[3] = hold_distance;
	hold_range[4] = hold_distance / cos(pi/8);
	hold_range[5] = hold_distance / cos(pi/4);
	hold_range[6] = hold_distance;
	error_ratio = 5;
	max_range = sensor_range *1.2;
}

void WallFollow::checkAlignment(){
	getScans(pscans, del_theta);
	if(direction == "left"){
		if (range[0] > 0.1 && range[0] < 1.5 * hold_distance) aligned = true;
	}
	else{
		if (range[6] > 0.1 && range[6] < 1.5 * hold_distance) aligned = true;
	}
}
void WallFollow::align(){
	getScans(pscans, del_theta);
	
	geometry_msgs::Twist displacement;
	double margin = hold_distance / 5;
	

	
	if(scan_id == 3){//forward laser
		getScans(pscans, del_theta);
		vel.linear.x = ((range[3] - hold_distance)/ hold_distance) * v_max;
		if(hold_distance - range[3] > -margin && hold_distance - range[3] < margin){ //move straight and find obstacle
			align_rotate = true;
			vel.linear.x = 0;
		}
		if(align_rotate){ //rotate till aligned with side (after proximity is achieved)
			if(direction == "left"){
				if(hold_distance - range[0] > -margin && hold_distance - range[0] < margin){
					aligned = true;
				}
				else{
					vel.angular.z =  -rot_v_max * (range[0] - hold_distance) / max_range;
				}				
			}
			else{
				if(hold_distance - range[6] > -margin && hold_distance - range[6] < margin){
					aligned = true;
				}
				else{
					vel.angular.z =  rot_v_max * (range[6] - hold_distance) / max_range;
				}
			}
		double rot_v_min = rot_v_max/10;
		if(vel.angular.z < 0 && vel.angular.z > -rot_v_min) vel.angular.z = -rot_v_min;
		if(vel.angular.z > 0 && vel.angular.z < rot_v_min) vel.angular.z = rot_v_min;
		}		
	}else{//rotate till forward laser is closest
		
		vel.angular.z =  -rot_v_max * (range[6] - hold_distance) / max_range; 
		align_active = false;
	}
	
	if(vel.linear.x > v_max) vel.linear.x = v_max;
	emergencyStop();
	publisher->publish(vel);
	
}

void WallFollow::start(){
	checkAlignment();
	if(!aligned){
		if(!align_active){
			scan_id = 0;
			double closest_point = max_range;
			for(int i = 0; i < sizeof(range)/sizeof(*range); i++){
				if (range[i] < closest_point) {
					closest_point = range[i];
					scan_id = i;
				}
			}
			align_active = true;
		}
		align();
	}
	else{
		float z_turn;
		float stopping_dist = 2; //TODO cal from dynamics based on max vel 
		float max_speed_dist = hold_distance; //apply max speed beyond this range, min dist req to stop from max speed
		float stopping_theta = 1; //TODO cal from dynamics based on max rit vel
		getScans(pscans, del_theta); //updates range
		double z_margin = hold_range[0] / error_ratio;
	
		if(direction == "left"){
			z_turn = 8 * (hold_range[0] -range[0])/10 + (hold_range[1] -range[1]) /10 + (hold_range[2] -range[2]) /10; //weight 8 1 1
			
			if (sqrt(pow((hold_range[0] -range[0]), 2)) < z_margin){//go straight
				vel.angular.z = 0;
			}
			else{
				vel.angular.z = -rot_v_max * z_turn / max_range;
			}
		}
	
		if(direction == "right"){
			z_turn = 8 * (hold_range[6] -range[6])/10 + (hold_range[5] -range[5]) /10 + (hold_range[4] -range[4]) /10; //weight 8 1 1
			//std::cout<<" z_turn :"<<z_turn<<" \n";
			
			if (sqrt(pow((hold_range[6] -range[6]), 2)) < z_margin){ //go straight
				vel.angular.z = 0;
			}
			else{
				vel.angular.z = rot_v_max * z_turn / max_range;
			}
			
		}
		if(vel.angular.z > rot_v_max) vel.angular.z = rot_v_max;
		if(vel.angular.z < -rot_v_max) vel.angular.z = -rot_v_max;

		
		//incase of seprated multiple obstacles
		bool gap;
		if(direction == "left"){
			if( range[0] == max_range || range[1] == max_range || range[2] == max_range){
				gap = true;
			}
			else gap =false;
		}
		else{
			if( range[4] == max_range || range[5] == max_range || range[6] == max_range){
				gap = true;
			}
			else gap =false;
		}
		
		if(range[3] < max_speed_dist * 1.5 && !gap){//influence dist
			if(range[3] <= max_speed_dist){
				vel.linear.x = 0; //P TODO PID
				if(direction == "left"){
					vel.angular.z = -rot_v_max;
				}
				else {
					vel.angular.z = rot_v_max;
				}
			}
			else{
				vel.linear.x = v_max * (range[3] / max_range ); //P TODO PID
				if(direction == "left"){
					if(vel.angular.z <= z_margin){//CW
						vel.angular.z = (1 - vel.linear.x / v_max)  * (-1 * rot_v_max); //TODO PID Variable effect
					}
				}
				else {
					if(vel.angular.z >= z_margin){
						vel.angular.z = (1 - vel.linear.x / v_max) * rot_v_max; //TODO PID Variable effect
					}
				}
			}
		}
		else{
			if(vel.angular.z == 0){
				vel.linear.x = v_max;
			}
			else{
				vel.linear.x = v_max * (rot_v_max - (sqrt(pow(vel.angular.z, 2)))) /rot_v_max  ;//TODO PID
				if(vel.linear.x < v_max/10) vel.linear.x = v_max/10;
			}
		}
		//safe for local minima
		if(range [3] <= hold_distance ) vel.linear.x = 0;
		
		zeroRange();
		emergencyStop();
		publisher->publish(vel);
	}
}

void WallFollow::getScans(sensor_msgs::LaserScan* scans, double del_theta){
	if(scans->angle_min <= -pi/2 && scans->angle_max >= pi/2){
		double scan_theta[] = {-pi/2, -3*pi/8, -pi/4 , -del_theta/2, pi/4, 3*pi/8,  pi/2 - del_theta};
		for(int ii =0; ii < sizeof(scan_theta)/sizeof(*scan_theta); ii++){
			int initial = sqrt(pow((scan_theta[ii] - scans->angle_min), 2))/ scans->angle_increment;
			for (int i = 0; i < del_theta/scans->angle_increment; i++){	
				range[ii] += scans->ranges[initial + i];
			}
			range[ii] = range[ii] / (del_theta/scans->angle_increment);
			if (range[ii] > max_range) {range[ii] = max_range;} 
			if (range[ii] < 0 ) {range[ii] = 0;} 
		}
		
		double scan_theta_[] = {atan(width/(2*hold_distance)), -del_theta/2, -atan(width/(2*hold_distance)) -del_theta};
		for(int i =0; i < 3; i++){
			int ini = sqrt(pow((scan_theta_[i] - scans->angle_min), 2))/ scans->angle_increment;
			for(int ii = 0; ii < del_theta/scans->angle_increment; ii++){
				stop_scan[i] += scans->ranges[ini + ii];
			}
			stop_scan[i] = stop_scan[i] /  (del_theta/scans->angle_increment);
			if (stop_scan[i] > max_range) {stop_scan[i] = max_range;} 
			if (stop_scan[i] < 0 ) {stop_scan[i] = 0;} 
		}
	}
	else {
		std::cout<<"laser data is less than 180 degrees \n";
	}
}

void WallFollow::emergencyStop(){
	for(int i =0; i < 3; i++){
		if(stop_scan[i] < hold_distance  ){
			vel.linear.x = 0.1 * v_max;
		}
		if(stop_scan[i] < 0.3 * hold_distance  ){
			vel.linear.x = 0;
		}
	}	
	publisher->publish(vel);
}

void WallFollow::zeroRange(){
	for(int i = 0; i < sizeof(range)/sizeof(*range); i++){	
		range[i] = 0;
	}
}
