# wall_follow-ROS
use laser data from a mobile robot base to move parallel to an obstacle by sending velocity commands.  

Example Usage 
WallFollow robot("left", 2.2, 6, 1, 0.5, 0.5, pl_scans, &lsp);
	ros::Rate loop_rate(30);	
	while (ros::ok){
		robot.start();
		ros::spinOnce();
		loop_rate.sleep();
		//break loop if some condition
	}
