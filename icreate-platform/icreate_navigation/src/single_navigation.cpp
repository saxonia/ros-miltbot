#include "icreate_navigation/single_navigation.h"

namespace icreate {

SingleNavigation::SingleNavigation(): sequence(24){
    // this->ac("move_base", true);
    navigation_mode = -1;
    client = nh_.serviceClient<std_srvs::Empty>(clear_costmap_service);
    requestToCreateTimer = true;
    requestToSetNewGoal = false;
    targetId = 0;
    input_mode = 0;
    // this->sequence = {0,3,5,0};
    sequence[0] = 0;
    sequence[1] = 3;
    sequence[2] = 5;
    sequence[3] = 0;
    
}

SingleNavigation::~SingleNavigation() {

}

// void SingleNavigation::setRobotTarget(move_base_msgs::MoveBaseGoal &goal) {
//     ROS_INFO("setRobotTarget: %lf",goal.target_pose.pose.position.x);
//     target = goal;
// }
void SingleNavigation::setRobotTarget(MoveBaseGoalData &data) {
    ROS_INFO("setRobotTarget: %s",data.getGoalName().c_str());
    target = data;
}

// void SingleNavigation::setRobotTarget(int selected_point) {
//     this->selected_point = selected_point;
//     target_iterator = targets.begin() + selected_point;
//     ROS_INFO("set Robot Target: %s", this->target_name[selected_point].c_str());
//     target.target_pose.pose.position.x    = target_iterator->target_pose.pose.position.x;
//     target.target_pose.pose.position.y    = target_iterator->target_pose.pose.position.y;
//     target.target_pose.pose.orientation.x = target_iterator->target_pose.pose.orientation.x;
//     target.target_pose.pose.orientation.y = target_iterator->target_pose.pose.orientation.y;
//     target.target_pose.pose.orientation.z = target_iterator->target_pose.pose.orientation.z;
//     target.target_pose.pose.orientation.w = target_iterator->target_pose.pose.orientation.w;
// }
bool SingleNavigation::setRobotTarget(int selected_point, std::string type) {
    this->selected_point = selected_point;
    if(type == "Floor") {
        this->target_iterator = targets.begin() + selected_point;
    }
    else if(type == "Lift") {
        this->target_iterator = lifts.begin() + selected_point;
    }
    else {
        return false;
    }
    
    ROS_INFO("set Robot Target: %s", this->target_iterator->getGoalName().c_str());
    target = *(this->target_iterator);
}

// move_base_msgs::MoveBaseGoal SingleNavigation::getRobotTarget() {
//     ROS_INFO("get Robot Target: %s",this->target_name[selected_point].c_str());
//     return targets[selected_point];
// }
MoveBaseGoalData SingleNavigation::getRobotTarget() {
    ROS_INFO("get Robot Target: %s",this->target.getGoalName().c_str());
    return this->target;
}

// void SingleNavigation::setRobotGoal(std::string frame_id) {
//     goal.target_pose.header.frame_id = frame_id;
//     goal.target_pose.header.stamp = ros::Time::now();

//     goal.target_pose.pose.position.x    = target.target_pose.pose.position.x;
//     goal.target_pose.pose.position.y    = target.target_pose.pose.position.y;
//     goal.target_pose.pose.orientation.x = target.target_pose.pose.orientation.x;
//     goal.target_pose.pose.orientation.y = target.target_pose.pose.orientation.y;
//     goal.target_pose.pose.orientation.z = target.target_pose.pose.orientation.z;
//     goal.target_pose.pose.orientation.w = target.target_pose.pose.orientation.w;
//     std::cout << "[AGENT] SET NEW GOAL ! " << std::endl;

// }
void SingleNavigation::setRobotGoal(std::string frame_id) {
    
    goal = this->target.getGoal();    
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();
    std::cout << "[AGENT] SET NEW GOAL ! " << std::endl;
}

move_base_msgs::MoveBaseGoal SingleNavigation::getRobotGoal() {
    return goal;
}

std::string SingleNavigation::doneRobotGoal(std::string robot_state) {
    client.call(clearer);
    std::string state_msg; 
    if(robot_state == "GOING") {
        state_msg = "WAITING";
        this->input_mode = waitParcel;
    }
    else if(robot_state == "WAITING") {
        this->input_mode = waitParcel;
    }
    else if(robot_state == "BACKTOBASE") {
        state_msg = "IDLE";
        this->input_mode = inputUser;
    }
    else if(robot_state == "SENDSUPPLIES") {
        state_msg = "WAITING";
        this->input_mode = waitParcel;
    }
    else if(robot_state == "SINGLERUN") {
        state_msg = "IDLE";
        this->input_mode = inputUser;
    }
    else if(robot_state == "EXECUTESEQ") {
        state_msg = "IDLE";
        this->input_mode = 2;
    }
    else {
        this->input_mode = 99;
    }
    ROS_INFO("The goal was reached! %s %d",state_msg.c_str(),this->input_mode);
    return state_msg;
}

std::string SingleNavigation::failRobotGoal(std::string robot_state, bool &finish) {
    std::string state_msg;
    //  if(robot_state == "GOING") {

    //  }
    //  else if(robot_state == "SINGLERUN") {

    //  }
    //  else if(robot_state == "EXECUTESEQ") {
    //  }
         
    state_msg = "IDLE";
    this->input_mode = 3;
    return state_msg;
}

std::string SingleNavigation::activeRobotGoal(std::string robot_state, std::string state_req) {
    std::string msg;
    ROS_INFO("Goal active! Now !!");
    if(robot_state == "IDLE" && state_req == "SINGLERUN") {
        msg = "SINGLERUN";
    }
    else if(robot_state == "IDLE" && state_req == "GOING") {
        msg = "GOING";
    }
    else if(robot_state == "IDLE" && state_req == "EXECUTESEQ") {
        msg = "EXECUTESEQ";
    }
    else if(robot_state == "WAITING") {
        msg = "BACKTOBASE";
    }
    // else {
    //     msg = "IDLE";
    // }
    return msg;
}

void SingleNavigation::getFeedbackRobotGoal() {
    ROS_INFO("Getting feedback! How cool is that?");
}

void SingleNavigation::readLiftFile(std::string package_name, std::string filename) {
    std::vector<std::string> tokenized;
    std::string path = ros::package::getPath(package_name)+ filename;
    std::ifstream inFile(path.c_str());
    std::string line;   
    //Prompt to User
    std::cout << "[POINT_READER]IMPORTING Point of interest !" <<std::endl;     
    //First Value is Waypoint Counts 
    getline(inFile,line);
    int waypoint_count = toint(line);
    std::cout << "[POINT_READER]Points Counted : " << waypoint_count <<std::endl;

    // //Read POI Line By Line 
    // while(getline(inFile,line)){
    //  // New Line
    //    std::stringstream strstr(line);
    //    std::string word = "";
    //  // Ignore First Param (Place's Name)
    //    getline(strstr,word,',');
    //    lift_name.push_back(word);
    //  // Gather Params
    //    while(getline(strstr,word,',')){
    //        tokenized.push_back(word);
    //    }
    // }
    //Read POI Line By Line 
    while(getline(inFile,line)){
        // New Line
        std::stringstream strstr(line);
        std::string word = "";
        // Gather Params
        while(getline(strstr,word,',')){
            tokenized.push_back(word);
        }
    }

    //Count All Parameters
    size_t counter = tokenized.size();
    std::cout << "[POINT_READER]Counter  : " << counter <<std::endl; 
    size_t point_amount = (size_t)((counter - waypoint_count) / 6.0);
    std::cout << "[POINT_READER]Div Count : " << point_amount << " ,  File Count = " << waypoint_count <<std::endl;

    std::vector<std::string>::iterator word_it;
    word_it = tokenized.begin();

    // // Create move_base GOAL and Push into vector ! 
    // for(int point_index = 0  ; point_index < counter ; point_index++){
    //    move_base_msgs::MoveBaseGoal newPoint;
    //    newPoint.target_pose.pose.position.x    = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.position.y    = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.orientation.x = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.orientation.y = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.orientation.z = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.orientation.w = std::atof((word_it++)->c_str());
    //    lifts.push_back(newPoint);
    //    if(word_it == tokenized.end())break;
    // }
    // Create move_base GOAL and Push into vector ! 
    for(int point_index = 0  ; point_index < counter ; point_index++){
       MoveBaseGoalData data;
       move_base_msgs::MoveBaseGoal newPoint;
       data.setGoalName((word_it++)->c_str());
       newPoint.target_pose.pose.position.x    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.position.y    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.x = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.y = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.z = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.w = std::atof((word_it++)->c_str());
       data.setGoal(newPoint);
       lifts.push_back(data);
       if(word_it == tokenized.end())break;
    }

    //Prompt End of Process
    std::cout << "[POINT_READER] Point of Interest in imported" << std::endl;
    ROS_INFO("Successfully Load waypoints !");
}

void SingleNavigation::readWaypointFile(std::string package_name, std::string filename) {
    std::vector<std::string> tokenized;
    std::string path = ros::package::getPath(package_name)+ filename;
    std::ifstream inFile(path.c_str());
    std::string line;   
    //Prompt to User
    std::cout << "[POINT_READER]IMPORTING Point of interest !" <<std::endl;     
    //First Value is Waypoint Counts 
    getline(inFile,line);
    int waypoint_count = toint(line);
    std::cout << "[POINT_READER]Points Counted : " << waypoint_count <<std::endl;

    // //Read POI Line By Line 
    // while(getline(inFile,line)){
    //  // New Line
    //    std::stringstream strstr(line);
    //    std::string word = "";
    //  // Ignore First Param (Place's Name)
    //    getline(strstr,word,',');
    //    target_name.push_back(word);
    //  // Gather Params
    //    while(getline(strstr,word,',')){
    //        tokenized.push_back(word);
    //    }
    // }
    //Read POI Line By Line 
    while(getline(inFile,line)){
     // New Line
       std::stringstream strstr(line);
       std::string word = "";
     // Gather Params
       while(getline(strstr,word,',')){
           tokenized.push_back(word);
       }
    }

    //Count All Parameters
    size_t counter = tokenized.size();
    std::cout << "[POINT_READER]Counter  : " << counter <<std::endl; 
    size_t point_amount = (size_t)((counter - waypoint_count) / 6.0);
    std::cout << "[POINT_READER]Div Count : " << point_amount << " ,  File Count = " << waypoint_count <<std::endl;

    std::vector<std::string>::iterator word_it;
    word_it = tokenized.begin();

    // // Create move_base GOAL and Push into vector ! 
    // for(int point_index = 0  ; point_index < counter ; point_index++){
    //    move_base_msgs::MoveBaseGoal newPoint;
    //    newPoint.target_pose.pose.position.x    = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.position.y    = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.orientation.x = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.orientation.y = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.orientation.z = std::atof((word_it++)->c_str());
    //    newPoint.target_pose.pose.orientation.w = std::atof((word_it++)->c_str());
    //    targets.push_back(newPoint);
    //    if(word_it == tokenized.end())break;
    // }
    // Create move_base GOAL and Push into vector ! 
    for(int point_index = 0  ; point_index < counter ; point_index++){
       MoveBaseGoalData data;
       move_base_msgs::MoveBaseGoal newPoint;
       data.setGoalName((word_it++)->c_str());
       newPoint.target_pose.pose.position.x    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.position.y    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.x = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.y = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.z = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.w = std::atof((word_it++)->c_str());
       data.setGoal(newPoint);
       targets.push_back(data);
       if(word_it == tokenized.end())break;
    }

    //Prompt End of Process
    std::cout << "[POINT_READER] Point of Interest in imported" << std::endl;
    ROS_INFO("Successfully Load waypoints !");
}

// Display Waypoints
// void SingleNavigation::displayWaypoints() {
//     for(int i = 0 ; i < target_name.size() ; i++) {
//       std::cout <<"["<<i<<"] " << target_name[i] << " " << targets[i].target_pose.pose.position.x <<std::endl; 
//     }
// }
void SingleNavigation::displayWaypoints() {
    for(int i = 0 ; i < targets.size() ; i++) {
      std::cout <<"["<<i<<"] " << targets[i].getGoalName() << " " <<std::endl; 
    }
}

std::string SingleNavigation::getWaitForDelivery() {
    //ฟังก์ชันสำหรับจัดการพัสดุ รับรหัสพัสดุ บอกตำแหน่งจุดหมายที่จะส่ง 
    ros::ServiceClient client = nh_.serviceClient<icreate_transportation::RunTransportation>("run_transportation");
	icreate_transportation::RunTransportation srv;
    srv.request.request = "req";
    if (client.call(srv))
    {
      std::string res = srv.response.mode;
      return res;
    }
    else
    {
      ROS_ERROR("Failed to call service run_transportation");
      return "";
    }
    // // Wait Time 
  	// int waittime = 10; //seconds
  	// ros::Duration waitingDuration(waittime);
  	// ros::Time startTime = ros::Time::now();
  	// ros::Time thisTime = ros::Time::now();

  	// // Prompt User To Accept Parcel
	// std::cout << "[AGENT] AFTER YOU GOT THE PACKAGE " <<std::endl;
	// std::cout << "[AGENT] PRESS ANYKEY TO ACCEPT PARCEL" << std::endl;
	// char keyin;
	// int flag = -1;// Time Out

  	// //Wait For User Input Within Specific Timeout
  	// while(true){
  	//   thisTime = ros::Time::now();
  	// //   if(thisTime - startTime > waitingDuration)break;
  	//   std::cin >> keyin;
  	//   if((int)keyin != 0){
  	//     flag = 1;
  	//     break;
  	//   }
	// }
}

int SingleNavigation::getWaitForNextPoint(int wait_time) {
    ros::Duration waitingDuration(wait_time);
    ros::Time startTime = ros::Time::now();
    ros::Time thisTime = ros::Time::now();

    // Prompt User To Accept Parcel
    std::cout << "[SEQ WAITFOR] PRESS ANYKEY TO PROCEED TO NEXT POI " <<std::endl;
    std::cout << "[SEQ WAITFOR] Current Pointer is ->" << targetId <<"/" <<SEQUENCE_LENGTH-1 << std::endl;
    char keyin;
    int flag = -1;// Time Out
    std::cout << "[SEQ WAITFOR] WAIT FOR " << wait_time << "SECONDS" <<std::endl;

    //Wait For User Input Within Specific Timeout
    while(true){
        thisTime = ros::Time::now();

        if(thisTime - startTime > waitingDuration)break;
        keyin = getchar();
        if((int)keyin != 0){
            flag = 1;
            break;
        }
    }
    return flag;
}

void SingleNavigation::getUserInput(Robot &robot, std::string base_frame_id, std::string robot_frame_id) {
    int selected_point = -1;
    while(true) {
        this->displayWaypoints();
        this->setNavigationMode(-1);

        // Ask for ID and wait user input
	    std::cout << "[AGENT] Input Target Waypoints ID : " ;
	    std::cin >> selected_point;

        if(selected_point > -1 && selected_point < this->targets.size()){ 
            // Select Mode for Transportation 
		    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
		    std::cout << "[AGENT] Select Mode" <<std::endl;
		    std::cout << "[ 0 ] Go to Specific Point." <<std::endl;
		    std::cout << "[ 1 ] Going and Come Back to This Place." <<std::endl;
		    std::cout << "[ 2 ] Going and Come Back to Base Station." <<std::endl;
            std::cout << "[ 3 ] Going and Wait for Cargo." <<std::endl;
            std::cout << "[ 4 ] Delivery to Target Place." <<std::endl;
            // std::cout << "[ 6 ] Execute The Memorized Sequence" <<std::endl;
            std::cout << "[ 99 ] Exit Sysytem." << std::endl;
		    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
		    std::cout << "Select Mode[0,1,2,3,4,6,99] : " ;
			int input; 
		    std::cin  >> input;
			this->setNavigationMode(input); 
		    std::cout << "You Selected : " << this->getNavigationMode() <<std::endl;
		    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;

            //Remember This Location as startPoint
			robot.setCurrentPosition(base_frame_id, robot_frame_id,"","");
            //Set New Goal
            //Set the endPoint to go
            this->setRobotTarget(selected_point, "Floor");
            robot.setEndPosition(this->target);

            // Do Action depends on the Mode selected
		    switch(this->getNavigationMode()){
		    	//MODE : Go to Specific Point
			    	case 0:
                        //Set the Robot State 
			    		robot.sendStateRequest("SINGLERUN");
                        requestToSetNewGoal = true;
			    	    return;
		    	//MODE : Going and Come back to this place
			    	case 1 :
			    		//Set the Robot State 
                        robot.sendStateRequest("GOING");
			      		requestToSetNewGoal = true;
			    		return;
				//MODE : Going and Come back to base
			    	case 2:
			    		//Set the Robot State 
			      		robot.sendStateRequest("GOING");
                        requestToSetNewGoal = true;
			    		return;
                //MODE : Going and Wait for Cargo.
			    	case 3:
			    		//Set the Robot State 
			      		robot.sendStateRequest("GOING");
                        requestToSetNewGoal = true;
			    		return;
                //MODE : Delivery to Target Place.
			    	case 4:
			    		//Set the Robot State 
                        robot.sendStateRequest("SENDSUPPLIES");
			      		requestToSetNewGoal = true;
			    		return;
			  	//MODE : Execute the Sequence
					// case 6:
					// 	//Set the Robot State 
					// 	robot.state_req_msg.data = "EXECUTESEQ";
			    	// 	//Set Target to Next Sequence 
					// 	this->setRobotTarget(sequence[targetId]);
                    //     MoveBaseGoalData data(this->getRobotTarget(), target_name[selected_point]);
                    //     robot.setEndPosition(data);
			    	// 	//Set the new Goal
			    	// 	requestToSetNewGoal = true;
			      	// 	return;
			    //MODE : EXIT
					case 99:
						//Set the Robot State 
			      		robot.sendStateRequest("IDLE");
						requestToSetNewGoal = false;
			      		// finish = true;
			      		return;
			    //Not Specified : Do Nothing ^_^
		      	default:
		      		std::cout << "[AGENT]You Selection was Wrong Input" <<std::endl;
                    std::cout << "Please Try Again" << std::endl;
					requestToSetNewGoal = false;
					return;
		    }//Mode Select
        }   
    }
}

void SingleNavigation::getNextStep(Robot &robot, std::string base_frame_id, std::string robot_frame_id) {
    ROS_INFO("Input Mode: %d",this->input_mode);
	switch(input_mode) {
		case inputUser :
        {
            std::cout << "[AGENT] REACH THE BASE , YAY ! " <<std::endl;
			this->getUserInput(robot, base_frame_id, robot_frame_id);
			break;
        }
		case waitParcel :
        {
            // WAIT FOR PARCEL DELIVERY
          	std::cout << "[AGENT] WAIT FOR ACCEPTANCE !" <<std::endl <<std::endl;
          	std::string res_mode = getWaitForDelivery();
            if(res_mode == "sending"){
                
            }
            else if(res_mode == "receiving") {

            }
            else if(res_mode == "cancel") {

            }
            // After Delivering = Set Back to the First place
			
			if(this->getNavigationMode() == 1) {
                robot.sendStateRequest("GOING");
				this->setRobotTarget(robot.startPosition);
                robot.setCurrentPosition(base_frame_id, robot_frame_id,"", "");
			    robot.setEndPosition(this->target);
          	    std::cout << "[AGENT] GOING BACK TO : ";
          	    std::cout << robot.startPosition.goal.target_pose.pose.position.x <<","<<robot.startPosition.goal.target_pose.pose.position.y  <<std::endl;
                requestToSetNewGoal = true;
			}
			else if(this->getNavigationMode() == 2) {
                robot.sendStateRequest("BACKTOBASE");
				this->setRobotTarget(0, "Floor");
                robot.setCurrentPosition(base_frame_id, robot_frame_id, "", "");
			    robot.setEndPosition(this->target);
          	    std::cout << "[AGENT] GOING BACK TO : ";
          	    std::cout << robot.startPosition.goal.target_pose.pose.position.x <<","<<robot.startPosition.goal.target_pose.pose.position.y  <<std::endl;
                requestToSetNewGoal = true;
			}
            else if(navigation_mode == 3) {
                this->getUserInput(robot, base_frame_id, robot_frame_id);
            }
            else if(navigation_mode == 4) {
                this->getUserInput(robot, base_frame_id, robot_frame_id);
            }
			break;
        }
		case waitQueue :
        {
            std::cout << "[AGENT] REACH THE TARGET" <<std::endl;
    		std::cout << "[AGENT] EXECUTESEQ NEXT SEQUENCE : "<< targetId <<std::endl;
			this->getWaitForNextPoint(3);
			targetId++;
			if(targetId >= SEQUENCE_LENGTH){
				// finish = true;
				break;
			}
			// Set The Next Sequence
			robot.setCurrentPosition(base_frame_id, robot_frame_id, "", "");
			this->setRobotTarget(sequence[targetId], "Floor");
			robot.setEndPosition(target);
			requestToSetNewGoal = true;
            robot.sendStateRequest("EXECUTESEQ");
			break;
        }
		default:
			break;
	}
}

void SingleNavigation::setNavigationMode(int mode) {
    this->navigation_mode = mode;
}

int SingleNavigation::getNavigationMode() {
    return this->navigation_mode;
}

void SingleNavigation::setTimer(int duration) {
    ROS_INFO("Loop Create Timer");
	requestToCreateTimer = false;
    timer = nh_.createTimer(ros::Duration(duration), &SingleNavigation::timerCallback, this);
}

void SingleNavigation::timerCallback(const ros::TimerEvent &event) {
    ROS_INFO("[TimerCallback] clear costmap");
	//Clear Costmap
  	client.call(clearer);
}

// Convert String To Int
int SingleNavigation::toint(std::string s) { //The conversion function 
    return atoi(s.c_str());
}

}