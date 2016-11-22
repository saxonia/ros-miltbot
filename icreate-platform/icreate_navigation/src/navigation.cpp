#include "navigation.h"

namespace icreate {

Navigation::Navigation(): sequence(24){
    // this->ac("move_base", true);
    navigation_mode = -1;
    client = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
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

Navigation::~Navigation() {

}

void Navigation::setRobotTarget(move_base_msgs::MoveBaseGoal goal) {
    targets.push_back(goal);
    target = targets.end() - 1;
}

void Navigation::setRobotTarget(int selected_point) {
    Navigation::selected_point = selected_point;
    target = targets.begin() + selected_point;
    ROS_INFO("set Robot Target: %s", this->target_name[selected_point].c_str());
}

move_base_msgs::MoveBaseGoal Navigation::getRobotTarget() {
    ROS_INFO("get Robot Target: %s",this->target_name[selected_point].c_str());
    return targets[selected_point];
}

void Navigation::setRobotGoal(std::string frame_id) {
    //frame_id = "/map"
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x    = target->target_pose.pose.position.x;
    goal.target_pose.pose.position.y    = target->target_pose.pose.position.y;
    goal.target_pose.pose.orientation.x = target->target_pose.pose.orientation.x;
    goal.target_pose.pose.orientation.y = target->target_pose.pose.orientation.y;
    goal.target_pose.pose.orientation.z = target->target_pose.pose.orientation.z;
    goal.target_pose.pose.orientation.w = target->target_pose.pose.orientation.w;
    std::cout << "[AGENT] SET NEW GOAL ! " << std::endl;

}

move_base_msgs::MoveBaseGoal Navigation::getRobotGoal() {
    return goal;
}

std::string Navigation::doneRobotGoal(std::string robot_state) {
    client.call(clearer);
    std::string state_msg; 
    if(robot_state == "GOING") {
        state_msg = "WAITING";
        this->input_mode = 1;
    }
    else if(robot_state == "WAITING") {
        this->input_mode = 1;
    }
    else if(robot_state == "BACKTOBASE") {
        state_msg = "IDLE";
        this->input_mode = 0;
    }
    else if(robot_state == "SINGLERUN") {
        state_msg = "IDLE";
        this->input_mode = 0;
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

std::string Navigation::failRobotGoal(std::string robot_state, bool &finish) {
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

std::string Navigation::activeRobotGoal(std::string robot_state, std::string state_req) {
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

void Navigation::getFeedbackRobotGoal() {
    ROS_INFO("Getting feedback! How cool is that?");
}

void Navigation::readWaypointConstant() {
    move_base_msgs::MoveBaseGoal newPoint;
    newPoint.target_pose.pose.position.x    = 7.687;
    newPoint.target_pose.pose.position.y    = 14.260;
    newPoint.target_pose.pose.orientation.x = 0.000;
    newPoint.target_pose.pose.orientation.y = 0.000;
    newPoint.target_pose.pose.orientation.z = 0.552;
    newPoint.target_pose.pose.orientation.w = 0.834;
    targets.push_back(newPoint);

    target = targets.begin();    
}

void Navigation::readLiftFile(std::string filename) {
    std::vector<std::string> tokenized;
    std::string path = ros::package::getPath("icreate_navigation")+ filename;
    std::ifstream inFile(path.c_str());
    std::string line;   
    //Prompt to User
    std::cout << "[POINT_READER]IMPORTING Point of interest !" <<std::endl;     
    //First Value is Waypoint Counts 
    getline(inFile,line);
    int waypoint_count = toint(line);
    std::cout << "[POINT_READER]Points Counted : " << waypoint_count <<std::endl;

    //Read POI Line By Line 
    while(getline(inFile,line)){
     // New Line
       std::stringstream strstr(line);
       std::string word = "";
     // Ignore First Param (Place's Name)
       getline(strstr,word,',');
       lift_name.push_back(word);
     // Gather Params
       while(getline(strstr,word,',')){
           tokenized.push_back(word);
       }
    }

    //Count All Parameters
    size_t counter = tokenized.size();
    std::cout << "[POINT_READER]Counter  : " << counter <<std::endl; 
    size_t point_amount = (size_t)(counter / 6.0);
    std::cout << "[POINT_READER]Div Count : " << point_amount << " ,  File Count = " << waypoint_count <<std::endl;

    std::vector<std::string>::iterator word_it;
    word_it = tokenized.begin();

    // Create move_base GOAL and Push into vector ! 
    for(int point_index = 0  ; point_index < counter ; point_index++){
       move_base_msgs::MoveBaseGoal newPoint;
       newPoint.target_pose.pose.position.x    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.position.y    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.x = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.y = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.z = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.w = std::atof((word_it++)->c_str());
       lifts.push_back(newPoint);
       if(word_it == tokenized.end())break;
    }

    //Prompt End of Process
    std::cout << "[POINT_READER] Point of Interest in imported" << std::endl;
    ROS_INFO("Successfully Load waypoints !");
}

void Navigation::readWaypointFile(std::string filename) {
    std::vector<std::string> tokenized;
    std::string path = ros::package::getPath("icreate_navigation")+ filename;
    std::ifstream inFile(path.c_str());
    std::string line;   
    //Prompt to User
    std::cout << "[POINT_READER]IMPORTING Point of interest !" <<std::endl;     
    //First Value is Waypoint Counts 
    getline(inFile,line);
    int waypoint_count = toint(line);
    std::cout << "[POINT_READER]Points Counted : " << waypoint_count <<std::endl;

    //Read POI Line By Line 
    while(getline(inFile,line)){
     // New Line
       std::stringstream strstr(line);
       std::string word = "";
     // Ignore First Param (Place's Name)
       getline(strstr,word,',');
       target_name.push_back(word);
     // Gather Params
       while(getline(strstr,word,',')){
           tokenized.push_back(word);
       }
    }

    //Count All Parameters
    size_t counter = tokenized.size();
    std::cout << "[POINT_READER]Counter  : " << counter <<std::endl; 
    size_t point_amount = (size_t)(counter / 6.0);
    std::cout << "[POINT_READER]Div Count : " << point_amount << " ,  File Count = " << waypoint_count <<std::endl;

    std::vector<std::string>::iterator word_it;
    word_it = tokenized.begin();

    // Create move_base GOAL and Push into vector ! 
    for(int point_index = 0  ; point_index < counter ; point_index++){
       move_base_msgs::MoveBaseGoal newPoint;
       newPoint.target_pose.pose.position.x    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.position.y    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.x = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.y = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.z = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.w = std::atof((word_it++)->c_str());
       targets.push_back(newPoint);
       if(word_it == tokenized.end())break;
    }

    //Prompt End of Process
    std::cout << "[POINT_READER] Point of Interest in imported" << std::endl;
    ROS_INFO("Successfully Load waypoints !");
}

// Display Waypoints
void Navigation::displayWaypoints() {
    for(int i = 0 ; i < target_name.size() ; i++) {
      std::cout <<"["<<i<<"] " << target_name[i] << " " << targets[i].target_pose.pose.position.x <<std::endl; 
    }
}

void Navigation::getWaitForDelivery() {
    //ฟังก์ชันสำหรับจัดการพัสดุ รับรหัสพัสดุ บอกตำแหน่งจุดหมายที่จะส่ง 

	// Wait Time 
  	int waittime = 10; //seconds
  	ros::Duration waitingDuration(waittime);
  	ros::Time startTime = ros::Time::now();
  	ros::Time thisTime = ros::Time::now();

  	// Prompt User To Accept Parcel
	std::cout << "[AGENT] AFTER YOU GOT THE PACKAGE " <<std::endl;
	std::cout << "[AGENT] PRESS ANYKEY TO ACCEPT PARCEL" << std::endl;
	char keyin;
	int flag = -1;// Time Out

  	//Wait For User Input Within Specific Timeout
  	while(true){
  	  thisTime = ros::Time::now();
  	//   if(thisTime - startTime > waitingDuration)break;
  	  std::cin >> keyin;
  	  if((int)keyin != 0){
  	    flag = 1;
  	    break;
  	  }
	}
}

int Navigation::getWaitForNextPoint(int wait_time) {
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

void Navigation::getUserInput(Robot &robot) {
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
		    std::cout << "[ 1 ] Delivery and Come Back to This Place." <<std::endl;
		    std::cout << "[ 2 ] Delivery and Come Back to Base Station" <<std::endl;
			std::cout << "[ 3 ] Execute The Memorized Sequence" <<std::endl;
		    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
		    std::cout << "Select Mode[0,1,2] : " ;
			int input; 
		    std::cin  >> input;
			this->setNavigationMode(input); 
		    std::cout << "You Selected : " << this->getNavigationMode() <<std::endl;
		    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;

            // Do Action depends on the Mode selected
		    switch(this->getNavigationMode()){
		    	//MODE : Go to Specific Point
			    	case 0:
                        //Set the Robot State 
                        robot.state_req_msg.data = "SINGLERUN";
			    		//Set New Goal
			    		this->setRobotTarget(selected_point);
						robot.setEndPosition(this->getRobotTarget());
			    		requestToSetNewGoal = true;
						robot.requestToSendStateReq = true;
						// finish = false;
			    	    return;
		    	//MODE : Delivery and Come back to this place
			    	case 1 :
			    		//Remember This Location as startPoint
			    		robot.setCurrentPosition();
			    		//Set the Robot State 
                        robot.state_req_msg.data = "GOING";
			    		//Set the endPoint to go
			      		this->setRobotTarget(selected_point);
						robot.setEndPosition(this->getRobotTarget());
			      		requestToSetNewGoal = true;
						robot.requestToSendStateReq = true;
			    		return;
				//MODE : Delivery and Come back to base
			    	case 2:
			    		//Remember This Location as startPoint
			    		robot.setCurrentPosition();
			    		//Set the Robot State 
                        robot.state_req_msg.data = "GOING";
			    		//Set the endPoint to go
			      		this->setRobotTarget(selected_point);
						robot.setEndPosition(this->getRobotTarget());
			      		requestToSetNewGoal = true;
						robot.requestToSendStateReq = true;
			    		return;
			  	//MODE : Execute the Sequence
					case 3:
						//Set the Robot State 
						robot.state_req_msg.data = "EXECUTESEQ";
			    		//Set Target to Next Sequence 
						this->setRobotTarget(sequence[targetId]);
						robot.setEndPosition(this->getRobotTarget());
			    		//Set the new Goal
			    		requestToSetNewGoal = true;
						robot.requestToSendStateReq = true;
			      		return;
			    //MODE : EXIT
					case 99:
						//Set the Robot State 
			      		robot.state_req_msg.data = "IDLE";
						requestToSetNewGoal = false;
						robot.requestToSendStateReq = false;
			      		// finish = true;
			      		return;
			    //Not Specified : Do Nothing ^_^
		      	default:
		      		robot.state_req_msg.data = "IDLE";
		      		std::cout << "[AGENT]You Selected NOTHING" <<std::endl;
		      		this->setNavigationMode(-1);
					requestToSetNewGoal = false;
					robot.requestToSendStateReq = false;
					return;
		    }//Mode Select
        }   
    }
}

void Navigation::getNextStep(Robot &robot) {
    ROS_INFO("Input Mode: %d",this->input_mode);
	switch(input_mode) {
		case inputUser :
			std::cout << "[AGENT] REACH THE BASE , YAY ! " <<std::endl;
			this->getUserInput(robot);
			break;
		case waitParcel :
			// WAIT FOR PARCEL DELIVERY
          	std::cout << "[AGENT] WAIT FOR ACCEPTANCE !" <<std::endl <<std::endl;
          	this->getWaitForDelivery();

			// After Delivering = Set Back to the First place
			robot.state_req_msg.data = "BACKTOBASE";
			if(this->getNavigationMode() == 1) {
				this->setRobotTarget(robot.startPosition);
			}
			else if(this->getNavigationMode() == 2) {
				this->setRobotTarget(0);
			}
			
			robot.setCurrentPosition();
			robot.setEndPosition(this->getRobotTarget());
          	std::cout << "[AGENT] GOING BACK TO : ";
          	std::cout << robot.startPosition.target_pose.pose.position.x <<","<<robot.startPosition.target_pose.pose.position.y  <<std::endl;
          	requestToSetNewGoal = true;
			break;

		case waitQueue :
			// break;
			std::cout <<std::endl <<std::endl;
    		std::cout << "[AGENT] REACH THE TARGET" <<std::endl;
    		std::cout << "[AGENT] EXECUTESEQ NEXT SEQUENCE : "<< targetId <<std::endl;
			this->getWaitForNextPoint(3);
			targetId++;
			if(targetId >= SEQUENCE_LENGTH){
				// finish = true;
				break;
			}
			// Set The Next Sequence
			robot.setCurrentPosition();
			this->setRobotTarget(sequence[targetId]);
			robot.setEndPosition(this->getRobotTarget());
			robot.state_req_msg.data = "EXECUTESEQ";
			requestToSetNewGoal = true;
			robot.requestToSendStateReq = true;
			break;
		default:
			break;
	}
}

void Navigation::setNavigationMode(int mode) {
    navigation_mode = mode;
}

int Navigation::getNavigationMode() {
    return navigation_mode;
}

void Navigation::setTimer(int duration) {
    ROS_INFO("Loop Create Timer");
	requestToCreateTimer = false;
    timer = nh_.createTimer(ros::Duration(duration), &Navigation::timerCallback, this);
}

void Navigation::timerCallback(const ros::TimerEvent &event) {
    ROS_INFO("[TimerCallback] clear costmap");
	//Clear Costmap
  	client.call(clearer);
}

// Convert String To Int
int Navigation::toint(std::string s) { //The conversion function 
    return atoi(s.c_str());
}

}