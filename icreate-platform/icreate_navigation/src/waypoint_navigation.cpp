#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include "robot.h"
#include "navigation.h"

//Class
icreate::Robot robot;
icreate::Navigation navigation;

//Publisher 
ros::Publisher state_req_pub;
ros::Subscriber state_sub;
std_msgs::String state_req_msg;

//Client Service of move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum inputMode {
	inputUser = 0,
	waitParcel = 1,
	waitQueue = 2,
	failGoal = 3
	
};

int input_mode = 0;

//Timer
ros::Timer timer;
bool requestToCreateTimer;

//Costmap Clearing Service Client (/move_base_node/clear_costmaps)
ros::ServiceClient client;
std_srvs::Empty clearer;

//Sending New Goal Request
bool requestToSetNewGoal; //sendNewGoal;

bool requestToSendStateReq;

bool isDoneGoal;

//Finish Flag
bool finish;

//State Machine Enumeration
std::string robot_state;
std::string state_req;

//Sequence for execution
int targetId;
//int sequence[13] = {5,1,2,5,1,2,5,1,2,5,1,2,5};
int SEQUENCE_LENGTH = 4;
int sequence[24] ={0,3,5,0};

//// System Function

void userInput(){
    int selected_point = -1;
    while(true) {
        // Display Waypoint 
        navigation.displayWaypoints();
		navigation.setNavigationMode(-1);

        // Ask for ID and wait user input
	    std::cout << "[AGENT] Input Target Waypoints ID : " ;
	    std::cin >> selected_point;

        if(selected_point > -1 && selected_point < navigation.targets.size()){ 
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
			navigation.setNavigationMode(input); 
		    std::cout << "You Selected : " << navigation.getNavigationMode() <<std::endl;
		    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;

            // Do Action depends on the Mode selected
		    switch(navigation.getNavigationMode()){
		    	//MODE : Go to Specific Point
			    	case 0:
                        //Set the Robot State 
                        state_req = "SINGLERUN";
			    		//Set New Goal
			    		navigation.setRobotTarget(selected_point);
						robot.setEndPosition(navigation.getRobotTarget());
			    		requestToSetNewGoal = true;
						requestToSendStateReq = true;
						finish = false;
			    	    return;
		    	//MODE : Delivery and Come back to this place
			    	case 1 :
			    		//Remember This Location as startPoint
			    		robot.setCurrentPosition();
			    		//Set the Robot State 
                        state_req = "GOING";
			    		//Set the endPoint to go
			      		navigation.setRobotTarget(selected_point);
						robot.setEndPosition(navigation.getRobotTarget());
			      		requestToSetNewGoal = true;
						requestToSendStateReq = true;
			    		return;
				//MODE : Delivery and Come back to base
			    	case 2:
			    		//Remember This Location as startPoint
			    		robot.setCurrentPosition();
			    		//Set the Robot State 
                        state_req = "GOING";
			    		//Set the endPoint to go
			      		navigation.setRobotTarget(selected_point);
						robot.setEndPosition(navigation.getRobotTarget());
			      		requestToSetNewGoal = true;
						requestToSendStateReq = true;
			    		return;
			  	//MODE : Execute the Sequence
					case 3:
						//Set the Robot State 
						state_req = "EXECUTESEQ";
			    		//Set Target to Next Sequence 
						navigation.setRobotTarget(sequence[targetId]);
						robot.setEndPosition(navigation.getRobotTarget());
			    		//Set the new Goal
			    		requestToSetNewGoal = true;
						requestToSendStateReq = true;
			      		return;
			    //MODE : EXIT
					case 99:
						//Set the Robot State 
			      		state_req = "IDLE";
						requestToSetNewGoal = false;
						requestToSendStateReq = false;
			      		finish = true;
			      		return;
			    //Not Specified : Do Nothing ^_^
		      	default:
		      		state_req = "IDLE";
		      		std::cout << "[AGENT]You Selected NOTHING" <<std::endl;
		      		navigation.setNavigationMode(-1);
					requestToSetNewGoal = false;
					requestToSendStateReq = false;
					return;
		    }//Mode Select                        
        }
    }

}

void waitfordelivery() {
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
	// return flag;
}

// Just Wait
int waitfor(int waittime){

  ros::Duration waitingDuration(waittime);
  ros::Time startTime = ros::Time::now();
  ros::Time thisTime = ros::Time::now();

  // Prompt User To Accept Parcel
	  std::cout << "[SEQ WAITFOR] PRESS ANYKEY TO PROCEED TO NEXT POI " <<std::endl;
	  std::cout << "[SEQ WAITFOR] Current Pointer is ->" << targetId <<"/" <<SEQUENCE_LENGTH-1 << std::endl;
	  char keyin;
	  int flag = -1;// Time Out

	std::cout << "[SEQ WAITFOR] WAIT FOR " << waittime << "SECONDS" <<std::endl;

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

void nextStep() {
	ROS_INFO("Input Mode: %d",input_mode);
	switch(input_mode) {
		case inputUser :
			std::cout << "[AGENT] REACH THE BASE , YAY ! " <<std::endl;
			userInput();
			break;
		case waitParcel :
			// WAIT FOR PARCEL DELIVERY
          	std::cout << "[AGENT] WAIT FOR ACCEPTANCE !" <<std::endl <<std::endl;
          	waitfordelivery();

			// After Delivering = Set Back to the First place
			state_req = "BACKTOBASE";
			if(navigation.getNavigationMode() == 1) {
				navigation.setRobotTarget(robot.startPosition);
			}
			else if(navigation.getNavigationMode() == 2) {
				navigation.setRobotTarget(0);
			}
			
			robot.setCurrentPosition();
			robot.setEndPosition(navigation.getRobotTarget());
          	std::cout << "[AGENT] GOING BACK TO : ";
          	std::cout << robot.startPosition.target_pose.pose.position.x <<","<<robot.startPosition.target_pose.pose.position.y  <<std::endl;
          	requestToSetNewGoal = true;
			break;

		case waitQueue :
			// break;
			std::cout <<std::endl <<std::endl;
    		std::cout << "[AGENT] REACH THE TARGET" <<std::endl;
    		std::cout << "[AGENT] EXECUTESEQ NEXT SEQUENCE : "<< targetId <<std::endl;
			waitfor(3);
			targetId++;
			if(targetId >= SEQUENCE_LENGTH){
				finish = true;
				break;
			}
			// Set The Next Sequence
			robot.setCurrentPosition();
			navigation.setRobotTarget(sequence[targetId]);
			robot.setEndPosition(navigation.getRobotTarget());
			state_req = "EXECUTESEQ";
			requestToSetNewGoal = true;
			requestToSendStateReq = true;
			break;
		default:
			break;
	}
}

bool waitMoveBaseServer(MoveBaseClient &ac) {
	int tries = 0;
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up %d",tries+1);
      tries++;
      if(tries == 3){
        ROS_INFO("Failed to Start Waypoint Node");
        return false;
      }
    }
    ROS_INFO("Navigation Waypoint Node Initialized !");
	return true;
}


/// CALLBACKS Function

// STATE CALLBACK
void stateCallback(const std_msgs::String::ConstPtr& msg) {
    robot_state = msg->data;
	ROS_INFO("Robot state: %s",robot_state.c_str());
}

// TIMER CALLBACK
void timerCallback(const ros::TimerEvent &event){
	ROS_INFO("[TimerCallback] clear costmap");
	//Clear Costmap
  	client.call(clearer);
  	//This Timer is finish , request to create it again;
    // requestToCreateTimer = true;
}

void goalDoneCallback_state(const actionlib::SimpleClientGoalState &state, 
  const move_base_msgs::MoveBaseResultConstPtr &result){
    ROS_INFO("Goal Done Now !!!!!");

    //Navigation completed
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
    	//Clear Costmap
		ROS_INFO("SUCCEEDED");
  		client.call(clearer);
        state_req_msg.data = navigation.doneRobotGoal(robot_state, input_mode);
    }

	if(state.state_ == actionlib::SimpleClientGoalState::REJECTED) {
		ROS_INFO("REJECTED");
	}

	if(state.state_ == actionlib::SimpleClientGoalState::LOST) {
		ROS_INFO("LOST");
	}

    //Navigation failed
    if(state.state_ == actionlib::SimpleClientGoalState::ABORTED){
      	ROS_WARN("Failed to reach the goal...");
		state_req_msg.data = navigation.failRobotGoal(robot_state, finish);
		input_mode = 3;
    }

    std::cout << "Goal Finished Or Cancelled by Joy" << std::endl;
	requestToSendStateReq = true;
	isDoneGoal = true;
}

void goalActiveCallback(){
    state_req_msg.data = navigation.activeRobotGoal(robot_state, state_req);
	requestToSendStateReq = true;
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
    // navigation.getFeedbackRobotGoal();
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "waypoint_navigation");

    MoveBaseClient ac("/icreate/move_base", true);

	// Callback polling Rate 
    ros::Rate r(30);

    navigation.readWaypointFile("/waypoint/build4_f20.csv");

    if(!robot.setCurrentPosition()) {
        return -1;
    }
	// robot.setCurrentPosition();

    // Wait for the action server to come up
    if(!waitMoveBaseServer(ac))
		return -1;

    // Subscriber to Get Current position 
    ros::NodeHandle nh;

    // Subsribe Robot State Topic
    state_sub = nh.subscribe("/icreate/state",100,stateCallback);
    state_req_pub = nh.advertise<std_msgs::String>("/icreate/state_req",10);

    // Subscribe to Map Clearing Service 
    client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	targetId = 0;

	// Point The iterator to the beginning of the sequence
    navigation.target = navigation.targets.begin();
	
	// Request To Create timer
    requestToCreateTimer = true;

	requestToSendStateReq = false;
	isDoneGoal = false;
    requestToSetNewGoal = false;

    // Ask User For Input
    userInput();

	

	// Start the Navigation Waypoint Loop
	while(ros::ok() && !finish) {
		// ROS_INFO("IN LOOP");

		if(requestToSendStateReq) {
			ROS_INFO("Loop Send State Request");
			requestToSendStateReq = false;
			state_req_pub.publish(state_req_msg);
		}
		// if(robot.requestToSendStateReq) {
		// 	robot.sendStateRequest();
		// }

		if(isDoneGoal) {
			ROS_INFO("Loop Done Goal");
			isDoneGoal = false;
			nextStep();
		}

		if(requestToSetNewGoal) {
			ROS_INFO("Loop Set New Goal");
			requestToSetNewGoal = false;

			navigation.setRobotGoal("/map");
			ac.sendGoal(navigation.goal, 
                      boost::bind(&goalDoneCallback_state, _1, _2), 
                      boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
		}

		if(requestToCreateTimer) {
			// ROS_INFO("Loop Create Timer");
			requestToCreateTimer = false;
			timer = nh.createTimer(ros::Duration(10), timerCallback);
		}
		// if(navigation.requestToCreateTimer) {
		// 	navigation.setTimer(10);
		// }

		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Exiting Waypoint Navigation");
    return 0;
}