#include <ros/ros.h>
#include <ros/package.h>
// #include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>

// #include <iostream>
// #include <sstream>
#include <fstream>
// #include <cstdlib>
// #include <termios.h>
// #include <ctime>


//Client Service of move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Timer
ros::Timer timer;
bool requestToCreateTimer;

//Costmap Clearing Service Client (/move_base_node/clear_costmaps)
ros::ServiceClient client;
std_srvs::Empty clearer;

//Sending New Goal Request
bool requestToSetNewGoal; //sendNewGoal;

//Finish Flag
bool finish;

//Location Marking Request
bool requestToMarkLocation;

//Waypoints 
std::vector<move_base_msgs::MoveBaseGoal> targets;
std::vector<move_base_msgs::MoveBaseGoal>::iterator target;
std::vector<std::string> target_name;

//Goal 
move_base_msgs::MoveBaseGoal startPoint;
move_base_msgs::MoveBaseGoal  endPoint;
move_base_msgs::MoveBaseGoal  currentPosition;

//State Machine Enumeration
  class robotState{
    public:
      enum state{
      IDLE = 0,
      GOING = 1,
      WAITING = 2,
      BACKTOBASE = 3,
      SINGLERUN = 4,
      EXECUTESEQ = 5
    };
  };
  int robot_state;
  int selected_point = -1; //choice

//Sequence for execution
int targetId;
//int sequence[13] = {5,1,2,5,1,2,5,1,2,5,1,2,5};
int SEQUENCE_LENGTH = 24;
int sequence[24] ={0,1,6,7,8,9,12,9,8,7,6,2,3,5,4,3,10,11,10,3,2,1,13,0};

//Log write
int startIndex, endIndex;
double startTime;

// Convert String To Int
int toint(std::string s) //The conversion function
{
    return atoi(s.c_str());
}

// Display Waypoints
void displayWaypoints(){
  for(int i = 0 ; i < target_name.size() ; i++){
      std::cout <<"["<<i<<"] " << target_name[i] <<std::endl; 
    }
}

void generateLog(){
    std::fstream fs;
    // std::fstream fs2;
    fs.open ("tutorLog1.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    // fs2.open ("tutorLog2.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    ros::Time noww = ros::Time::now();
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    fs << "SEPERATE=======" << (now->tm_year + 1900) << '-' 
        << (now->tm_mon + 1) << '-'
        <<  now->tm_mday << "============" << noww
        << std::endl;
    // fs2 << "SEPERATE" << (now->tm_year + 1900) << '-' 
    //      << (now->tm_mon + 1) << '-'
    //      <<  now->tm_mday << "============" <<noww
    //      << std::endl;      
    fs.close();
    // fs2.close();
}

void writeFile(int start_id , int end_id , double timeusing){
  std::fstream fs;
  std::fstream fs2;
  fs.open ("tutorLog1.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  fs2.open ("tutorLog2.txt", std::fstream::in | std::fstream::out | std::fstream::app);

  fs << start_id << "," << end_id << "," << timeusing <<std::endl ;
  fs2 << target_name[start_id] << "->" << target_name[end_id] << std::endl <<" = " << timeusing <<std::endl; 

  fs.close();
  fs2.close();
}

void markCurrentLocation(){

    tf::TransformListener listener;  
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    try {
        listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    startPoint.target_pose.pose.position.x    = transform.getOrigin().x();
    startPoint.target_pose.pose.position.y    = transform.getOrigin().y();
    startPoint.target_pose.pose.orientation.x = transform.getRotation().x();
    startPoint.target_pose.pose.orientation.y = transform.getRotation().y();
    startPoint.target_pose.pose.orientation.z = transform.getRotation().z();
    startPoint.target_pose.pose.orientation.w = transform.getRotation().w();

  std::cout << "RETURNING POINT MARKED : ";
  std::cout << startPoint.target_pose.pose.position.x <<","<<startPoint.target_pose.pose.position.y  <<std::endl;
}

// Wait FOR USER ACCEPTANCE !
int waitfordelivery(){
  // Wait Time 
  int waittime = 5; //seconds
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
    if(thisTime - startTime > waitingDuration)break;
    keyin = getchar();
    if((int)keyin != 0){
      flag = 1;
      break;
    }
  }
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

// 
void userInput(){
  selected_point = -1; 
   // Loop for Waiting For User Input
	  while(true){
	    // Display Waypoint 
	      displayWaypoints();

	    // Ask for ID and wait user input
	      std::cout << "[AGENT] Input Target Waypoints ID : " ;
	      std::cin >> selected_point;
	    // Check the integrity of SelectedPoint
	    if(selected_point != -1 && selected_point < targets.size()){

		    // Select Mode for Transportation 
		      int selectedMode = -1;
		      std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
		      std::cout << "[AGENT] Select Mode" <<std::endl;
		      std::cout << "[ 0 ] Go to Specific Point." <<std::endl;
		      std::cout << "[ 1 ] Delivery and Come Back to This Place." <<std::endl;
		      std::cout << "[ 2 ] Execute The Memorized Sequence" <<std::endl;
		      std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
		      std::cout << "Select Mode[0,1,2] : " ; 
		      std::cin 	>> selectedMode; 
		      std::cout << "You Selected : " << selectedMode <<std::endl;
		      std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
		      
		      // Do Action depends on the Mode selected
		      switch(selectedMode){
		      	//MODE : Go to Specific Point
			      	case 0:
			      		std::cout << "[AGENT]You Selected : " << selectedMode <<std::endl;
			      		//Set the Robot State 
			      			robot_state = robotState::SINGLERUN;
			      		//Set New Goal
			      			*target = targets[selected_point];
			      			requestToSetNewGoal = true;
			      		return;
			      	break;

		      	//MODE : Delivery and Come back
			      	case 1:
			      		std::cout << "[AGENT]You Selected : " << selectedMode <<std::endl;
			      		//Remember This Location as startPoint
			      			markCurrentLocation();

			      		//Set the Robot State 
			      			robot_state = robotState::GOING;

			      		//We are on the "startPoint"
			      		//Set the endPoint to go
				      		endPoint = targets[selected_point];
				      		*target = endPoint;
				      		requestToSetNewGoal = true;
				      	return;
			      	break;

			    //MODE : Execute the Sequence
					// case 2:
					// std::cout << "[AGENT]You Selected : " << selectedMode <<std::endl;
					// 	//Set the Robot State 
			    //   			robot_state = robotState::EXECUTESEQ;
			    //   		//Set Target to Next Sequence 
			    //   			*target = targets[sequence[targetId]];
			    //   		//Set the new Goal
			    //   			requestToSetNewGoal = true;
			    //   		//[Logger] Set The StartPoint
			    //   			startIndex = sequence[targetId];
			    //   		return;
			    //   	break;
			    //MODE : EXIT
					case 99:
						//Set the Robot State 
			      			robot_state = robotState::IDLE;
			      			requestToSetNewGoal = false;
			      		finish = true;
			      		return;
			      	break;

			    //Not Specified : Do Nothing ^_^
		      	default:
		      		robot_state = robotState::IDLE;
		      		std::cout << "[AGENT]You Selected NOTHING" <<std::endl;
		      		selectedMode = -1;
		      }//Mode Select
	  	}//selectedPoi integrity check 

	}
}

// Reading Point Of Interest
void read_waypoint_file(std::string filename){

    std::vector<std::string> tokenized;
    std::string path = ros::package::getPath("simple_navigation_goals")+ filename;
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
}

void read_waypoint_constant()
{
    move_base_msgs::MoveBaseGoal newPoint;
    newPoint.target_pose.pose.position.x    = -6.326;
    newPoint.target_pose.pose.position.y    = -0.674;
    newPoint.target_pose.pose.orientation.x = 0.000;
    newPoint.target_pose.pose.orientation.y = 0.000;
    newPoint.target_pose.pose.orientation.z = -2.870;
    newPoint.target_pose.pose.orientation.w = 0.958;
    targets.push_back(newPoint);
}

/// CALLBACKS Function

// TIMER CALLBACK
void timerCallback(const ros::TimerEvent &event){

	//Clear Costmap
  		client.call(clearer);
  	//This Timer is finish , request to create it again;
    	requestToCreateTimer = true;
    return;
}

void goalDoneCallback_state(const actionlib::SimpleClientGoalState &state, 
  const move_base_msgs::MoveBaseResultConstPtr &result){

    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
    	//Clear Costmap
  			client.call(clearer);
        ROS_INFO("The goal was reached!");
    }

    if(state.state_ == actionlib::SimpleClientGoalState::ABORTED)
        ROS_WARN("Failed to reach the goal...");
    
    std::cout << "Goal Finished Or Cancelled by Joy ::=>>>Ask For New Goal" << std::endl;
    
    // If It's just arrived
    if(robot_state == robotState::GOING){

        // WAIT FOR PARCEL DELIVERY
          std::cout << "[AGENT] WAIT FOR ACCEPTANCE !" <<std::endl <<std::endl;
          int delivery_status = waitfordelivery();

        // After Delivering = Set Back to the First place
          robot_state = robotState::BACKTOBASE;
          *target = startPoint;
          std::cout << "[AGENT] GOING BACK TO : ";
          std::cout << startPoint.target_pose.pose.position.x <<","<<startPoint.target_pose.pose.position.y  <<std::endl;
          requestToSetNewGoal = true;

    }else if(robot_state == robotState::BACKTOBASE){
      std::cout <<std::endl <<std::endl;
    	std::cout << "[AGENT] REACH THE BASE , YAY ! " <<std::endl;
      // IT IS JUST BACK FROM HELL - RELAX, MAN
      robot_state = robotState::IDLE;
      // Ask User For the Next Target
      userInput();

    }else if(robot_state == robotState::SINGLERUN){
    	std::cout <<std::endl <<std::endl;
    	std::cout << "[AGENT] REACH THE TARGET" <<std::endl;
        // SINGLE POINT APPROACH
          robot_state = robotState::IDLE;
        // ASK USER FOR NEXT TARGET
          userInput();

    }else if(robot_state == robotState::EXECUTESEQ){
    	//[LOGGER] WRITE
    	std::cout << "[LOGGER] REACH THE TARGET => WRITEFILE & Reset Timer" <<std::endl ;
    	double timeUse = ros::Time::now().toSec();
    	timeUse = timeUse - startTime;
    	writeFile(startIndex,endIndex,timeUse);
    	startIndex = endIndex;

    	// Normal Flow
    	std::cout <<std::endl <<std::endl;
    	std::cout << "[AGENT] REACH THE TARGET" <<std::endl;
    	std::cout << "[AGENT] EXECUTESEQ NEXT SEQUENCE : "<< targetId <<std::endl;
    	// IDLE
          robot_state = robotState::IDLE;
        //Wait Just a while
          waitfor(3);
        // the Next Sequence
          targetId++;
		  if(targetId >= SEQUENCE_LENGTH){
		  	robot_state = robotState::IDLE;
		  	finish = true;
		  }
		  // Set The Next Sequence
      *target = targets[sequence[targetId]];
      robot_state = robotState::EXECUTESEQ;
      requestToSetNewGoal = true;
      //[LOGGER] NextTarget
      endIndex = sequence[targetId];
    }
}

void goalActiveCallback(){
  ROS_INFO("Goal active! Now !!");
}


void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
  //ROS_INFO("Getting feedback! How cool is that?");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goal");

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("/icreate/move_base", true);

    // Read waypoint from file to vector
    // read_poi_from("/waypoints/waypoint.csv"); 

    // Read waypoint from constant
    read_waypoint_constant();

    ROS_INFO("Successfully Load waypoints !");
    
    // Point The iterator to the beginning of the sequence
    target = targets.begin();

    // Initialize ROBOT :)
    robot_state = robotState::IDLE;

    // Marking Current Location (For First Time Usage)
    markCurrentLocation();

    // Request To Create timer
    requestToCreateTimer = true;

    // Callback polling Rate 
    ros::Rate r(30);

    // Logger init
    generateLog();
  	startIndex = 0; 
  	endIndex = 0;

    // Flag For Marking Base Location (Request Current Location Flag)
    requestToMarkLocation = false;

    // Wait for the action server to come up
    int tries = 0;
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up %d",tries+1);
      tries++;
      if(tries == 3){
        ROS_INFO("Failed to Start Waypoint Node");
        return -1;
      }
    }
    ROS_INFO("Navigation Waypoint Node Initialized !");

    // Subscriber to Get Current position 
    ros::NodeHandle n;

    // Subscribe to Map Clearing Service 
    client = n.serviceClient<std_srvs::Empty>("/move_base_node/clear_costmaps");

    // Ask User For Input
    userInput();

    // Loop for Setting Goal and Navigate ! 
    requestToSetNewGoal = true;

    // Start the Navigation Waypoint Loop
    while(ros::ok() && !finish ){

      // The Next Goal ! 
      if(requestToSetNewGoal){

      	  // [LOGGER]Start the timer 
      	  startTime = ros::Time::now().toSec();

          // Do this Target until its end
          requestToSetNewGoal = false;

          // Move base Goal
          move_base_msgs::MoveBaseGoal goal;
        
          // Send a goal to the robot
          goal.target_pose.header.frame_id = "/map";
          goal.target_pose.header.stamp = ros::Time::now();
        
          // Set the goal
          goal.target_pose.pose.position.x    = target->target_pose.pose.position.x;
          goal.target_pose.pose.position.y    = target->target_pose.pose.position.y;
          goal.target_pose.pose.orientation.x = target->target_pose.pose.orientation.x;
          goal.target_pose.pose.orientation.y = target->target_pose.pose.orientation.y;
          goal.target_pose.pose.orientation.z = target->target_pose.pose.orientation.z;
          goal.target_pose.pose.orientation.w = target->target_pose.pose.orientation.w;
          std::cout << "[AGENT] MARKED NEW TARGET ! " << std::endl;
          // Send Goal to Navigation Stack
          ac.sendGoal(goal, 
                      boost::bind(&goalDoneCallback_state, _1, _2), 
                      boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));

      }

      //Check if the timer should be create.
      if(requestToCreateTimer){
        timer = n.createTimer(ros::Duration(10), timerCallback);
        requestToCreateTimer = false;  //We Don't want to create timer anymore
      }

      // Spinning the loop and Callback
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Exiting Waypoint Navigation");
    return 0;
}
