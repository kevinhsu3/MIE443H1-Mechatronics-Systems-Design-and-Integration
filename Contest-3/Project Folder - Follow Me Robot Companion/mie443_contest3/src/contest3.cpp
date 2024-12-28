#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <inttypes.h>
#include <std_msgs/Bool.h>
#include <kobuki_msgs/BumperEvent.h>

//BUMPER
#define N_BUMPER (3)

//BUMPER 
//GLOBAL VARIABLE FOR BUMPER 
uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//State or nav related 
geometry_msgs::Twist follow_cmd;
int world_state;
int prev_state;
bool state_lockout = false;
bool obj_detected = false;

//Timer related global vars/structs
ros::Timer state_timer;
std::chrono::time_point<std::chrono::system_clock> state_start;
void set_start_time(); //Start time is set on state change
uint64_t get_time_elapsed(); //Returns millis since state start

bool enable_show_img = false;
int key_time = 0;
int minHessian = 800;
const float ratio_thresh = 0.75f;
//Preallocate memory needed for image matching to make things faster
std::vector<KeyPoint> keypoints_ref, keypoints_img;
Mat descriptor_ref, descriptor_img;
Ptr<SURF> detector = SURF::create( minHessian );
Mat img_ref;

bool matchImage(Mat inp_frame);
void excited();
void rightBumper();
void leftBumper();
void rage();
void fear();
void turnLeft();
void turnRight();


void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
	// if (follow_cmd.linear.x == 0 && follow_cmd.angular.z == 0){
	// 	obj_detected=false;
	// } else obj_detected=true;
}

void centroidCB(const std_msgs::Bool msg){
	obj_detected=msg.data;
	// if (!obj_detected) ROS_INFO("Can't see person!");
	// else ROS_INFO("I see person!");
}

//BUMPER
bool bumperPressed[]={false, false, false};

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumper[msg->bumper]=msg->state;// IDK IF THIS IS RIGHT 
    
    //if a bumper is pressed the robot enters a corresponding state and function to react
    if (bumper[0]==kobuki_msgs::BumperEvent::PRESSED && bumper[1]==kobuki_msgs::BumperEvent::RELEASED) {
        //world_state = 6;
        ROS_INFO("Left Bumper Pressed");
        bumperPressed[0]=true;
    } else if (bumper[1]==kobuki_msgs::BumperEvent::PRESSED) {
        //world_state = 7;
        ROS_INFO("Middle Bumper Pressed");
        bumperPressed[1]=true;
    } else if (bumper[2]==kobuki_msgs::BumperEvent::PRESSED && bumper[1]==kobuki_msgs::BumperEvent::RELEASED) {
        //world_state = 8;
        ROS_INFO("Right Bumper Pressed");
        bumperPressed[2]=true;
    } else {
        bumperPressed[0]=false;
		bumperPressed[1]=false;
		bumperPressed[2]=false;
    }
}

void timerCB(const ros::TimerEvent& event){
	state_timer.stop();
	world_state = 0;
	state_lockout = false;
	ROS_INFO("Releasing lockout");
}

//-------------------------------------------------------------
void imshow_fast(Mat img){
	if (enable_show_img){
		imshow("image", img);
		cv::waitKey(1);
		cv::destroyAllWindows();
	}
	return;
}

void initAll(){ //Consider doing everything in grayscale it make it faster
	string path_to_imgref = ros::package::getPath("mie443_contest3") + "/imgs/imgref.jpg";
	img_ref = imread(path_to_imgref);
	detector -> detectAndCompute( img_ref, noArray(), keypoints_ref, descriptor_ref );
	imshow_fast(img_ref);
}

// call on this to display images
void imshow_emotion(Mat img){
	imshow("image", img);
	cv::waitKey(1); 
}

double angular = 0.0;
double linear = 0.0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	state_timer = nh.createTimer(ros::Duration(30.0), timerCB);
	state_timer.stop();
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber centroid_bool = nh.subscribe("centroid_bool",1,&centroidCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	//imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	world_state = 0;
	prev_state = 0;
	bool ranOnce=false;
	
	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	initAll();

	//sc.playWave(path_to_sounds + "rage.wav");
	ros::Duration(0.5).sleep();
	//Mat img_test=imread(ros::package::getPath("mie443_contest3")+"/imgs/imgtest.jpg");
	//imshow_fast(img_test);
	//imshow_emotion(img_test);

	set_start_time();

	int emotionCtr = 0;
	int centroidCtr = 0;
	// world_state = 8;
	// state_lockout = true;

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();
		//State evaluation, put the most important states first! timerCB will automatically return to state 0 and reset state_lockout
		//ROS_INFO("Time elapsed: %" PRIu64 "\n", get_time_elapsed());
		
		if (world_state == 0 && !obj_detected){
			centroidCtr++;
		} else if (world_state != 0 || obj_detected){
			centroidCtr = 0;
		} else {
			centroidCtr=0;
		}

		//State evaluation
		if (!state_lockout && prev_state == 0 && centroidCtr>50) { //Each ctr is 100ms
			world_state = 5;
			centroidCtr = 0;
		} else if (!state_lockout && prev_state == 0 && bumperPressed[0]){
			world_state = 6;
		} else if (!state_lockout && prev_state == 0 && bumperPressed[1]){
			world_state = 7;
		} else if (!state_lockout && prev_state == 0 && bumperPressed[2]){
			world_state = 8;
		} else if (!state_lockout && prev_state == 0 && get_time_elapsed()%500 < 100 && matchImage(rgbTransport.getImg())){
			world_state = 4;
		} 

		//State execution
		if (prev_state != world_state && world_state != 0){
			state_timer.start();
			state_lockout=true;
			set_start_time();
			ROS_INFO("Starting lockout");
			ROS_INFO("State = %d", world_state);
		}
		if (prev_state != world_state && world_state == 0) ROS_INFO("State = 0");
		prev_state=world_state;

		if(world_state == 0){
			//robot follows person

			//display neutral emotion image
			Mat neutral=imread(ros::package::getPath("mie443_contest3")+"/imgs/neutralCat.jpeg"); 
			imshow_emotion(neutral); 
			state_lockout = false;
			bumperPressed[0]=false;
			bumperPressed[1]=false;
			bumperPressed[2]=false;
			ranOnce=false;
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){
			/*
			nothing happens here can delete
			...
			...
			*/
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);

		}else if (world_state == 4){
			//cat sees picture of tuna and gets excited
			if (!ranOnce) {
				sc.playWave(path_to_sounds + "excited.wav");
				ranOnce=true;
			}
			if (get_time_elapsed()%2000 < 200) {
				sc.playWave(path_to_sounds + "excited.wav");
			}
			excited();
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);
		
		}else if(world_state == 5){
			//cat has lost track of person and gets scared
			if (!ranOnce) {
				sc.playWave(path_to_sounds + "fear1.wav");
				ranOnce=true;
			}
			if (get_time_elapsed()>10000 && get_time_elapsed()<10200){
				sc.playWave(path_to_sounds + "fear2.wav");
			}
		
			fear();
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);
		
		}else if(world_state == 6){
			//left bumper has been kicked and cat gets surprised
			if (!ranOnce) {
				sc.playWave(path_to_sounds + "surprise.wav"); // default sounds
				ranOnce=true;
			}
			leftBumper();
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);

		
		}else if(world_state == 7){
			//center bumper is pressed due to obstacle and cat cannot get to person and rages
			if (!ranOnce) {
				sc.playWave(path_to_sounds + "rage1.wav");
				ranOnce=true;
			}
			if (get_time_elapsed()>10000 && get_time_elapsed()<10200) {
				sc.playWave(path_to_sounds + "rage2.wav");
			}
			rage();
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);

		}else if(world_state == 8){ 
			//right bumper has been kicked and cat gets surprised
			//sc.playWave(path_to_sounds + "surprise.wav");
			if (!ranOnce) {
				sc.playWave(path_to_sounds + "surprise.wav");
				ranOnce=true;
			}
			rightBumper();
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);

		}

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}
	return 0;
}


void excited(){
//robot turns in a circle in excitement and the excited cat image is displayed
	if(get_time_elapsed()<10000) {
		Mat excited=imread(ros::package::getPath("mie443_contest3")+"/imgs/excitedCat.jpeg"); //compiles but does not work
		imshow_emotion(excited);
		linear = 0.5;
		angular = 1;
		return;
	}

	cv::destroyAllWindows();
	world_state=0;
	return;

}

bool matchImage(Mat inp_frame){
	detector -> detectAndCompute( inp_frame, noArray(), keypoints_img, descriptor_img );
	
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptor_ref, descriptor_img, knn_matches, 2 );

	std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    //ROS_INFO("Good matches: %lu", good_matches.size());

	Mat img_matches;
    /* drawMatches( img_ref, keypoints_ref, inp_frame, keypoints_img, good_matches,img_matches, Scalar::all(-1),
    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	imshow_fast(img_matches); */

	if (good_matches.size()>80) return true;
	else return false;
}

void set_start_time(){
	state_start = std::chrono::system_clock::now();
	return;
}
uint64_t get_time_elapsed(){
	uint64_t mil_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-state_start).count();
	return mil_elapsed;
}
void rightBumper() {
    //the rightBumper function turns the robot right by 90 degrees if the right bumper is hit
    //make a surprised cat sound 
    //Return to its original position 
    //returns state 0 to move forward
    if(get_time_elapsed()<=1500){
		//ROS_INFO("in right bumper");
		Mat surpriseRight=imread(ros::package::getPath("mie443_contest3")+"/imgs/surprisedCat.jpeg"); //compiles but does not work
		imshow_emotion(surpriseRight); 
        angular=-1.4;
        linear=-0.2; 
        return;
    }
	if(get_time_elapsed() > 1500 && get_time_elapsed() <= 4000) {
		angular =0;
		linear = 0;
	}
    /*if(get_time_elapsed()>3000 && get_time_elapsed()<6000 ){
        angular=0.8;
        linear=0;
        return;

    }*/else {
		cv::destroyAllWindows();
        world_state=0;
		bumperPressed[0]=false;
		bumperPressed[1]=false;
		bumperPressed[2]=false;
        return;
    }
}

//BUMPER  
void leftBumper() {
    //the rightBumper function turns the robot left by 90 degrees if the right bumper is hit
    //make a surprised cat sound 
    //Return to its original position 
    //returns state 0 to move forward
    if(get_time_elapsed()<=1500){
       	Mat surpriseLeft=imread(ros::package::getPath("mie443_contest3")+"/imgs/surprisedCat.jpeg"); //compiles but does not work
		imshow_emotion(surpriseLeft);
	    angular=1.4;
        linear=-0.2;
        return;
    }
	if(get_time_elapsed() > 1500 && get_time_elapsed() <= 4000) {
		angular =0;
		linear = 0;
	}
	/*if(get_time_elapsed()>3000 && get_time_elapsed()<=6000 ){
        // Mat img_test_sur=imread(ros::package::getPath("mie443_contest3")+"/imgs/meow_surprised_yoga.jpg");
		// imshow_emotion(img_test_sur);
		angular=0.0;
        linear=0;
        return;
	}
    if(get_time_elapsed()>6000 && get_time_elapsed()<9000 ){
        angular=-0.8;
        linear=0;
        return;
    }*/else {
		cv::destroyAllWindows();
        world_state=0;
		bumperPressed[0]=false;
		bumperPressed[1]=false;
		bumperPressed[2]=false;
        return; 
    }
}

void rage() {
	
	//function definition for when bot cannot reach user, triggered by obstacle preventing bot from following
	

	//ROS_INFO("in the rage emotion");
	// reverse for 2 seconds

	if (get_time_elapsed() >= 0 && get_time_elapsed() <= 2500){
		linear  = -0.25; 
		angular = 0.0;
		return;

		// move forward for 2 seconds
	}
	
	if (get_time_elapsed() > 2500 && get_time_elapsed() <= 4500){ // moving forwards back to obstacle
		linear = 0.25;
		angular = 0.0;		
		return;

	// reverse  faster to build frustration
	}
	
	if (get_time_elapsed() > 4500 && get_time_elapsed() <= 6500){
		linear  = -0.3; 
		angular = 0.0;
		//ros::Duration(500).sleep();
		return;

	}
	
	if (get_time_elapsed()> 6500 && get_time_elapsed() <= 7000){
		linear = 0.0;
		angular = 0.0;
		return;

		// move forward for 2 seconds
	}
	
	if (get_time_elapsed() > 7000 && get_time_elapsed() <= 8500){ // moving forwards back to obstacle
		linear = 0.4;
		angular = 0.0;
		return;

	}
	
	if (get_time_elapsed() > 8500 && get_time_elapsed() <=10700) {
		// scratching image
		Mat rageOne=imread(ros::package::getPath("mie443_contest3")+"/imgs/scratchCat.jpeg");
		imshow_emotion(rageOne);

		linear = 0;
		angular = 0;
		return;

	}
	
	if (get_time_elapsed() > 10700 && get_time_elapsed() <= 30000){
		// rage image #2
		//cv::destroyAllWindows();
		Mat rageTwo=imread(ros::package::getPath("mie443_contest3")+"/imgs/catRage.jpeg");
		imshow_emotion(rageTwo);

		linear = 0.0;
		angular = 1.7; 
		return;
	}

	else {
		linear = 0;
		angular = 0;
		cv::destroyAllWindows();
		world_state=0;
		bumperPressed[0]=false;
		bumperPressed[1]=false;
		bumperPressed[2]=false;
		return; // end function
	} 
}


void fear() {
	// this emotional state is triggered when the robot loses track of the person it's following
	//normal picture up;

	//the scared image is put up
	Mat fear=imread(ros::package::getPath("mie443_contest3")+"/imgs/catFear.jpeg");
	imshow_emotion(fear);

	//look left
	if(get_time_elapsed()<=3500){
        angular=0.8;
        linear=0;
        return;
    } 
	//look right
	if(get_time_elapsed()>3500 && get_time_elapsed()<=9000){
        angular=-0.8;
        linear=0;
        return;
	}
	//turn back to center
	if(get_time_elapsed()>9000 && get_time_elapsed()<=12000){
        angular=0.8;
        linear=0;
        return;
    }
	if(get_time_elapsed()>12000 && get_time_elapsed()<=13000){
		angular = 0;
		linear = 0;
		return;
	}

	//robot starts shaking in fear
	if (get_time_elapsed()==13000) ROS_INFO("Start shaking");

	if (get_time_elapsed() > 13000 && get_time_elapsed()%1000<500){
		angular = 0.3;
		linear = 0.1;
		return;
	}
	if (get_time_elapsed() > 13000 && get_time_elapsed()%1000>=500){
		angular = -0.3;
		linear = -0.1;
		return;
	}
	if (get_time_elapsed()>20000){
		cv::destroyAllWindows();
		world_state=0;
		return;
	}
	
}
