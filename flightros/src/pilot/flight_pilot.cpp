#include "flightros/pilot/flight_pilot.hpp"

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::NATUREFOREST),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

   // initialize publishers
  image_transport::ImageTransport it(pnh);
  depth_cam_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("/camera_info", 10);
  rgb_pub = it.advertise("/rgb", 1);
  depth_pub = it.advertise("/depth", 1);
  segmentation_pub = it.advertise("/segmentation", 1);
  opticalflow_pub = it.advertise("/opticalflow", 1);
 
  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 1.0, 3.0);
  Matrix<3, 3> R_BC = Quaternion(0.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(91);
  rgb_camera_->setWidth(640);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, false, false});
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);


  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);


  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x+101; //+ 101 -20;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y+86; //+ 86 -20;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z+31;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(send_frame_id_);
    int receive_frame_id = unity_bridge_ptr_->handleOutput();

  if (send_frame_id_ == receive_frame_id) {
  // it means the received image is corresponding to the current send pose, otherwise it was the image from last update
    send_frame_id_ += 1;
  }

    if (quad_ptr_->getCollision()) {
      // collision happened
      ROS_INFO("COLLISION");
    }

    cv::Mat img;

    ros::Time timestamp = ros::Time::now();

    // From avoid_manage.cpp 
    sensor_msgs::CameraInfo cam_info;
    std::vector<double> D{0.0, 0.0, 0.0, 0.0, 0.0};
    boost::array<double, 9> K = {314.463, 0.0, 320.5, 0.0, 314.463, 240.5, 0.0, 0.0, 1.0};
    //boost::array<double, 9> K = {360, 0.0, 640, 0.0, 360, 360, 0.0, 0.0, 1.0};
    boost::array<double, 12> P = {314.463, 0.0, 320.5, -0.0, 0.0, 314.463, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    //boost::array<double, 12> P = {360, 0.0, 640, -0.0, 0.0, 360, 640, 0.0, 0.0, 0.0, 1.0, 0.0};
    boost::array<double, 9> R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_info.height = 480;
    cam_info.width = 640;
    cam_info.distortion_model = "plumb_bob";
    cam_info.D = D;
    cam_info.K = K;
    cam_info.P = P;
    cam_info.R = R;
    cam_info.binning_x = 0;
    cam_info.binning_y = 0;
    cam_info.header.frame_id = "camera_depth_optical_center_link";
    cam_info.header.stamp = timestamp;
    depth_cam_info_pub.publish(cam_info);

    rgb_camera_->getRGBImage(img);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    rgb_pub.publish(rgb_msg);

    rgb_camera_->getDepthMap(img);
    sensor_msgs::ImagePtr depth_msg =
      cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
    depth_msg->header.stamp = timestamp;
    depth_msg->header.frame_id = "camera_depth_optical_center_link";
    depth_pub.publish(depth_msg);

    /*rgb_camera_->getSegmentation(img);
    sensor_msgs::ImagePtr segmentation_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    segmentation_msg->header.stamp = timestamp;
    segmentation_pub.publish(segmentation_msg);

    rgb_camera_->getOpticalFlow(img);
    sensor_msgs::ImagePtr opticflow_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    opticflow_msg->header.stamp = timestamp;
    opticalflow_pub.publish(opticflow_msg);*/

  }
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // empty
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros