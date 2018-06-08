/*
 * ControllerApplication.cpp
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#include "ControllerApplication.h"
#include <Console/Console.h>
#include <Transform/LinearMath/Quaternion.h>
#include <Transform/LinearMath/Vector3.h>
#include <Time/Rate.h>
#include <DataSet/DataType/PoseStamped.h>
#include <Service/ServiceType/ServiceOdometry.h>
#include <Service/ServiceType/ServiceTransform.h>
#include <Time/Utils.h>
#include <Time/Time.h>
#include <Parameter/Parameter.h>

#define USE_DBG
#ifdef USE_DBG
#define DBG_PRINT printf
#else
DBG_PRINT
#endif

namespace NS_Controller
{
  static double odom_pose_covariance_move[] =
    {1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e3};

  static double odom_pose_covariance_stop[] =
    {1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9};

  static double odom_twist_covariance_move[] =
    {1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e3};

  static double odom_twist_covariance_stop[] =
    {1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9};

  static double imu_orientation_covariance[] =
    {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};

  static double imu_angular_velocity_covariance[] =
    {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};

  static double imu_linear_acceleration_covariance[] =
    {-1, 0, 0, 0, 0, 0, 0, 0, 0};

  ControllerApplication::ControllerApplication()
  {
    memset(&original_pose, 0, sizeof(original_pose));
    memset(&current_odometry, 0, sizeof(current_odometry));
    last_action = 0;
    odom_srv = new NS_Service::Server< sgbot::Odometry >(
        "BASE_ODOM",
        boost::bind(&ControllerApplication::odomService, this, _1));
    odom_tf_srv = new NS_Service::Server< sgbot::tf::Transform2D >(
        "BASE_ODOM_TF",
        boost::bind(&ControllerApplication::odomTransformService, this, _1));

    twist_sub = new NS_DataSet::Subscriber< sgbot::Velocity2D >(
        "TWIST",
        boost::bind(&ControllerApplication::velocityCallback, this, _1));

    slave_action_sub = new NS_DataSet::Subscriber<int>(
    	"MASTER_ACTION",
		boost::bind(&ControllerApplication::slaveActionCallback, this, _1));
    plan_dist_sub = new NS_DataSet::Subscriber<float>(
    		"PLAN_DIST",
			boost::bind(&ControllerApplication::planDistanceCallback, this, _1));
    plan_theta_sub = new NS_DataSet::Subscriber<float>(
    		"PLAN_THETA",
			boost::bind(&ControllerApplication::planThetaCallback, this, _1));

    //current_odom_transform.setIdentity();
  }

  ControllerApplication::~ControllerApplication()
  {
    if(comm)
      delete comm;

    delete odom_srv;
    delete odom_tf_srv;
  }

  void ControllerApplication::getPoseLoop(double frequency)
  {
    NS_NaviCommon::Rate rate(frequency);

    while(running)
    {
      original_pose = getBasePose();
      if(use_ekf_)
      {
        estimate();
      }
      else
      {
        boost::mutex::scoped_lock locker_(base_lock);

        //NS_Transform::Quaternion ori_trans;
        NS_DataType::Quaternion ori_msg;

        ori_msg.x = 0.0f;
        ori_msg.y = 0.0f;
        ori_msg.z = sin(original_pose.theta / 2.0f);
        ori_msg.w = cos(original_pose.theta / 2.0f);
/*
        current_odometry.pose.position.x = original_pose.x;
        current_odometry.pose.position.y = original_pose.y;
        current_odometry.pose.orientation = ori_msg;
        //NS_Transform::quaternionMsgToTF(ori_msg, ori_trans);
  */


        current_odom_transform = sgbot::tf::Transform2D(original_pose.x, original_pose.y, original_pose.theta, 1);
/*
        current_odom_transform = NS_Transform::Transform(
            ori_trans,
            NS_Transform::Vector3(original_pose.x, original_pose.y, 0));
            */
      }
      rate.sleep();
    }
  }

  void ControllerApplication::estimate()
  {
    NS_Transform::Quaternion qo;

    NS_DataType::Quaternion q_msg;

    q_msg.x = 0.0f;
    q_msg.y = 0.0f;
    q_msg.z = sin(original_pose.theta / 2.0f);
    q_msg.w = cos(original_pose.theta / 2.0f);

    NS_Transform::quaternionMsgToTF(q_msg, qo);

    NS_Transform::Transform odom_measure;
    odom_measure = NS_Transform::Transform(
        qo, NS_Transform::Vector3(original_pose.x, original_pose.y, 0));

    console.debug("Base odometry: %f, %f, %f.", original_pose.x, original_pose.y, original_pose.theta);
    console.debug("Base velocity: %f, %f.", original_pose.linear_vel, original_pose.angular_vel);
    console.debug("Base Imu yaw: %f", original_pose.yaw);

    //add odom measure
    MatrixWrapper::SymmetricMatrix odom_covar(6);
    if(original_pose.linear_vel == 0 && original_pose.angular_vel == 0)
    {
      for(unsigned int i = 0; i < 6; i++)
        for(unsigned int j = 0; j < 6; j++)
          odom_covar(i + 1, j + 1) = odom_pose_covariance_stop[(6 * i) + j];
    }
    else
    {
      for(unsigned int i = 0; i < 6; i++)
        for(unsigned int j = 0; j < 6; j++)
          odom_covar(i + 1, j + 1) = odom_pose_covariance_move[(6 * i) + j];
    }
    estimation.addMeasurement("odom", odom_measure, odom_covar);

    NS_Transform::Quaternion qi;
    NS_Transform::Transform imu_measure;
    NS_DataSet::Subscriber< NS_DataType::Twist >* twist_sub;
    qi = NS_Transform::createQuaternionFromRPY(original_pose.roll,
                                               original_pose.pitch,
                                               original_pose.yaw);
    imu_measure = NS_Transform::Transform(qi, NS_Transform::Vector3(0, 0, 0));

    //add imu measure
    MatrixWrapper::SymmetricMatrix imu_covar(3);
    for(unsigned int i = 0; i < 3; i++)
      for(unsigned int j = 0; j < 3; j++)
        imu_covar(i + 1, j + 1) = imu_orientation_covariance[(3 * i) + j];
    estimation.addMeasurement("imu", imu_measure, imu_covar);

    NS_NaviCommon::Time filter_stamp = NS_NaviCommon::Time::now();
    if(estimation.update(filter_stamp))
    {
      NS_Transform::StampedTransform ekf_trans;

      estimation.getEstimate(ekf_trans);

      //current_odom_transform = ekf_trans;

     // NS_Transform::poseTFToMsg(current_odom_transform, current_odometry.pose);

      //current_odom_transform.getOrigin().setZ(0.0);

     // console.debug("EKF transform : %f, %f, %f", current_odometry.pose.position.x, current_odometry.pose.position.y, NS_Transform::getYaw(current_odometry.pose.orientation));
    }

    if(!estimation.isInitialized())
    {
      estimation.initialize(odom_measure, NS_NaviCommon::Time::now());
    }
  }

  void ControllerApplication::odomService(
		  sgbot::Odometry& odometry)
  {
    boost::mutex::scoped_lock locker_(base_lock);

    odometry.pose2d= sgbot::Pose2D(original_pose.x, original_pose.y, original_pose.theta);
    odometry.velocity2d.linear = original_pose.linear_vel;
    odometry.velocity2d.angular = original_pose.angular_vel;

/*
    current_odometry.twist.linear.x = original_pose.linear_vel;
    current_odometry.twist.angular.z = original_pose.angular_vel;
    */
  }

  PoseState ControllerApplication::getBasePose()
  {
    PoseState p;

    boost::mutex::scoped_lock locker_(base_lock);

    p.x = comm->getFloat64Value(BASE_REG_ODOM_X);
    p.y = comm->getFloat64Value(BASE_REG_ODOM_Y);
    p.theta = comm->getFloat64Value(BASE_REG_ODOM_THETA);
    p.linear_vel = comm->getFloat64Value(BASE_REG_ODOM_LINEAR_VEL);
    p.angular_vel = comm->getFloat64Value(BASE_REG_ODOM_ANGULAR_VEL);

    p.roll = comm->getFloat64Value (BASE_REG_IMU_ROLL);
    p.pitch = comm->getFloat64Value (BASE_REG_IMU_PITCH);
    p.yaw = comm->getFloat64Value (BASE_REG_IMU_YAW);

    return p;
  }

  void ControllerApplication::odomTransformService(
      sgbot::tf::Transform2D& transform)
  {
    boost::mutex::scoped_lock locker_(base_lock);
    transform = current_odom_transform;
    //NS_Transform::transformTFToMsg(current_odom_transform, transform.transform);
    //transform.result = true;
  }

  void ControllerApplication::velocityCallback(sgbot::Velocity2D& velocity2d)
  {
    boost::mutex::scoped_lock locker_(base_lock);

    double linear = velocity2d.linear;
    double angular = velocity2d.angular;
    DBG_PRINT("[velocity2d][%f,%f]\n", linear, angular);
    comm->setFloat64Value(BASE_REG_LINEAR_V, linear);
    comm->setFloat64Value(BASE_REG_ANGULAR_V, angular);
    comm->setInt32Value(BASE_REG_V_SETTED, 1);

  }


  void ControllerApplication::slaveActionCallback(int action)
  {
	  DBG_PRINT("[action sub]>>>>>%d\n", action);
	  boost::mutex::scoped_lock locker_(base_lock);
	  comm->setInt32Value(BASE_REG_ACTION, action);
  }

  void ControllerApplication::planDistanceCallback(float distance)
  {
	  boost::mutex::scoped_lock locker_(base_lock);
	  comm->setFloat32Value(BASE_REG_BORDER_DIST, distance);
	  comm->setFloat32Value(BASE_REG_BORDER_THETA, 0);
	  comm->setInt32Value(BASE_REG_BORDER_SYNC, 1);
  }

  void ControllerApplication::planThetaCallback(float theta)
  {
	  boost::mutex::scoped_lock locker_(base_lock);
	  comm->setFloat32Value(BASE_REG_BORDER_THETA, theta);
	  comm->setFloat32Value(BASE_REG_BORDER_DIST, 0.0f);
	  comm->setInt32Value(BASE_REG_BORDER_SYNC, 1);
  }

  void ControllerApplication::loadParameters()
  {
    NS_NaviCommon::Parameter parameter;
    parameter.loadConfigurationFile("controller.xml");
    comm_dev_name_ = parameter.getParameter("device", "/dev/stm32");
    wheel_diameter_ = parameter.getParameter("wheel_diameter", 0.068f);
    encoder_resolution_ = parameter.getParameter("encoder_resolution", 16);
    gear_reduction_ = parameter.getParameter("gear_reduction", 62);

    wheel_track_ = parameter.getParameter("wheel_track", 0.265f);
    accel_limit_ = parameter.getParameter("accel_limit", 1.0f);

    pid_kp_right_ = parameter.getParameter("pid_kp_r", 20.0f);
    pid_kd_right_ = parameter.getParameter("pid_kd_r", 12.0f);
    pid_ki_right_ = parameter.getParameter("pid_ki_r", 0.0f);
    pid_ko_right_ = parameter.getParameter("pid_ko_r", 50.0f);
    pid_max_right_ = parameter.getParameter("pid_max_r", 100.0f);
    pid_min_right_ = parameter.getParameter("pid_min_r", 0.0f);

    pid_kp_left_ = parameter.getParameter("pid_kp_l", 20.0f);
    pid_kd_left_ = parameter.getParameter("pid_kd_l", 12.0f);
    pid_ki_left_ = parameter.getParameter("pid_ki_l", 0.0f);
    pid_ko_left_ = parameter.getParameter("pid_ko_l", 50.0f);
    pid_max_left_ = parameter.getParameter("pid_max_l", 100.0f);
    pid_min_left_ = parameter.getParameter("pid_min_l", 0.0f);

    control_duration_ = parameter.getParameter("control_duration", 100);
    control_timeout_ = parameter.getParameter("control_timeout", 1000);

    sync_duration_ = parameter.getParameter("sync_duration", 100);

    if(parameter.getParameter("use_ekf", 1) == 1)
    {
      use_ekf_ = true;
    }
    else
    {
      use_ekf_ = false;
    }
  }

  void ControllerApplication::configController()
  {
    comm->setFloat64Value(BASE_REG_WHEEL_TRACK, wheel_track_);
    comm->setFloat64Value(BASE_REG_WHEEL_DIAMETER, wheel_diameter_);
    comm->setFloat64Value(BASE_REG_ENCODER_RESOLUTION, encoder_resolution_);
    comm->setFloat64Value(BASE_REG_GEAR_REDUCTION, gear_reduction_);
    comm->setFloat64Value(BASE_REG_ACCEL_LIMIT, accel_limit_);

    comm->setFloat64Value(BASE_REG_PID_KP_RIGHT, pid_kp_right_);
    comm->setFloat64Value(BASE_REG_PID_KI_RIGHT, pid_ki_right_);
    comm->setFloat64Value(BASE_REG_PID_KD_RIGHT, pid_kd_right_);
    comm->setFloat64Value(BASE_REG_PID_KO_RIGHT, pid_ko_right_);
    comm->setFloat64Value(BASE_REG_PID_MAX_RIGHT, pid_max_right_);
    comm->setFloat64Value(BASE_REG_PID_MIN_RIGHT, pid_min_right_);

    comm->setFloat64Value(BASE_REG_PID_KP_LEFT, pid_kp_left_);
    comm->setFloat64Value(BASE_REG_PID_KI_LEFT, pid_ki_left_);
    comm->setFloat64Value(BASE_REG_PID_KD_LEFT, pid_kd_left_);
    comm->setFloat64Value(BASE_REG_PID_KO_LEFT, pid_ko_left_);
    comm->setFloat64Value(BASE_REG_PID_MAX_LEFT, pid_max_left_);
    comm->setFloat64Value(BASE_REG_PID_MIN_LEFT, pid_min_left_);

    comm->setInt32Value(BASE_REG_CNTL_DURATION, control_duration_);
    comm->setInt32Value(BASE_REG_VEL_TIMEOUT, control_timeout_);

    comm->setInt32Value(BASE_REG_SYNC_DURATION, sync_duration_);

    comm->setInt32Value(BASE_REG_CFG_DONE, 1);

    console.debug("finish config base parameter!");
  }

  bool ControllerApplication::checkDevice()
  {
    unsigned int test_code = 1234;
    unsigned int test_val = 0;
    comm->setInt32Value(BASE_REG_TEST, test_code);
    NS_NaviCommon::delay(100);
    comm->setInt32Value(BASE_REG_TEST, test_code);
    test_val = comm->getInt32Value(BASE_REG_TEST);
    if(test_val != test_code)
    {
      console.debug("test stm32 connection... check code [%d], but get [%d]!",
                    test_code, test_val);
      return false;
    }
    console.debug("test stm32 connection...ok!");

    return true;
  }

  void ControllerApplication::run()
  {
    console.message("controller is running!");

    loadParameters();

    comm = new SpiComm(comm_dev_name_);
    if(!comm->open())
    {
      console.error("can't open base controller device!");
      return;
    }

    configController();

    running = true;

    //get_pose_thread = boost::thread(
    //    boost::bind(&ControllerApplication::getPoseLoop, this,
     //               control_duration_));
    get_pose_thread = boost::thread(
    		boost::bind(&ControllerApplication::getPoseLoop, this,
    				   sync_duration_));
  }

  void ControllerApplication::quit()
  {
    console.message("controller is quitting!");

    running = false;

    get_pose_thread.join();
  }

} /* namespace NS_Controller */
