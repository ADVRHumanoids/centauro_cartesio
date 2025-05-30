#include <centauro_cartesio/omnisteering_safety_sensor.h>
#include <exception>
#include <ros/duration.h>
#include <stdexcept>

namespace tree {

  // SafetySensor::SafetySensor(ros::NodeHandle _nh, std::string _topicName, float _threshold):
  template <typename RosMSG>
  SafetySensor<RosMSG>::SafetySensor(ros::NodeHandle _nh, std::string _topicName, std::vector<double> _thresholds, Eigen::Affine3d _pose):
    nh(_nh), topicName(_topicName), thresholds(_thresholds), pose(_pose)
  {
    // initialize();
    sub = nh.subscribe<RosMSG>(topicName, 10, &SafetySensor::processData, this);
    typename RosMSG::ConstPtr msg = ros::topic::waitForMessage<RosMSG>(topicName, nh, ros::Duration(1.0));

    if(!msg)
      throw std::runtime_error("Unable to find sensor");
 
    sensorFrame = msg->header.frame_id;
    baseFrame = "base_link";
  }
  
  template <typename RosMSG> 
  SafetySensor<RosMSG>::~SafetySensor()
  {

  }
  
  template <typename RosMSG>
  void SafetySensor<RosMSG>::initialize()
  {
    sub = nh.subscribe<RosMSG>(topicName, 10, &SafetySensor::processData, this);
    typename RosMSG::ConstPtr msg = ros::topic::waitForMessage<RosMSG>(topicName, nh);

    sensorFrame = msg->header.frame_id;
    baseFrame = "base_link";
  }
  
  Sonar::Sonar(ros::NodeHandle _nh, std::string _topicName, std::vector<double> _thresholds, Eigen::Affine3d _pose): 
      SafetySensor<sensor_msgs::Range>(_nh, _topicName, _thresholds, _pose)
  {
    SafetySensor<sensor_msgs::Range>::initialize();

  } 
  
  Sonar::~Sonar()
  {

  }

  void Sonar::processData(const sensor_msgs::Range::ConstPtr& msg)
  {

    if (fabs(msg->range) < thresholds[0])
    {
      obstacleFlag = true;
      currentDistance = msg->range;
    }
    else 
    {
      obstacleFlag = false;
    }
  }
  

  void Sonar::initialize()
  {
  }
  
  bool Sonar::checkSafety(Eigen::Vector6d& referenceTwist)
  {
    if(!obstacleFlag or (referenceTwist.norm() <= 0.05)) {return true;}
    
    Eigen::Vector3d linearVel = referenceTwist.head(3);
    Eigen::Vector3d projectedVel = pose.linear() * linearVel;
    Eigen::Vector3d sensorAxis(1, 0, 0);

    std::cout << "Sensor: " << sensorFrame << std::endl;
    std::cout << "Pose Linear: " << pose.linear() << std::endl;
    std::cout << "Projected: "<< projectedVel.x() << std::endl;
    std::cout << "Base: " << linearVel.y() << std::endl;
    
    const int relDirection = copysign(1.0, projectedVel.x()); 
    const double normProduct = projectedVel.norm() * sensorAxis.norm();
    double theta = acos(projectedVel.dot(sensorAxis)/normProduct);



    std::cout << "Theta: " <<  theta << std::endl; 

    if (fabs(theta) < fov/2)
    {
      for (size_t i = 0; i < referenceTwist.head(3).size(); i++)
      {
        referenceTwist(i) *= ((currentDistance - thresholds[1])/(thresholds[0] - thresholds[1]));
      }
      if (currentDistance < thresholds[1])
      {
      referenceTwist.setZero();
      }
            // referenceTwist.setZero();
      return false;
      // std::cout << " S T O P " << std::endl;

    }
    return true;
  }

  void Sonar::update()
  { 
    ros::spinOnce();
  }
  
  bool Sonar::getObstacleFlag() const
  {
    return obstacleFlag;
  }

  Eigen::Vector3d Sonar::getDirection() const
  {
    return direction;
  }
  
  double Sonar::getFOV() const
  {
    return fov;
  }
}
