#ifndef SAFETY_SENSOR_H
#define SAFETY_SENSOR_H

#include <Eigen/Core>
#include <XBotInterface/Utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>




namespace tree {
  
  template <typename RosMSG>
  class SafetySensor {
    public:
      
      SafetySensor(ros::NodeHandle _nh, std::string _topicName, std::vector<double> _thresholds, Eigen::Affine3d _pose);
      ~SafetySensor();

      float getThreshold() const;
      void setThreshold(const float val);
      
      void getBitMask(Eigen::Vector6d bitMask) const;
      // virtual void run(); 
      virtual void processData(const typename RosMSG::ConstPtr& msg) = 0;
      virtual bool checkSafety(Eigen::Vector6d& referenceTwist) = 0;
      virtual void update() = 0;
      virtual void initialize(); 
    protected:
      std::vector<double> thresholds; // Start slow down once the max is reached then stop 
      std::string baseFrame, sensorFrame;
      Eigen::Affine3d pose;

    private:
      ros::NodeHandle nh;
      ros::Subscriber sub;
      const std::string topicName;

  };
 

  class Sonar : public SafetySensor<sensor_msgs::Range>{
    // static std::map<uint8_t, std::string> sonarMap;

    public:
      Sonar(ros::NodeHandle _nh, std::string _topicName, std::vector<double> _thresholds, Eigen::Affine3d _pose);
      ~Sonar();
      
      bool checkSafety(Eigen::Vector6d& referenceTwist) override;

      bool getObstacleFlag() const;
      double getFOV() const;
      void update() override;
      Eigen::Vector3d getDirection() const;
       
    private:
      void initialize() override;
      const double fov= 2.3; //radians
      double currentDistance;
      bool obstacleFlag;
      Eigen::Vector3d direction;
      void processData(const sensor_msgs::Range::ConstPtr& msg) override;
  };
  

} // namespace tree

#endif // !SAFETY_SENSOR_H
