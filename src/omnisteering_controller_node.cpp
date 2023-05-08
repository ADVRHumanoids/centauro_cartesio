#include <centauro_cartesio/omnisteering_controller.h>

#include <ros/ros.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omnisteering_controller_node");
    ros::NodeHandle n("~");

    // get parameters
    std::vector<std::string> wheel_names;
    if(!n.getParam("wheel_names", wheel_names))
    {
        ROS_ERROR("missing parameter '~wheel_names'");
        exit(1);
    }

    std::vector<double> wheel_radius;
    if(!n.getParam("wheel_radius", wheel_radius))
    {
        ROS_ERROR("missing parameter '~wheel_radius'");
        exit(1);
    }

    double dt = 1./n.param("rate", 100.0);
    ros::Rate rate(1./dt);

    ros::Duration cmd_vel_ttl(n.param("vel_cmd_timeout", 0.2));

    // create robot and model
    auto cfg = XBot::ConfigOptionsFromParamServer(ros::NodeHandle("xbotcore"));
    auto robot = XBot::RobotInterface::getRobot(cfg);
    auto model = XBot::ModelInterface::getModel(cfg);

    // sync model
    XBot::JointNameMap qmap;
    robot->getPositionReference(qmap);
    model->setJointPosition(qmap);
    model->update();

    // create controller
    XBot::Cartesian::OmniSteeringController osc(
                model,
                wheel_names,
                wheel_radius,
                dt,
                n.param("max_steering_speed", 1.0));

    // control mode handling
    auto wheel_joints = osc.getWheelJointNames();
    auto steering_joints = osc.getSteeringJointNames();

    std::map<std::string, XBot::ControlMode> ctrl_map;

    for(auto w : wheel_joints)
    {
        ctrl_map[w] = XBot::ControlMode::Velocity();
    }

    for(auto w : steering_joints)
    {
        ctrl_map[w] = XBot::ControlMode::Position();
    }

    robot->setControlMode(XBot::ControlMode::Idle());
    robot->setControlMode(ctrl_map);

    // vref subscriber
    ros::Time cmd_vel_timeout = ros::Time::now();

    auto cmd_vel_cb = [&](const geometry_msgs::TwistConstPtr& msg)
    {
        Eigen::Vector6d vcmd;
        vcmd << msg->linear.x,
                msg->linear.y,
                msg->linear.z,
                msg->angular.x,
                msg->angular.y,
                msg->angular.z;

        osc.setBaseVelocity(vcmd);

        cmd_vel_timeout = ros::Time::now() + cmd_vel_ttl;
    };

    auto sub1 = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmd_vel_cb);



    // main loop
    while(ros::ok())
    {
        // recv cmd vel
        ros::spinOnce();

        // vel timeout
        if(ros::Time::now() > cmd_vel_timeout)
        {
            osc.setBaseVelocity(Eigen::Vector6d::Zero());
        }

        // update controller
        osc.update();

        // send reference
        robot->setReferenceFrom(*model);
        robot->move();

        // sync
        rate.sleep();
    }


}
