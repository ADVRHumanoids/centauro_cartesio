#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>
#include <centauro_cartesio/omnisteering_controller.h>
#include <geometry_msgs/Twist.h>

namespace XBot {

class OmnisteeringControllerPlugin : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void run() override;

private:

    ModelInterface::Ptr _model;
    std::unique_ptr<Cartesian::OmniSteeringController> _osc;
    std::unique_ptr<RosSupport> _ros;

    SubscriberPtr<geometry_msgs::Twist> _cmd_vel_sub;
    SubscriberPtr<Eigen::Vector6d> _cmd_vel_sub_V6;

    chrono::steady_clock::time_point _cmd_vel_timeout, _cmd_vel_timeout_V6;
    std::chrono::nanoseconds _cmd_vel_ttl;
};

bool OmnisteeringControllerPlugin::on_initialize()
{
    _model = ModelInterface::getModel(_robot->getConfigOptions());

    // parameters
    auto wheel_names = getParamOrThrow<std::vector<std::string>>("~wheel_names");

    auto wheel_radius = getParamOrThrow<std::vector<double>>("~wheel_radius");

    double max_steering_speed = getParamOr("~max_steering_speed", 2.0);

    _cmd_vel_ttl = 200ms;
    getParam("cmd_vel_ttl", _cmd_vel_ttl);


    // create controller
    _osc = std::make_unique<Cartesian::OmniSteeringController>(
                _model, wheel_names, wheel_radius, getPeriodSec(), max_steering_speed
                );

    // control mode handling
    auto wheel_joints = _osc->getWheelJointNames();
    auto steering_joints = _osc->getSteeringJointNames();

    std::map<std::string, XBot::ControlMode> ctrl_map;

    for(auto w : wheel_joints)
    {
        ctrl_map[w] = XBot::ControlMode::Velocity();
    }

    for(auto w : steering_joints)
    {
        ctrl_map[w] = XBot::ControlMode::Position();
    }

    setDefaultControlMode(ctrl_map);

    // ros
    _ros = std::make_unique<RosSupport>(ros::NodeHandle(getName()));

    auto cmd_vel_cb = [this](const geometry_msgs::Twist& msg)
    {
        Eigen::Vector6d vcmd;
        vcmd << msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z;


        _osc->setBaseVelocity(vcmd);

        _cmd_vel_timeout = chrono::steady_clock::now() + _cmd_vel_ttl;
    };

    auto cmd_vel_cb_V6 = [this](const Eigen::Vector6d& vcmd)
    {
        _osc->setBaseVelocity(vcmd);

        _cmd_vel_timeout = chrono::steady_clock::now() + _cmd_vel_ttl;
    };

    _cmd_vel_sub = _ros->subscribe<geometry_msgs::Twist>("cmd_vel", cmd_vel_cb, 1);
    
    _cmd_vel_sub_V6 = subscribe<Eigen::Vector6d>( "~cmd_vel_V6", cmd_vel_cb_V6, 1);
    
    

    return true;
}

void OmnisteeringControllerPlugin::on_start()
{
    _robot->sense();

    JointNameMap qmap;
    _robot->getPositionReference(qmap);
    _model->setJointPosition(qmap);
    _model->update();

}

void OmnisteeringControllerPlugin::run()
{
    // recv cmd vel
    _cmd_vel_sub->run();
    _cmd_vel_sub_V6->run();

    // vel timeout
    if(_cmd_vel_timeout < chrono::steady_clock::now())
    {
        _osc->setBaseVelocity(Eigen::Vector6d::Zero());
    }

    // update controller
    _osc->update();

    // send reference
    _robot->setReferenceFrom(*_model);
    _robot->move();
}

}

XBOT2_REGISTER_PLUGIN(XBot::OmnisteeringControllerPlugin,
                      omnisteering_controller_plugin)

