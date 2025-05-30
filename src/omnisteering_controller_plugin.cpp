#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>
#include <centauro_cartesio/omnisteering_controller.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>

namespace XBot {

class OmnisteeringControllerPlugin : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void run() override;
    
    // Safety
    bool loadParameters();
    void initSensors();
    void checkSafety(Eigen::Vector6d& referenceTwist);
    bool setSafetyModeCb(const std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

private:

    ModelInterface::Ptr _model;
    std::unique_ptr<Cartesian::OmniSteeringController> _osc;
    std::unique_ptr<RosSupport> _ros;
    ros::NodeHandle _nh;
    XBot::CallbackQueue _xbot_queue;

    SubscriberPtr<geometry_msgs::Twist> _cmd_vel_sub;
    SubscriberPtr<Eigen::Vector6d> _cmd_vel_sub_V6;

    chrono::steady_clock::time_point _cmd_vel_timeout, _cmd_vel_timeout_V6;
    std::chrono::nanoseconds _cmd_vel_ttl;

    // Safety
    std::vector<std::string> sensor_names;
    using sonarMap = std::unordered_map<std::string, std::shared_ptr<tree::Sonar>>;
    sonarMap sonars; 
    bool safetyMode;
    int sensorType;
    std::vector<double> sensorThresholds;

    XBot::ServiceServerPtr<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> safety_setter;
};

bool OmnisteeringControllerPlugin::on_initialize()
{
    // SetGlobalJournalLevel(Journal::Level::Low);
    if (!loadParameters())
    {
        return false;
    }

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
    _nh = ros::NodeHandle(getName());
    _ros = std::make_unique<RosSupport>(_nh);

    auto cmd_vel_cb = [this](const geometry_msgs::Twist& msg)
    {
        Eigen::Vector6d vcmd;
        vcmd << msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z;

        // Check safety before setting the velocity
        if(safetyMode)
        {
            checkSafety(vcmd);
        }

        _osc->setBaseVelocity(vcmd);

        _cmd_vel_timeout = chrono::steady_clock::now() + _cmd_vel_ttl;
    };

    auto cmd_vel_cb_V6 = [this](const Eigen::Vector6d& vcmd)
    {
        Eigen::Vector6d vcmd_copy = vcmd;
        
        // Check safety before setting the velocity
        if(safetyMode)
        {
            checkSafety(vcmd_copy);
        }

        _osc->setBaseVelocity(vcmd_copy);

        _cmd_vel_timeout = chrono::steady_clock::now() + _cmd_vel_ttl;
    };

    _cmd_vel_sub = _ros->subscribe<geometry_msgs::Twist>("cmd_vel", cmd_vel_cb, 1);
    
    _cmd_vel_sub_V6 = subscribe<Eigen::Vector6d>( "~cmd_vel_V6", cmd_vel_cb_V6, 1);
    
    safety_setter = _ros->advertiseService("/omnisteering/safety_switch", 
                                            &OmnisteeringControllerPlugin::setSafetyModeCb,
                                            this,
                                            &_xbot_queue);

    return true;
}

void OmnisteeringControllerPlugin::on_start()
{
    _robot->sense();

    JointNameMap qmap;
    _robot->getPositionReference(qmap);
    _model->setJointPosition(qmap);
    _model->update();

    if(safetyMode)
    {
        jinfo("Init Sensors");
        initSensors();
    }

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

bool OmnisteeringControllerPlugin::loadParameters()
{   
    if (!getParam("~use_safety", safetyMode))
    {   
        safetyMode = false;
        jhigh().jwarn("Safety mode disabled");
    }
    
    if (!getParam("~sensor_type", sensorType))
    {   
        sensorType = 0;
        jhigh().jwarn("Loading default sensor type: %i", sensorType);
    }

    if (!getParam("~sensor_names", sensor_names))
    {   
        XBOT2_ERROR("Failed to load sensor_names ");
        safetyMode = false;
        jhigh().jwarn("Safety mode disabled since no sensor_names were provided");
    }
    
    if (!getParam("~sensor_threshold", sensorThresholds))
    {   
        sensorThresholds = {0.75, 0.3};
        jhigh().jwarn("Loading default sensor thresholds {} {}", sensorThresholds[0], sensorThresholds[1]);
    }   

    XBOT2_INFO("Safety Mode {}", safetyMode);
    XBOT2_INFO("Sensortype {}", sensorType);

    return true;
}

void OmnisteeringControllerPlugin::initSensors()
{
    switch (sensorType)
    {
        case 0:
            for (const auto s : sensor_names)
            {
                jinfo("Loading sensor {}", s);
                Eigen::Affine3d pose;
                _model->getPose("base_link", s, pose);
                jinfo("Orientation {}", pose.linear());
                std::string topic = "/bosch_uss5/" + s;
                try 
                {
                    sonars[s] = std::make_shared<tree::Sonar>(_nh, topic, sensorThresholds, pose); 
                } 
                catch (std::exception &e)
                {
                    XBOT2_ERROR("Failed to load sensors, disabling safety");
                    safetyMode = false;
                    break;
                }
            }
            break;

        default:
            XBOT2_ERROR("Unsupported sensor type: {}", sensorType);
            safetyMode = false;
            break;
    }
}

bool OmnisteeringControllerPlugin::setSafetyModeCb(const std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
{
    safetyMode = req.data;
    res.success = true;
    res.message = "Safety mode set to " + std::string(safetyMode ? "ON" : "OFF");
    return true;
}

void OmnisteeringControllerPlugin::checkSafety(Eigen::Vector6d& referenceTwist)
{
    if(!safetyMode) {
        return;
    }

    for (const auto& [key, value] : sonars)
    {
        value->update();
        if (!value->checkSafety(referenceTwist)){
            XBOT2_WARN("Safety check failed, stopping motion");
            // referenceTwist.setZero();
            break;
        }
    }    
}

} // namespace XBot

XBOT2_REGISTER_PLUGIN(XBot::OmnisteeringControllerPlugin,
                      omnisteering_controller_plugin)

