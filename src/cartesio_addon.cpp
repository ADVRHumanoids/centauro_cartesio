#include <cartesian_interface/sdk/problem/Task.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <OpenSoT/tasks/velocity/CentauroAnkleSteering.h>
#include <OpenSoT/tasks/velocity/PureRolling.h>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <centauro_cartesio/SetNormal.h>

using namespace XBot::Cartesian;

struct CentauroSteeringTask : public TaskDescriptionImpl
{
    CARTESIO_DECLARE_SMART_PTR(CentauroSteeringTask)

    std::string wheel_name;
    double max_steering_speed;
    Eigen::Vector3d contact_plane_normal;
    
    CentauroSteeringTask(YAML::Node task_node,
                         Context::ConstPtr ctx);
};

CentauroSteeringTask::CentauroSteeringTask(YAML::Node task_node,
                                           Context::ConstPtr ctx):
    TaskDescriptionImpl(task_node,
                        ctx,
                        "steering_" + task_node["wheel_name"].as<std::string>(),
                        1),
    contact_plane_normal(0, 0, 1.)
{
    wheel_name = task_node["wheel_name"].as<std::string>();

    if(task_node["max_steering_speed"])
    {
        max_steering_speed = task_node["max_steering_speed"].as<double>();
    }
    else
    {
        max_steering_speed = 3.0;
    }

    if(task_node["contact_plane_normal"])
    {
        auto contact_plane_normal_std = task_node["contact_plane_normal"].as<std::vector<double>>();

        if(contact_plane_normal_std.size() != 3)
        {
            throw std::runtime_error("contact_plane_normal size() must be 3");
        }

        contact_plane_normal = Eigen::Vector3d::Map(contact_plane_normal_std.data());
    }
}

CARTESIO_REGISTER_TASK_PLUGIN(CentauroSteeringTask, CentauroSteering);

class CentauroSteeringOpenSot : public OpenSotTaskAdapter
{
    
public:
    
    CentauroSteeringOpenSot(TaskDescription::Ptr task_desc,
                            Context::ConstPtr ctx):
        OpenSotTaskAdapter(task_desc, ctx)
    {
        
        _ci_steering = std::dynamic_pointer_cast<CentauroSteeringTask>(task_desc);

        if(!_ci_steering) throw std::runtime_error("Provided task description "
                                                   "does not have expected type 'CentauroSteeringTask'");
    }
    
    TaskPtr constructTask() override
    {
        _sot_steering = boost::make_shared<OpenSoT::tasks::velocity::CentauroAnkleSteering>
                (_ci_steering->wheel_name,
                 _model,
                 _ctx->params()->getControlPeriod(),
                 _ci_steering->max_steering_speed
                 );

        return _sot_steering;
    }
    
    
private:
    
    boost::shared_ptr<OpenSoT::tasks::velocity::CentauroAnkleSteering> _sot_steering;
    CentauroSteeringTask::Ptr _ci_steering;
    
};

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(CentauroSteeringOpenSot, CentauroSteering);


struct WheelRollingTask : public TaskDescriptionImpl
{
    CARTESIO_DECLARE_SMART_PTR(WheelRollingTask)

    std::string wheel_name;
    double wheel_radius;
    Eigen::Vector3d contact_plane_normal;
    
    WheelRollingTask(YAML::Node task_node,
                     Context::ConstPtr ctx);
};

WheelRollingTask::WheelRollingTask(YAML::Node task_node,
                                   Context::ConstPtr ctx):
    TaskDescriptionImpl (task_node,
                         ctx,
                         "rolling_" + task_node["wheel_name"].as<std::string>(),
                         2),
    contact_plane_normal(0, 0, 1.)
{
    wheel_name = task_node["wheel_name"].as<std::string>();

    wheel_radius = task_node["wheel_radius"].as<double>();

    if(task_node["contact_plane_normal"])
    {
        auto contact_plane_normal_std = task_node["contact_plane_normal"].as<std::vector<double>>();

        if(contact_plane_normal_std.size() != 3)
        {
            throw std::runtime_error("contact_plane_normal size() must be 3");
        }

        contact_plane_normal = Eigen::Vector3d::Map(contact_plane_normal_std.data());
    }
}

CARTESIO_REGISTER_TASK_PLUGIN(WheelRollingTask, WheelRolling)


class WheelRollingOpenSot : public OpenSotTaskAdapter
{

public:

    WheelRollingOpenSot(TaskDescription::Ptr task_desc,
                        Context::ConstPtr ctx):
        OpenSotTaskAdapter(task_desc, ctx)
    {

        _ci_rolling = std::dynamic_pointer_cast<WheelRollingTask>(task_desc);

        if(!_ci_rolling) throw std::runtime_error("Provided task description "
                                                   "does not have expected type 'WheelRollingTask'");
    }

    TaskPtr constructTask() override
    {
        auto sot_rolling = boost::make_shared<OpenSoT::tasks::velocity::PureRollingPosition>
                        (_ci_rolling->wheel_name,
                         _ci_rolling->wheel_radius,
                         *_model
                         );

        return sot_rolling;
    }


private:

    WheelRollingTask::Ptr _ci_rolling;

};

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(WheelRollingOpenSot, WheelRolling);
