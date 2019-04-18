#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/open_sot/TaskInterface.h>
#include <OpenSoT/tasks/velocity/CentauroAnkleSteering.h>
#include <OpenSoT/tasks/velocity/PureRolling.h>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <centauro_cartesio/SetNormal.h>

using namespace XBot::Cartesian;

struct CentauroSteeringTask : public TaskDescription
{
    std::string wheel_name;
    double max_steering_speed;
    Eigen::Vector3d contact_plane_normal;
    
    CentauroSteeringTask();
};

CentauroSteeringTask::CentauroSteeringTask():
    TaskDescription(TaskInterface::None, 
                    "CentauroSteering", 
                    1),
    contact_plane_normal(0, 0, 1.)
{
}

extern "C" TaskDescription * CentauroSteeringTaskDescriptionFactory(YAML::Node task_node, 
                                                     XBot::ModelInterface::ConstPtr model)
{
    CentauroSteeringTask * task_desc = new CentauroSteeringTask;
    
    task_desc->wheel_name = task_node["wheel_name"].as<std::string>();
    
    if(task_node["max_steering_speed"])
    {
        task_desc->max_steering_speed = task_node["max_steering_speed"].as<double>();
    }
    else
    {
        task_desc->max_steering_speed = 3.0;
    }
    
    if(task_node["contact_plane_normal"])
    {
        auto contact_plane_normal_std = task_node["contact_plane_normal"].as<std::vector<double>>();
        
        if(contact_plane_normal_std.size() != 3)
        {
            throw std::runtime_error("contact_plane_normal size() must be 3");
        }
        
        task_desc->contact_plane_normal = Eigen::Vector3d::Map(contact_plane_normal_std.data());
    }
    
    return task_desc;
    
}

class CentauroSteeringOpenSot : public SoT::TaskInterface
{
    
public:
    
    CentauroSteeringOpenSot(TaskDescription::Ptr task_desc, 
                            XBot::ModelInterface::ConstPtr model):
        SoT::TaskInterface(task_desc, model)
    {
        
        auto steering_desc = std::dynamic_pointer_cast<CentauroSteeringTask>(task_desc);
        
        auto steering_task = boost::make_shared<OpenSoT::tasks::velocity::CentauroAnkleSteering>
                                (steering_desc->wheel_name,
                                 model,
                                 0.01,
                                 steering_desc->max_steering_speed
                                );
                                
        _task = steering_task;
        
        steering_task->setOutwardNormal(steering_desc->contact_plane_normal);
                                
        ros::NodeHandle nh;
        _normal_srv = nh.advertiseService("cartesian/centauro_steering/" + steering_desc->wheel_name + "/set_normal",
                                          &CentauroSteeringOpenSot::on_normal_changed, 
                                          this
                                         );
        
    }
    
    SoT::TaskPtr getTaskPtr() const override
    {
        return _task;
    }
    
    bool setBaseLink(const std::string & ee_name, const std::string & base_link) override
    {
        return false;
    }
    
    bool setControlMode(const std::string & ee_name, ControlType ctrl_mode) override
    {
        return false;
    }
    
    bool update(const CartesianInterface * ci, double time, double period) override
    {
        return true;
    }
    
private:
    
    bool on_normal_changed(centauro_cartesio::SetNormalRequest& req, 
                           centauro_cartesio::SetNormalResponse& res)
    {
        Eigen::Vector3d n;
        n << req.plane_normal.x, 
             req.plane_normal.y, 
             req.plane_normal.z;
        
        _task->setOutwardNormal(n);
        
        res.success = true;
        res.message = "Successfully set contact plane normal";
        
        return true;
    }
    
    
    boost::shared_ptr<OpenSoT::tasks::velocity::CentauroAnkleSteering> _task;
    
    ros::ServiceServer _normal_srv;
    
    
};


extern "C" SoT::TaskInterface * CentauroSteeringOpenSotTaskFactory(TaskDescription::Ptr task_desc, 
                                                            XBot::ModelInterface::ConstPtr model)
{
    return new CentauroSteeringOpenSot(task_desc, model);
}


struct WheelRollingTask : public TaskDescription
{
    std::string wheel_name;
    double wheel_radius;
    Eigen::Vector3d contact_plane_normal;
    
    WheelRollingTask();
};

WheelRollingTask::WheelRollingTask():
    TaskDescription(TaskInterface::None, 
                    "WheelRolling", 
                    2),
    contact_plane_normal(0, 0, 1.)
{
}

extern "C" TaskDescription * WheelRollingTaskDescriptionFactory(YAML::Node task_node, 
                                                     XBot::ModelInterface::ConstPtr model)
{
    WheelRollingTask * task_desc = new WheelRollingTask;
    
    task_desc->wheel_name = task_node["wheel_name"].as<std::string>();
    
    task_desc->wheel_radius = task_node["wheel_radius"].as<double>();
    
    if(task_node["contact_plane_normal"])
    {
        auto contact_plane_normal_std = task_node["contact_plane_normal"].as<std::vector<double>>();
        
        if(contact_plane_normal_std.size() != 3)
        {
            throw std::runtime_error("contact_plane_normal size() must be 3");
        }
        
        task_desc->contact_plane_normal = Eigen::Vector3d::Map(contact_plane_normal_std.data());
    }
    
    return task_desc;
    
}

class WheelRollingOpenSot : public SoT::TaskInterface
{
    
public:
    
    WheelRollingOpenSot(TaskDescription::Ptr task_desc, 
                            XBot::ModelInterface::ConstPtr model):
        SoT::TaskInterface(task_desc, model)
    {
        
        auto rolling_desc = std::dynamic_pointer_cast<WheelRollingTask>(task_desc);
        
        auto rolling_task =  boost::make_shared<OpenSoT::tasks::velocity::PureRollingPosition>
                                (rolling_desc->wheel_name,
                                 rolling_desc->wheel_radius,
                                 *model
                                );
                                
        _task = rolling_task;

        rolling_task->setOutwardNormal(rolling_desc->contact_plane_normal);
        
        ros::NodeHandle nh;
        _normal_srv = nh.advertiseService("cartesian/wheel_rolling/" + rolling_desc->wheel_name + "/set_normal",
                                          &WheelRollingOpenSot::on_normal_changed, 
                                          this
                                         );
        
    }
    
    SoT::TaskPtr getTaskPtr() const override
    {
        return _task;
    }
    
    bool setBaseLink(const std::string & ee_name, const std::string & base_link) override
    {
        return false;
    }
    
    bool setControlMode(const std::string & ee_name, ControlType ctrl_mode) override
    {
        return false;
    }
    
    bool update(const CartesianInterface * ci, double time, double period) override
    {
        return true;
    }
    
private:
    
    bool on_normal_changed(centauro_cartesio::SetNormalRequest& req, 
                           centauro_cartesio::SetNormalResponse& res)
    {
        Eigen::Vector3d n;
        n << req.plane_normal.x, 
             req.plane_normal.y, 
             req.plane_normal.z;
        
        _task->setOutwardNormal(n);
        
        res.success = true;
        res.message = "Successfully set contact plane normal";
        
        return true;
    }
    
    
    boost::shared_ptr<OpenSoT::tasks::velocity::PureRollingPosition> _task;
    
    ros::ServiceServer _normal_srv;
    
    
};


extern "C" SoT::TaskInterface * WheelRollingOpenSotTaskFactory(TaskDescription::Ptr task_desc, 
                                                            XBot::ModelInterface::ConstPtr model)
{
    return new WheelRollingOpenSot(task_desc, model);
}

