#ifndef CENTAUROGCOMP_H
#define CENTAUROGCOMP_H

#include <xbot2/rt_plugin/control_plugin.h>
#include <cartesio_acceleration_support/Force.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <geometry_msgs/WrenchStamped.h>
#include <xbot2/ros/ros_support.h>

namespace XBot {

class CentauroGcomp : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void on_start() override;

    void run() override;

    void on_stop() override;

private:

    struct Contact {
        Cartesian::acceleration::ForceTask::Ptr task;
        Publisher<geometry_msgs::WrenchStamped>::Ptr pub;
        SubscriberBase::Ptr sub;
    };

    ModelInterface::Ptr _model;
    Cartesian::CartesianInterfaceImpl::Ptr _ci;
    std::map<std::string, Contact> _force_tasks;

    std::unique_ptr<RosSupport> _ros;

    Eigen::VectorXd _zero;


};

}

#endif // CENTAUROGCOMP_H
