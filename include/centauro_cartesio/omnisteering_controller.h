#ifndef OMNISTEERINGCONTROLLER_H
#define OMNISTEERINGCONTROLLER_H

#include "centauro_ankle_steering.h"
#include <OpenSoT/tasks/velocity/PureRolling.h>
#include <xbot2_interface/robotinterface2.h>

namespace XBot::Cartesian {

class OmniSteeringController {

public:

    typedef OpenSoT::tasks::velocity::PureRollingPosition RollingTask;
    typedef Centauro::CentauroAnkleSteering SteeringTask;

    OmniSteeringController(ModelInterface::Ptr model,
                           RobotInterface::Ptr robot,
                           std::vector<std::string> wheel_names,
                           std::vector<double> wheel_radius,
                           double dt,
                           double max_steering_speed);

    void setBaseVelocity(const Eigen::Vector6d& v);

    void initWheelPosition();

    void setVelOffsetGain(double gain);

    std::vector<std::string> getWheelJointNames() const;
    std::vector<std::string> getSteeringJointNames() const;

    void update(bool use_base_vel_from_model = false);

private:
    ModelInterface::Ptr _model;
    RobotInterface::Ptr _robot;
    const int _nc;
    const double _dt;
    std::vector<RollingTask> _rolling_tasks;
    std::vector<SteeringTask> _steering_tasks;
    std::vector<int> _rolling_id, _steering_id;

    std::vector<Eigen::Affine3d> _T_init;

    Eigen::Vector6d _vlocal;
    Eigen::VectorXd _q, _qdot;

    double _vel_offset_gain;

};

}

#endif // OMNISTEERINGCONTROLLER_H
