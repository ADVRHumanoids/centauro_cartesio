#ifndef OMNISTEERINGCONTROLLER_H
#define OMNISTEERINGCONTROLLER_H

#include "centauro_ankle_steering.h"
#include <OpenSoT/tasks/velocity/PureRolling.h>

namespace XBot::Cartesian {

class OmniSteeringController {

public:

    typedef OpenSoT::tasks::velocity::PureRollingPosition RollingTask;
    typedef Centauro::CentauroAnkleSteering SteeringTask;

    OmniSteeringController(ModelInterface::Ptr model,
                           std::vector<std::string> wheel_names,
                           std::vector<double> wheel_radius,
                           double dt,
                           double max_steering_speed);

    void setBaseVelocity(const Eigen::Vector6d& v);

    std::vector<std::string> getWheelJointNames() const;
    std::vector<std::string> getSteeringJointNames() const;

    void update();

private:

    ModelInterface::Ptr _model;
    const int _nc;
    const double _dt;
    std::vector<RollingTask> _rolling_tasks;
    std::vector<SteeringTask> _steering_tasks;
    std::vector<int> _rolling_id, _steering_id;

    Eigen::Vector6d _vlocal;
    Eigen::VectorXd _q, _qdot;

};

}

#endif // OMNISTEERINGCONTROLLER_H
