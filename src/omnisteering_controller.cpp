#include <centauro_cartesio/omnisteering_controller.h>

using namespace XBot::Cartesian;

OmniSteeringController::OmniSteeringController(ModelInterface::Ptr model,
                                               std::vector<std::string> wheel_names,
                                               std::vector<double> wheel_radius,
                                               double dt,
                                               double max_steering_speed):
    _model(model),
    _nc(wheel_names.size()),
    _dt(dt)
{
    double deadzone_threshold = 0.01;

    for(int i = 0; i < _nc; i++)
    {
        _rolling_tasks.emplace_back(wheel_names[i],
                                    wheel_radius[i],
                                    *_model);

        _steering_tasks.emplace_back(wheel_names[i],
                                     _model,
                                     dt,
                                     deadzone_threshold,
                                     max_steering_speed);

        auto& urdf = _model->getUrdf();
        auto link = urdf.getLink(wheel_names[i]);
        int rid = _model->getDofIndex(link->parent_joint->name);
        int sid = rid - 1;

        _rolling_id.push_back(rid);
        _steering_id.push_back(sid);
    }
}

void OmniSteeringController::setBaseVelocity(const Eigen::Vector6d &v)
{
    _vlocal = v;
}

std::vector<std::string> OmniSteeringController::getSteeringJointNames() const
{
    std::vector<std::string> ret;

    for(int i = 0; i < _nc; i++)
    {
        int sid = _steering_id[i];
        ret.push_back(_model->getEnabledJointNames()[sid]);
    }

    return ret;
}

std::vector<std::string> OmniSteeringController::getWheelJointNames() const
{
    std::vector<std::string> ret;

    for(int i = 0; i < _nc; i++)
    {
        int rid = _rolling_id[i];
        ret.push_back(_model->getEnabledJointNames()[rid]);
    }

    return ret;
}

void OmniSteeringController::update(bool use_base_vel_from_model)
{
    if(!use_base_vel_from_model)
    {
        Eigen::Affine3d w_T_base;
        _model->getFloatingBasePose(w_T_base);

        Eigen::Vector6d vbase;
        vbase << w_T_base.linear() * _vlocal.head<3>(),
            w_T_base.linear() * _vlocal.tail<3>();

        _model->setFloatingBaseTwist(vbase);
        _model->update();
    }

    for(auto& task : _rolling_tasks)
    {
        task.update(_q);
    }

    for(auto& task : _steering_tasks)
    {
        task.update(_q);
    }

    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);

    for(int i = 0; i < _nc; i++)
    {
        auto& rolling = _rolling_tasks[i];
        auto& steering = _steering_tasks[i];
        int rid = _rolling_id[i];
        int sid = _steering_id[i];
        _qdot[rid] = 0.0;
        _qdot[sid] = 0.0;

        double As = steering.getA()(0, sid);
        double bs = steering.getb().value() - _dt*(steering.getA()*_qdot).value();
        double steering_dq = bs / As;
        _qdot[sid] = steering_dq / _dt;

        Eigen::Vector2d Ar = rolling.getA().block<2,1>(0, rid);
        Eigen::Vector2d br = rolling.getb() - _dt*rolling.getA()*_qdot;

        double rolling_dq = Ar.dot(br)/Ar.squaredNorm();
        _qdot[rid] = rolling_dq / _dt;

        // std::cout << "wheel " << i << "\n";
        // std::cout << "rid=" << rid << ", sid=" << sid << "\n";
        // std::cout << "Ar=" << Ar.transpose() << ", br=" << br.transpose() << "\n";
        // std::cout << "As=" << As << ", bs=" << bs << "\n";
        // std::cout << "vr=" << _qdot[rid] << ", vs=" << _qdot[sid] << "\n\n";

    }

    _q += _qdot*_dt;
    _model->setJointPosition(_q);
    _model->setJointVelocity(_qdot);
    _model->update();

}
