#include "centauro_gcomp.h"
#include <std_msgs/Bool.h>

using namespace XBot;


bool XBot::CentauroGcomp::on_initialize()
{
    // model
    _model = ModelInterface::getModel(_robot->getConfigOptions());

    // ci
    auto ci_params = std::make_shared<Cartesian::Parameters>(getPeriodSec());

    auto ctx = std::make_shared<Cartesian::Context>(ci_params, _model);

    auto pb_yaml = getParamOrThrow<YAML::Node>("~problem_description/content");

    Cartesian::ProblemDescription pb(pb_yaml, ctx);

    _ci = Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot", pb, ctx);

    auto contact_links = getParamOr("~contact_links", std::vector<std::string>());

    // save ros publishers and subscribers
    _ros = std::make_unique<RosSupport>(ros::NodeHandle(getName()));

    for(auto c : contact_links)
    {
        auto t = _ci->getTask(c);

        auto ft = std::dynamic_pointer_cast<Cartesian::acceleration::ForceTask>(t);

        if(!ft)
        {
            throw std::runtime_error(fmt::format("bad task '{}' type", c));
        }

        jinfo("found contact {}", c);

        Contact ct;

        ct.task = ft;

        // optimal force value publisher
        ct.pub = _ros->advertise<geometry_msgs::WrenchStamped>(ft->getLinkName() + "/force/value", 1);

        // contact flag subscriber
        Eigen::MatrixXd w_stance, w_swing;
        w_stance = w_swing.setIdentity(ft->getSize(), ft->getSize());
        w_swing *= 1000;

        ct.sub = _ros->subscribe<std_msgs::Bool>(ft->getLinkName() + "/force/contact",
            [ft, w_stance, w_swing](const auto& m)
            {
                if(m.data)
                {
                    ft->setWeight(w_stance);
                }
                else
                {
                    ft->setWeight(w_swing);
                }
            },
            1, &_queue);

        _force_tasks[c] = ct;
    }

    _zero.setZero(_robot->getNv());

    setDefaultControlMode(ControlMode::Effort());

    return true;
}

void XBot::CentauroGcomp::on_start()
{
}

void XBot::CentauroGcomp::run()
{
    // receive contact flags
    _queue.run();

    // sync model
    _model->syncFrom(*_robot, Sync::Position);

    // solve force opt problem
    if(!_ci->update(0, getPeriodSec()))
    {
        jerror("unable to solve");
        return;
    }

    // publish forces to ros
    geometry_msgs::WrenchStamped msg;

    msg.header.stamp.fromSec(chrono::to_sec(chrono::system_clock::now().time_since_epoch()));

    for(auto& [cname, c] : _force_tasks)
    {
        auto f_value = c.task->getForceValue();

        f_value.head<3>() = c.task->getForceFrame().linear().transpose() * f_value.head<3>();

        msg.header.frame_id = c.task->getLinkName();

        msg.wrench.force.x = f_value[0];
        msg.wrench.force.y = f_value[1];
        msg.wrench.force.z = f_value[2];

        c.pub->publish(msg);

    }

    // send torques
    _robot->setReferenceFrom(*_model, Sync::Effort);
    _robot->move();
}

void XBot::CentauroGcomp::on_stop()
{
    _robot->setEffortReference(_zero);
    _robot->move();
}

XBOT2_REGISTER_PLUGIN(CentauroGcomp, centauro_gcomp_plugin)
