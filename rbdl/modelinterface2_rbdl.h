#ifndef MODELINTERFACE2_RBDL_H
#define MODELINTERFACE2_RBDL_H

#include <xbot2_interface/xbotinterface2.h>

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/Model.h>

namespace XBot {


class ModelInterface2Rbdl : public XBotInterface2
{

public:

    ModelInterface2Rbdl(const ConfigOptions& opt);

    void update() override;
    MatConstRef getJacobian(std::string_view link_name) const override;
    Eigen::Affine3d getPose(std::string_view link_name) const override;
    VecConstRef sum(VecConstRef q0, VecConstRef v) const override;
    VecConstRef difference(VecConstRef q1, VecConstRef q0) const override;

protected:
    JointParametrization get_joint_parametrization(std::string_view jname) override;

private:

    mutable RigidBodyDynamics::Model _mdl;

    struct Temporaries
    {
        Eigen::MatrixXd J, Jaux;
        Eigen::VectorXd qsum, vdiff;

        void resize(int nq, int nv);
    };

    mutable Temporaries _tmp;

};

}
#endif // MODELINTERFACE2_RBDL_H
