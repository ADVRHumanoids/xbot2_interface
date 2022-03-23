#ifndef MODELINTERFACE2_RBDL_H
#define MODELINTERFACE2_RBDL_H

#include <xbot2_interface/xbotinterface2.h>

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>

namespace XBot {


class ModelInterface2Rbdl : public ModelInterface2
{

public:

    ModelInterface2Rbdl(const ConfigOptions& opt);

    void update() override;
    MatConstRef getJacobian(string_const_ref link_name) const override;
    Eigen::Affine3d getPose(string_const_ref link_name) const override;
    VecConstRef computeInverseDynamics() const override;
    VecConstRef sum(VecConstRef q0, VecConstRef v) const override;
    VecConstRef difference(VecConstRef q1, VecConstRef q0) const override;

protected:
    JointParametrization get_joint_parametrization(string_const_ref jname) override;

private:

    mutable RigidBodyDynamics::Model _mdl;

    struct Temporaries
    {
        Eigen::MatrixXd J, Jaux;
        Eigen::VectorXd qsum, vdiff, rnea;

        void resize(int nq, int nv);
    };

    mutable Temporaries _tmp;

};

}
#endif // MODELINTERFACE2_RBDL_H
