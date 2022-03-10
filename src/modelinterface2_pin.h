#ifndef MODELINTERFACE2_PIN_H
#define MODELINTERFACE2_PIN_H

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "impl/xbotinterface2.hxx"



namespace XBot {

class ModelInterface2Pin : public XBotInterface2
{

public:

    ModelInterface2Pin(const ConfigOptions& opt);

    virtual void update() override;

    Eigen::Affine3d getPose(std::string_view link_name) const override;

    MatConstRef getJacobian(std::string_view link_name) const override;

    VecConstRef sum(VecConstRef q0, VecConstRef v) const override;

    VecConstRef difference(VecConstRef q1, VecConstRef q0) const override;

protected:

    JointParametrization get_joint_parametrization(std::string_view jname) override;

private:

    pinocchio::Model _mdl;
    mutable pinocchio::Data _data;

    struct Temporaries
    {
        Eigen::MatrixXd J;
        Eigen::VectorXd qsum;

        void resize(int nq, int nv);
    };

    Temporaries _tmp;
    Eigen::VectorXd _qneutral;


};

}
#endif // MODELINTERFACE2_PIN_H
