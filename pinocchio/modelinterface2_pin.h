#ifndef MODELINTERFACE2_PIN_H
#define MODELINTERFACE2_PIN_H

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <xbot2_interface/xbotinterface2.h>



namespace XBot {

class ModelInterface2Pin : public ModelInterface2
{

public:

    ModelInterface2Pin(const ConfigOptions& opt);

    virtual void update() override;

    Eigen::Affine3d getPose(string_const_ref link_name) const override;

    MatConstRef getJacobian(string_const_ref link_name) const override;

    Eigen::Vector6d getVelocityTwist(string_const_ref link_name) const override;

    Eigen::Vector6d getAccelerationTwist(string_const_ref link_name) const override;

    Eigen::Vector6d getJdotTimesV(string_const_ref link_name) const override;

    VecConstRef computeInverseDynamics() const override;

    VecConstRef sum(VecConstRef q0, VecConstRef v) const override;

    VecConstRef difference(VecConstRef q1, VecConstRef q0) const override;

protected:

    JointParametrization get_joint_parametrization(string_const_ref jname) override;

private:

    pinocchio::Index get_frame_id(string_const_ref name) const;

    pinocchio::Model _mdl;
    mutable pinocchio::Data _data;
    mutable pinocchio::Data _data_no_acc;

    std::unordered_map<std::string, pinocchio::Index> _frame_idx;

    pinocchio::ReferenceFrame _world_aligned;

    enum ComputationType
    {
        None = 0,
        Kinematics = 1,
        KinematicsNoAcc = 2,
        Jacobians = 4,
        Rnea = 8
    };

    mutable uint16_t _cached_computation;

    struct Temporaries
    {
        Eigen::MatrixXd J;
        Eigen::VectorXd qsum;
        Eigen::VectorXd qdiff;
        Eigen::VectorXd rnea;

        void resize(int nq, int nv);
    };

    mutable Temporaries _tmp;
    Eigen::VectorXd _qneutral;

    Eigen::VectorXd _vzero;

};

}
#endif // MODELINTERFACE2_PIN_H
