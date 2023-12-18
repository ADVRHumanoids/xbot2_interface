#ifndef MODELINTERFACE2_PIN_H
#define MODELINTERFACE2_PIN_H

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <xbot2_interface/xbotinterface2.h>



namespace XBot {

class ModelInterface2Pin : public ModelInterface
{

public:

    ModelInterface2Pin(const ConfigOptions& opt);

    virtual void update() override;

    int getLinkId(string_const_ref link_name) const override;

    Eigen::Affine3d getPose(int link_id) const override;

    void getJacobian(int link_id, MatRef J) const override;

    Eigen::Vector6d getVelocityTwist(int link_id) const override;

    Eigen::Vector6d getAccelerationTwist(int link_id) const override;

    Eigen::Vector6d getJdotTimesV(int link_id) const override;

    double getMass() const override;

    Eigen::Vector3d getCOM() const override;

    void getCOMJacobian(MatRef J) const override;

    VecConstRef computeInverseDynamics() const override;

    MatConstRef computeRegressor() const;

    void sum(VecConstRef q0, VecConstRef v, Eigen::VectorXd& q1) const override;

    void difference(VecConstRef q1, VecConstRef q0, Eigen::VectorXd& v) const override;


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
        Rnea = 8,
        Com = 16
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