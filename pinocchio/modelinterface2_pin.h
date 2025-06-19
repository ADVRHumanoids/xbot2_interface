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

    UniquePtr clone() const override;

    void update_impl() override;

    int getLinkId(string_const_ref link_name) const override;

    Eigen::Affine3d getPose(int link_id) const override;

    void getJacobian(int link_id, MatRef J) const override;

    Eigen::Vector6d getVelocityTwist(int link_id) const override;

    Eigen::Vector6d getAccelerationTwist(int link_id) const override;

    Eigen::Vector6d getJdotTimesV(int link_id) const override;

    double getMass() const override;

    Eigen::Vector3d getCOM() const override;

    Eigen::Vector3d getCOMVelocity() const override;

    Eigen::Vector3d getCOMAcceleration() const override;

    void getCOMJacobian(MatRef J) const override;

    Eigen::Vector3d getCOMJdotTimesV() const override;

    VecConstRef computeInverseDynamics() const override;

    VecConstRef computeGravityCompensation() const override;

    VecConstRef computeForwardDynamics() const override;

    MatConstRef computeInertiaMatrix() const override;

    MatConstRef computeCentroidalMomentumMatrix() const override;

    Eigen::Vector6d computeCentroidalMomentum() const override;

    Eigen::Vector6d computeCentroidalMomentumdotTimesV() const override;

    MatConstRef computeInertiaInverse() const override;

    VecConstRef computeNonlinearTerm() const override;

    MatConstRef computeRegressor() const;

    void sum(VecConstRef q0, VecConstRef v, Eigen::VectorXd& q1) const override;

    void difference(VecConstRef q1, VecConstRef q0, Eigen::VectorXd& v) const override;

    int addFixedLink(string_const_ref link_name,
                      string_const_ref parent_name,
                      double mass,
                      Eigen::Matrix3d inertia,
                      Eigen::Affine3d pose) override;

    bool updateFixedLink(int link_id,
                         double mass,
                         Eigen::Matrix3d inertia,
                         Eigen::Affine3d pose) override;


protected:

    JointParametrization get_joint_parametrization(string_const_ref jname) override;

private:

    pinocchio::Index get_frame_id(string_const_ref name) const;

    void check_frame_idx_throw(int idx) const;

    pinocchio::Model _mdl_orig;
    pinocchio::Model _mdl;
    pinocchio::Model _mdl_zerograv;
    mutable pinocchio::Data _data;
    mutable pinocchio::Data _data_no_acc;

    mutable std::unordered_map<std::string, pinocchio::Index> _frame_idx;

    pinocchio::ReferenceFrame _world_aligned;

    enum ComputationType
    {
        None = 0,
        Kinematics = 1,
        KinematicsNoAcc = 2,
        Jacobians = 4,
        Rnea = 8,
        Gcomp = 16,
        NonlinearEffects = 32,
        Com = 64,
        ComNoAcc = 128,
        Crba = 256,
        Minv = 512,
        CCrba = 1024
    };

    mutable uint16_t _cached_computation;

    struct Temporaries
    {
        Eigen::MatrixXd J;
        Eigen::VectorXd qsum;
        Eigen::VectorXd qdiff;
        Eigen::VectorXd rnea;
        Eigen::VectorXd gcomp;
        Eigen::VectorXd h;

        void resize(int nq, int nv);
    };

    Eigen::MatrixXd _eye;

    mutable Temporaries _tmp;

    Eigen::VectorXd _qneutral;

    Eigen::VectorXd _vzero;

    struct AttachedBody
    {
        pinocchio::Frame frame;
        int frame_idx;
        Eigen::Affine3d m_T_p;
    };

    std::unordered_map<int, AttachedBody> _attached_body_map;


};

}
#endif // MODELINTERFACE2_PIN_H
