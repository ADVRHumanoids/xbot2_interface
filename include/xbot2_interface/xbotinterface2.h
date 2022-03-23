#ifndef XBOTINTERFACE2_H
#define XBOTINTERFACE2_H

#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include "joint.h"

namespace XBot {

class RobotInterface2;

class ModelInterface2;

class XBotInterface2 : public ReadStateInterface<XBotInterface2>
{

public:

    XBOT_DECLARE_SMART_PTR(XBotInterface2);

    struct ConfigOptions
    {
        urdf::ModelConstSharedPtr urdf;
        srdf::ModelConstSharedPtr srdf;
    };

    explicit XBotInterface2(const ConfigOptions& opt);

    XBotInterface2(urdf::ModelConstSharedPtr urdf,
                   srdf::ModelConstSharedPtr srdf = nullptr);

    XBotInterface2(XBotInterface2&&) = default;

    urdf::ModelConstSharedPtr getUrdf() const;

    srdf::ModelConstSharedPtr getSrdf() const;

    bool hasRobotState(string_const_ref name) const;

    Eigen::VectorXd getRobotState(string_const_ref name) const;

    int getJointNum() const;

    Joint::ConstPtr getJoint(string_const_ref name) const;

    Joint::ConstPtr getJoint(int i) const;

    virtual void update() = 0;

    virtual MatConstRef getJacobian(string_const_ref link_name) const = 0;

    void getJacobian(string_const_ref link_name, MatRef J) const;

    void getJacobian(string_const_ref link_name, Eigen::MatrixXd& J) const;

    virtual Eigen::Affine3d getPose(string_const_ref link_name) const = 0;

    virtual Eigen::Vector6d getVelocityTwist(string_const_ref link_name) const;

    virtual VecConstRef computeInverseDynamics() const = 0;

    virtual VecConstRef sum(VecConstRef q0, VecConstRef v) const = 0;

    virtual VecConstRef difference(VecConstRef q1, VecConstRef q0) const = 0;

    virtual ~XBotInterface2();

    friend ReadStateInterface<XBotInterface2>;

    friend WriteStateInterface<ModelInterface2>;

    friend RobotInterface2;

protected:

    void finalize();

    struct JointParametrization
    {
        int id = -1;
        int iq = -1;
        int iv = -1;
        int nq = -1;
        int nv = -1;

        Eigen::VectorXd q0;

        std::function<void(VecConstRef, VecRef)> fn_minimal_to_q;

        std::function<void(VecConstRef, VecRef)> fn_q_to_minimal;

        std::function<void(VecConstRef,
                           VecConstRef,
                           Eigen::Affine3d*,
                           Eigen::Vector6d*)> fn_fwd_kin;

        std::function<void(const Eigen::Affine3d& p_T_c,
                           const Eigen::Vector6d& c_vc,
                           VecRef q,
                           VecRef v)> fn_inv_kin;

    };


    virtual JointParametrization get_joint_parametrization(string_const_ref jname);


protected:

    UniversalJoint::Ptr getUniversalJoint(string_const_ref name);

    UniversalJoint::Ptr getUniversalJoint(int i);

    class Impl;

    XBotInterface2(std::shared_ptr<Impl> impl);

    std::shared_ptr<Impl> impl;

};

class ModelInterface2 : public XBotInterface2,
                        public WriteStateInterface<ModelInterface2>
{

public:

    XBOT_DECLARE_SMART_PTR(ModelInterface2);

    using XBotInterface2::XBotInterface2;

    static UniquePtr getModel(urdf::ModelConstSharedPtr urdf,
                              srdf::ModelConstSharedPtr srdf,
                              std::string type);

    void syncFrom(const XBotInterface2& other);

    ModelJoint::Ptr getJoint(string_const_ref name);

    ModelJoint::Ptr getJoint(int i);

    virtual ~ModelInterface2();

};

}

#endif // XBOTINTERFACE2_H
