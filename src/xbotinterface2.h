#ifndef XBOTINTERFACE2_H
#define XBOTINTERFACE2_H

#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <xbot2_interface/types.h>

namespace XBot {

class XBotInterface2
{

public:

    XBOT_DECLARE_SMART_PTR(XBotInterface2);

    XBotInterface2(urdf::ModelConstSharedPtr urdf,
                   srdf::ModelConstSharedPtr srdf);

    static XBotInterface2::Ptr getModel(urdf::ModelConstSharedPtr urdf,
                                        srdf::ModelConstSharedPtr srdf);

    bool hasRobotState(std::string_view name) const;

    Eigen::VectorXd getRobotState(std::string_view name) const;

    VecConstRef getJointPosition() const;

    void setJointPosition(VecConstRef);

    virtual void update() = 0;

    virtual MatConstRef getJacobian(std::string_view link_name) const = 0;

    virtual Eigen::Affine3d getPose(std::string_view link_name) const = 0;

    virtual VecConstRef sum(VecConstRef q0, VecConstRef v) const = 0;

    virtual VecConstRef difference(VecConstRef q1, VecConstRef q0) const = 0;

    ~XBotInterface2();

protected:

    struct JointParametrization
    {
        int id = -1;
        int iq = -1;
        int iv = -1;
        int nq = -1;
        int nv = -1;

        Eigen::VectorXd q0;
    };

    void finalize();

    virtual JointParametrization get_joint_parametrization(std::string_view jname);

private:

    class Impl;

    std::unique_ptr<Impl> impl;

};

}

#endif // XBOTINTERFACE2_H
