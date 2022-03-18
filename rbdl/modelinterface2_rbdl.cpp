#include "modelinterface2_rbdl.h"

#include <xbot2_interface/common/plugin.h>
#include <urdf_parser/urdf_parser.h>
#include <rbdl/Kinematics.h>

using namespace XBot;

ModelInterface2Rbdl::ModelInterface2Rbdl(const XBotInterface2::ConfigOptions &opt):
    ModelInterface2(opt)
{
    // convert urdf dom -> string
    auto tixml = urdf::exportURDF(const_cast<urdf::Model&>(*getUrdf()));
    TiXmlPrinter printer;
    tixml->Accept(&printer);
    std::string urdf = printer.CStr();

    // let rbdl parse it
    if(!RigidBodyDynamics::Addons::URDFReadFromString(urdf.c_str(), &_mdl, false))
    {
        throw std::invalid_argument("rbdl could not parse urdf");
    }

    // we only support "euclidean" models with rbdl
    if(_mdl.q_size != _mdl.dof_count)
    {
        throw std::invalid_argument("_mdl.q_size != _mdl.dof_count: unsupported");
    }

    // resize temporaries
    _tmp.resize(_mdl.q_size, _mdl.dof_count);

    finalize();
}

void ModelInterface2Rbdl::update()
{
    RigidBodyDynamics::UpdateKinematics(_mdl, getJointPosition(), getJointVelocity(), getJointAcceleration());
}

MatConstRef ModelInterface2Rbdl::getJacobian(string_const_ref link_name) const
{
    auto body_id = _mdl.GetBodyId(std::string(link_name).c_str());
    _tmp.J.setZero(6, getNv());
    RigidBodyDynamics::CalcPointJacobian6D(_mdl,
                                           getJointPosition(),
                                           body_id,
                                           Eigen::Vector3d::Zero(),
                                           _tmp.J,
                                           false);

    _tmp.Jaux.topRows<3>() = _tmp.J.bottomRows<3>();
    _tmp.Jaux.bottomRows<3>() = _tmp.J.topRows<3>();
    return _tmp.Jaux;
}

Eigen::Affine3d ModelInterface2Rbdl::getPose(string_const_ref link_name) const
{
    int body_id = _mdl.GetBodyId(std::string(link_name).c_str());

    Eigen::Affine3d T;

    T.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(_mdl,
                                                             getJointPosition(),
                                                             body_id,
                                                             false);

    T.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(_mdl,
                                                                   getJointPosition(),
                                                                   body_id,
                                                                   Eigen::Vector3d::Zero(),
                                                                   false);

    T.linear().transposeInPlace();

    return T;
}

VecConstRef ModelInterface2Rbdl::sum(VecConstRef q0, VecConstRef v) const
{
    _tmp.qsum = q0 + v;
    return _tmp.qsum;
}

VecConstRef ModelInterface2Rbdl::difference(VecConstRef q1, VecConstRef q0) const
{
    _tmp.vdiff = q1 - q0;
    return _tmp.vdiff;
}

XBotInterface2::JointParametrization ModelInterface2Rbdl::get_joint_parametrization(string_const_ref jname)
{
    JointParametrization ret;

    auto urdf = getUrdf();

    auto joint_urdf = urdf->getJoint(std::string(jname));

    size_t body_id = _mdl.GetBodyId(joint_urdf->child_link_name.c_str());

    auto& joint_rbd = _mdl.mJoints.at(body_id);

    ret.iq = joint_rbd.q_index;
    ret.iv = ret.iq;
    ret.nq = joint_rbd.mDoFCount;
    ret.nv = ret.nq;
    ret.id = body_id;

    if(joint_urdf->type == urdf::Joint::FLOATING)
    {
        ret.nq = ret.nv = 6;
        ret.iq -= 3;
        ret.iv -= 3;
    }

    ret.q0.setZero(ret.nq);

    return ret;

}


void ModelInterface2Rbdl::Temporaries::resize(int nq, int nv)
{
    J.setZero(6, nv);
    Jaux.setZero(6, nv);
    vdiff.setZero(nv);
    qsum.setZero(nq);
}

XBOT2_REGISTER_MODEL_PLUGIN(ModelInterface2Rbdl, rbdl);
