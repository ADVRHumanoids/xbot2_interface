#include "modelinterface2_rbdl.h"

#include <xbot2_interface/common/plugin.h>
#include <xbot2_interface/common/utils.h>
#include <urdf_parser/urdf_parser.h>
#include <rbdl/Kinematics.h>

using namespace XBot;

ModelInterface2Rbdl::ModelInterface2Rbdl(const XBotInterface::ConfigOptions &opt):
    ModelInterface(opt)
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

VecConstRef ModelInterface2Rbdl::computeInverseDynamics() const
{
    RigidBodyDynamics::InverseDynamics(_mdl,
                                       getJointPosition(),
                                       getJointVelocity(),
                                       getJointAcceleration(),
                                       _tmp.rnea);

    return _tmp.rnea;
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

XBotInterface::JointParametrization ModelInterface2Rbdl::get_joint_parametrization(string_const_ref jname)
{
    JointParametrization ret;

    auto urdf = getUrdf();

    auto joint_urdf = urdf->getJoint(std::string(jname));

    size_t body_id = _mdl.GetBodyId(joint_urdf->child_link_name.c_str());

    auto& joint_rbd = _mdl.mJoints.at(body_id);

    ret.info.iq = joint_rbd.q_index;
    ret.info.iv = ret.info.iq;
    ret.info.nq = joint_rbd.mDoFCount;
    ret.info.nv = ret.info.nq;
    ret.info.id = body_id;

    if(joint_urdf->type == urdf::Joint::FLOATING)
    {
        ret.info.nq = ret.info.nv = 6;
        ret.info.iq -= 3;
        ret.info.iv -= 3;

        ret.fn_fwd_kin =  [](VecConstRef q, VecConstRef v,
                             Eigen::Affine3d* p_T_c, Eigen::Vector6d* c_vc)
        {
            Eigen::Matrix3d p_R_c  = Utils::rpyToRotationMatrix(q.tail<3>());

            if(p_T_c)
            {
                p_T_c->translation() = q.head<3>();
                p_T_c->linear() = p_R_c;
            }

            if(c_vc)
            {
                c_vc->head<3>() = p_R_c.transpose() * v.head<3>();

                Eigen::Matrix3d om_J_v = Utils::rpyJacobian(q.tail<3>());

                c_vc->tail<3>() = p_R_c.transpose() * om_J_v * v.tail<3>();
            }
        };

        ret.fn_inv_kin =  [](const Eigen::Affine3d& p_T_c, const Eigen::Vector6d& c_vc,
                              VecRef q, VecRef v)
        {
            q.head<3>() = p_T_c.translation();
            q.tail<3>() = Utils::rotationMatrixToRpy(p_T_c.linear());

            v.head<3>() = p_T_c.linear() * c_vc.head<3>();

            Eigen::Matrix3d c_om_J = p_T_c.linear().transpose() * Utils::rpyJacobian(q.tail<3>());
            v.tail<3>() = c_om_J.lu().solve(c_vc.tail<3>());

        };
    }

    ret.q0.setZero(ret.info.nq);

    return ret;

}


void ModelInterface2Rbdl::Temporaries::resize(int nq, int nv)
{
    J.setZero(6, nv);
    Jaux.setZero(6, nv);
    vdiff.setZero(nv);
    qsum.setZero(nq);
    rnea.setZero(nv);
}

XBOT2_REGISTER_MODEL_PLUGIN(ModelInterface2Rbdl, rbdl);
