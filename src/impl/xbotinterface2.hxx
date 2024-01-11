#ifndef XBOTINTERFACE2_HXX
#define XBOTINTERFACE2_HXX

#include <map>
#include <string>

#include <xbot2_interface/xbotinterface2.h>
#include "state.hxx"
#include "chain.hxx"
#include "joint.hxx"


namespace XBot {

class XBotInterface::Impl
{

public:

    friend class XBotInterface;
    friend class ModelInterface;
    friend class RobotInterface;
    friend ReadStateInterface<XBotInterface>;
    friend ReadCmdInterface<RobotInterface>;
    friend WriteCmdInterface<RobotInterface>;
    friend WriteStateInterface<ModelInterface>;

    Impl(urdf::ModelConstSharedPtr urdf,
         srdf::ModelConstSharedPtr srdf,
         XBotInterface& api);

    Eigen::VectorXd getRobotState(string_const_ref name) const;

    UniversalJoint::Ptr getJoint(string_const_ref name) const;

    UniversalJoint::Ptr getJoint(int i) const;

    JointParametrization get_joint_parametrization(string_const_ref jname);

    int get_link_id_error(string_const_ref link_name) const;

    int get_link_id_throw(string_const_ref link_name) const;

    void finalize();

private:

    void parse_ft();

    void parse_imu();

    void parse_chains();

    // type
    std::string _type;

    // api
    XBotInterface * _api;

    // urdf dom
    urdf::ModelConstSharedPtr _urdf;

    // srdf dom
    srdf::ModelConstSharedPtr _srdf;

    // model state
    detail::State _state;

    // model command
    detail::Command _cmd;

    // joint name -> joint id
    std::map<std::string, int> _name_id_map;
    std::map<std::string, int> _qname_iq_map;
    std::map<std::string, int> _vname_iv_map;

    // joint id -> conf dim, tangent dim
    std::vector<JointInfo> _joint_info;

    // joint id -> name
    std::vector<std::string> _joint_name;

    // joints
    std::vector<UniversalJoint::Ptr> _joints;
    std::vector<Joint::Ptr> _joints_xbi;
    std::vector<Joint::ConstPtr> _joints_xbi_const;
    std::vector<ModelJoint::Ptr> _joints_mdl;
    std::vector<ModelJoint::ConstPtr> _joints_mdl_const;
    std::vector<RobotJoint::Ptr> _joints_rob;
    std::vector<RobotJoint::ConstPtr> _joints_rob_const;

    // chains
    std::map<std::string, Chain::Ptr> _chain_map;
    std::vector<Chain::Ptr> _chains_xbi;
    std::vector<Chain::ConstPtr> _chains_xbi_const;
    std::vector<std::string> _chain_names;

    // sensors
    std::map<std::string, Sensor::Ptr> _sensor_map;

    // imu
    std::map<std::string, ImuSensor::Ptr> _imu_map;

    // ft
    std::map<std::string, ForceTorqueSensor::Ptr> _ft_map;

    struct Temporaries
    {
        Eigen::MatrixXd J, J1, Jarg;
        Eigen::VectorXd q, v;


        // inertia matrix ldlt
        bool ldlt_dirty = true;
        Eigen::LDLT<Eigen::MatrixXd> ldlt;
        Eigen::MatrixXd M;

        void setZero(int nq, int nv);
        void setDirty();
    };

    Temporaries _tmp;




};

}

#endif // XBOTINTERFACE2_HXX
