#ifndef JOINT_H
#define JOINT_H

#include <urdf_model/joint.h>

#include <xbot2_interface/types.h>

namespace XBot {

class XBotInterface2;

class Joint
{

public:

    XBOT_DECLARE_SMART_PTR(Joint);

    VecConstRef getJointPosition() const;
    void setJointPosition(VecConstRef q);
    void setJointPosition(double q);

    friend class XBotInterface2;

private:

    class Impl;

    std::unique_ptr<Impl> impl;

};

}

#endif // JOINT_H
