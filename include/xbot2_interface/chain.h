#ifndef XBOT2_IFC_CHAIN_H
#define XBOT2_IFC_CHAIN_H

#include "joint.h"

namespace XBot {

inline namespace v2 {

class XBotInterface;

class ModelChain;

class RobotChain;


class XBOT2IFC_API Chain : public ReadStateInterface<Chain>
{

public:

    XBOT_DECLARE_SMART_PTR(Chain);

    string_const_ref getName() const;

    string_const_ref getBaseLink() const;

    string_const_ref getTipLink() const;

    int getJointNum() const;

    int getQIndex() const;

    int getVIndex() const;

    const std::vector<std::string>& getJointNames() const;

    const std::vector<Joint::Ptr>& getJoints();

    const std::vector<Joint::ConstPtr>& getJoints() const;

    bool hasJoint(string_const_ref jname) const;

    class Impl;

    Chain(std::unique_ptr<Impl>);

    ~Chain();

    friend class XBotInterface;
    friend ReadStateInterface<Chain>;
    friend ReadCmdInterface<RobotChain>;

protected:

    std::unique_ptr<Impl> impl;

};

}}

#endif // CHAIN_H
