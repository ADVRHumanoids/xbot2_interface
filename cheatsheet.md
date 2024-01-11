# Cheat-sheet 

## Quick start


## Porting

| v1       | v2      | notes |
| -------- | ------- | ------- |
| `#include <XBotInterface/ModelInterface.h>`  | `#include <xbot2_interface/xbotinterface2.h>`    | |
| `#include <XBotInterface/RobotInterface.h>`  | `#include <xbot2_interface/robotinterface2.h>`    | |
| `#include <XBotInterface/Utils.h>`  | `#include <xbot2_interface/common/utils.h>`    | |
| `m->getEnabledJointNames()`  | `m->getJointNames()`    | |
| `m->getJointNum()` | `m->getJointNum()` OR `m->getNv()` OR `m->getNq()`   | `m->getJointNum()` returns the number of joints;  `m->getNv()` returns the number of DoFs, i.e. the size of a motion or effort vector; `m->getNq()` returns the size of a configuration vector |
| `m->eigenToMap(x, xmap)`    | `m->eigenToQ(q, qmap)` OR `m->eigenToV(v, vmap)`    | Depends on whether `x` is a configuration or a motion/effort  |
| `q += qdot*dt` | `q = m->sum(q, v*dt)` |  |
| `qdot = (q1 - q0)/dt` | `qdot = m->difference(q1, q0)/dt` | |
| `m->syncFrom(*m1, Sync::Position, Sync::Velocity)` | `m->syncFrom(*m1, ControlMode::POSITION\|ControlMode::VELOCITY` | |
| `m->syncFrom(*r, Sync::Position, Sync::Velocity, Sync::MotorSide)` | `m->syncFrom(*r, ControlMode::POSITION\|ControlMode::VELOCITY, Sync::MotorSide)` | |
| `m->getInertiaMatrix(M)` | `m->computeInertiaMatrix(M)` | | 
| `m->getCentroidalMomentumMatrix(A)` | `m->computeCentroidalMomentumMatrix(A)` | | 
| `m->getVelocityTwist("distal", "base", v)` | `m->getRelativeVelocityTwist("distal", "base", v)` | | 
| `m->getJdotQdot(...)` | `m->getJdotTimesV(...)` | | 
