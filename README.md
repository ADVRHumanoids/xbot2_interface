# xbot2_interface

This package replaces the good ole' [XBotInterface](https://github.com/advrhumanoids/xbotinterface) 
package by HHCM.
We try to keep the API as close as possible to the old one. 
This is not always possible since this package supports non-Euclidean joints, 
and therefore the number of joints is 
in general different than the number of element in the `q` vector, 
which is also different than the number of elements in the `v` vector.



## Quick guide

### Build the library

After installing the required dependencies (TBD)

```c++
mkdir build && cd build
cmake ..
make
make install
```

### Use the library in your project
```cmake
find_package(xbot2_interface REQUIRED)
target_link_libraries(mytarget xbot2_interface::xbot2_interface)
```

### Model object construction
```c++
std::string urdf;  // read urdf file into this string somehow
auto model = ModelInterface::getModel(urdf, "pin");
```
More constructors are available which take the SRDF file, too.

### Vector dimensions
As we support non-Euclidean joints, care must be taken when manipulating configurations 
and motion vectors.
```c++

// the neutral (i.e., zero) configuration 
// note: this is in general different than Eigen::VectorXd::Zero(model->getNq()) !!
Eigen::VectorXd qn = model->getNeutralQ();

// its size is model->getNq()
assert(qn.size() == model->getNq());

// a random configuration complying with joint limits
Eigen::VectorXd qrand = model->generateRandomQ();

// the velocity vector that brings qn to qrand in unit time
// note: you cannot compute the vector difference of qn and qrand,
// since these are NOT vectors in the linear-algebraic sense!
Eigen::VectorXd vrand = model->difference(qrand, qn);

// its size is model->getNv()
assert(vrand.size() == model->getNv());

// the configuration obtained by applying a velocity vector to 
// a given initial configuration
// note: this will be the same configuration as qrand
// note: you can sum two motions, but you cannot sum a motion with
// a configuration
Eigen::VectorXd q1 = model->sum(qn, vrand);  

// the list of joint names
auto jnames = model->getJointNames();

// its length is model->getJointNum()
assert(jnames.size() == model->getJointNum());

// the i-th joint can have nq and/or nv greater than one;
// its index inside a configuration or motion is therefore != id
int id = jnames.size()/2;

// this data structure contains indexing information
auto jinfo = model->getJointInfo(i);

std::cout <<  "id = " << jinfo.id <<
             " iq = " << jinfo.iq <<
             " iv = " << jinfo.iv <<
             " nq = " << jinfo.nq <<
             " nv = " << jinfo.nv << std::endl;
             
// this is how you index a configuration or motion
auto qj = qrand.segment(jinfo.iq, jinfo.nq);
auto vj = vrand.segment(jinfo.iv, jinfo.nv);

// you can use the joint interface (same result);
Joint::Ptr joint = model->getJoint(i);
auto qj_1 = joint->getJointPosition(); // same as qj
auto vj_1 = joint->getJointVelocity(); // same as vj

// similar for effort, acceleration, ...

```


### Using the model to perform computations
```c++
// every time you change the model state, 
// call update() to apply the new configuration
model->setJointPosition(qrand);
model->setJointVelocity(vrand);
model->update();

// forward kinematics
Eigen::Affine3d T_1 = model->getPose("my_link");
Eigen::Affine3d T_12 = model->getPose("my_link", "my_base_link");
Eigen::Vector6d v_1 = model->getVelocityTwist("my_link");
Eigen::Vector6d v_12 = model->getRelativeVelocityTwist("my_link", "my_base_link");

// similar for acceleration...

// jacobians
Eigen::MatrixXd J;

if(!model->getJacobian("my_link", J))
{
    std::cerr << "error: bad link name \n";
}
```


### Examples
Check [this small collection](examples) of commented examples

### Porting from XBotInterface v1
Check [the cheatsheet](cheatsheet.md)
