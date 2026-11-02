#include <xbot2_interface/common/utils.h>
#include <urdf_parser/urdf_parser.h>
#include <gtest/gtest.h>

TEST(UrdfExport, RoundTripModelWithModernGeometryAndLimits)
{
    const auto original = urdf::parseURDF(R"(
<robot name="export_test" version="1.2">
  <material name="paint"><color rgba="0.1 0.2 0.3 1"/></material>
  <link name="base">
    <inertial><origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>
      <mass value="4"/><inertia ixx="1" ixy="0" ixz="0" iyy="2" iyz="0" izz="3"/>
    </inertial>
    <visual><geometry><box size="1 2 3"/></geometry><material name="paint"/></visual>
    <visual><geometry><mesh filename="package://test/mesh.stl" scale="2 3 4"/></geometry></visual>
    <collision><geometry><sphere radius="0.4"/></geometry></collision>
    <collision><geometry><cylinder radius="0.2" length="0.8"/></geometry></collision>
  </link>
  <link name="tip"><collision><geometry><capsule radius="0.3" length="0.7"/></geometry></collision></link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="tip"/><axis xyz="0 1 0"/>
    <origin xyz="0.1 0.2 0.3" rpy="0.2 0.1 0.3"/>
    <limit lower="-1" upper="2" effort="3" velocity="4" acceleration="5" deceleration="6" jerk="7"/>
    <dynamics damping="0.5" friction="0.6"/>
    <safety_controller soft_lower_limit="-0.8" soft_upper_limit="1.8" k_position="2" k_velocity="3"/>
    <calibration rising="0.2" falling="0.4"/>
  </joint>
  <link name="follower"/>
  <joint name="mimic" type="continuous">
    <parent link="tip"/><child link="follower"/>
    <limit effort="2" velocity="3"/><mimic joint="hinge" multiplier="2" offset="0.1"/>
  </joint>
</robot>)");
    ASSERT_TRUE(original);
    const auto result = urdf::parseURDF(XBot::Utils::urdfToString(*original));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->getName(), original->getName());
    EXPECT_EQ(result->links_.size(), original->links_.size());
    EXPECT_EQ(result->joints_.size(), original->joints_.size());
    const auto base = result->getLink("base");
    ASSERT_TRUE(base->inertial);
    EXPECT_DOUBLE_EQ(base->inertial->mass, 4);
    EXPECT_DOUBLE_EQ(base->inertial->origin.position.z, 3);
    ASSERT_EQ(base->visual_array.size(), 2u);
    EXPECT_EQ(base->visual_array[0]->geometry->type, urdf::Geometry::BOX);
    EXPECT_EQ(base->visual_array[0]->material_name, "paint");
    EXPECT_DOUBLE_EQ(result->materials_.at("paint")->color.g, 0.2f);
    const auto mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(base->visual_array[1]->geometry);
    ASSERT_TRUE(mesh);
    EXPECT_EQ(mesh->filename, "package://test/mesh.stl");
    EXPECT_DOUBLE_EQ(mesh->scale.y, 3);
    ASSERT_EQ(base->collision_array.size(), 2u);
    EXPECT_EQ(base->collision_array[0]->geometry->type, urdf::Geometry::SPHERE);
    EXPECT_EQ(base->collision_array[1]->geometry->type, urdf::Geometry::CYLINDER);
    const auto capsule = urdf::dynamic_pointer_cast<urdf::Capsule>(result->getLink("tip")->collision->geometry);
    ASSERT_TRUE(capsule);
    EXPECT_DOUBLE_EQ(capsule->radius, 0.3);
    const auto joint = result->getJoint("hinge");
    ASSERT_TRUE(joint->limits);
    EXPECT_DOUBLE_EQ(joint->limits->acceleration, 5);
    EXPECT_DOUBLE_EQ(joint->limits->deceleration, 6);
    EXPECT_DOUBLE_EQ(joint->limits->jerk, 7);
    EXPECT_DOUBLE_EQ(joint->axis.y, 1);
    ASSERT_TRUE(joint->dynamics);
    EXPECT_DOUBLE_EQ(joint->dynamics->friction, 0.6);
    ASSERT_TRUE(joint->safety);
    EXPECT_DOUBLE_EQ(joint->safety->soft_upper_limit, 1.8);
    ASSERT_TRUE(joint->calibration);
    ASSERT_TRUE(joint->calibration->rising);
    EXPECT_DOUBLE_EQ(*joint->calibration->rising, 0.2);
    const auto mimic = result->getJoint("mimic")->mimic;
    ASSERT_TRUE(mimic);
    EXPECT_EQ(mimic->joint_name, "hinge");
    EXPECT_DOUBLE_EQ(mimic->multiplier, 2);
    EXPECT_DOUBLE_EQ(mimic->offset, 0.1);
}
