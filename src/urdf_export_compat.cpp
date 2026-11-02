/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

// Export-only portions of ros/urdfdom 4.0.1, adapted for urdfdom >= 5.
// Kept private to XBot; no replacement urdf symbols are exported.
#include "urdf_export_compat.h"
#include <urdf_parser/urdf_parser.h>
#include <console_bridge/console.h>
#include <stdexcept>
#include <cmath>

namespace XBot { namespace detail { namespace {
using namespace urdf;
bool exportPose(Pose &pose, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement* origin = xml->GetDocument()->NewElement("origin");
  std::string pose_xyz_str = urdf_export_helpers::values2str(pose.position);
  std::string pose_rpy_str = urdf_export_helpers::values2str(pose.rotation);
  origin->SetAttribute("xyz", pose_xyz_str.c_str());
  origin->SetAttribute("rpy", pose_rpy_str.c_str());
  xml->LinkEndChild(origin);
  return true;
}

bool exportMaterial(Material &material, tinyxml2::XMLElement *xml)
{
  tinyxml2::XMLElement* material_xml = xml->GetDocument()->NewElement("material");
  material_xml->SetAttribute("name", material.name.c_str());

  tinyxml2::XMLElement* texture = material_xml->GetDocument()->NewElement("texture");
  if (!material.texture_filename.empty())
    texture->SetAttribute("filename", material.texture_filename.c_str());
  material_xml->LinkEndChild(texture);

  tinyxml2::XMLElement* color = material_xml->GetDocument()->NewElement("color");
  color->SetAttribute("rgba", urdf_export_helpers::values2str(material.color).c_str());
  material_xml->LinkEndChild(color);
  xml->LinkEndChild(material_xml);
  return true;
}

bool exportSphere(Sphere &s, tinyxml2::XMLElement *xml)
{
  // e.g. add <sphere radius="1"/>
  tinyxml2::XMLElement *sphere_xml = xml->GetDocument()->NewElement("sphere");
  sphere_xml->SetAttribute("radius", urdf_export_helpers::values2str(s.radius).c_str());
  xml->LinkEndChild(sphere_xml);
  return true;
}

bool exportBox(Box &b, tinyxml2::XMLElement *xml)
{
  // e.g. add <box size="1 1 1"/>
  tinyxml2::XMLElement *box_xml = xml->GetDocument()->NewElement("box");
  box_xml->SetAttribute("size", urdf_export_helpers::values2str(b.dim).c_str());
  xml->LinkEndChild(box_xml);
  return true;
}

bool exportCylinder(Cylinder &y, tinyxml2::XMLElement *xml)
{
  // e.g. add <cylinder radius="1"/>
  tinyxml2::XMLElement *cylinder_xml = xml->GetDocument()->NewElement("cylinder");
  cylinder_xml->SetAttribute("radius", urdf_export_helpers::values2str(y.radius).c_str());
  cylinder_xml->SetAttribute("length", urdf_export_helpers::values2str(y.length).c_str());
  xml->LinkEndChild(cylinder_xml);
  return true;
}

bool exportMesh(Mesh &m, tinyxml2::XMLElement *xml)
{
  // e.g. add <mesh filename="my_file" scale="1 1 1"/>
  tinyxml2::XMLElement *mesh_xml = xml->GetDocument()->NewElement("mesh");
  if (!m.filename.empty())
    mesh_xml->SetAttribute("filename", m.filename.c_str());
  mesh_xml->SetAttribute("scale", urdf_export_helpers::values2str(m.scale).c_str());
  xml->LinkEndChild(mesh_xml);
  return true;
}

bool exportGeometry(GeometrySharedPtr &geom, tinyxml2::XMLElement *xml)
{
  tinyxml2::XMLElement *geometry_xml = xml->GetDocument()->NewElement("geometry");
  if (urdf::dynamic_pointer_cast<Sphere>(geom))
  {
    exportSphere((*(urdf::dynamic_pointer_cast<Sphere>(geom).get())), geometry_xml);
  }
  else if (urdf::dynamic_pointer_cast<Box>(geom))
  {
    exportBox((*(urdf::dynamic_pointer_cast<Box>(geom).get())), geometry_xml);
  }
  else if (urdf::dynamic_pointer_cast<Cylinder>(geom))
  {
    exportCylinder((*(urdf::dynamic_pointer_cast<Cylinder>(geom).get())), geometry_xml);
  }
  else if (urdf::dynamic_pointer_cast<Mesh>(geom))
  {
    exportMesh((*(urdf::dynamic_pointer_cast<Mesh>(geom).get())), geometry_xml);
  }
#ifdef XBOT2IFC_URDFDOM_HAS_CAPSULE
  else if (auto capsule = urdf::dynamic_pointer_cast<Capsule>(geom))
  {
    auto* element = xml->GetDocument()->NewElement("capsule");
    element->SetAttribute("radius", capsule->radius);
    element->SetAttribute("length", capsule->length);
    geometry_xml->LinkEndChild(element);
  }
#endif
  else
  {
    throw std::runtime_error("Cannot export unsupported URDF geometry");
  }

  xml->LinkEndChild(geometry_xml);
  return true;
}

bool exportInertial(Inertial &i, tinyxml2::XMLElement *xml)
{
  // adds <inertial>
  //        <mass value="1"/>
  //        <pose xyz="0 0 0" rpy="0 0 0"/>
  //        <inertia ixx="1" ixy="0" />
  //      </inertial>
  tinyxml2::XMLElement *inertial_xml = xml->GetDocument()->NewElement("inertial");

  tinyxml2::XMLElement *mass_xml = inertial_xml->GetDocument()->NewElement("mass");
  mass_xml->SetAttribute("value", urdf_export_helpers::values2str(i.mass).c_str());
  inertial_xml->LinkEndChild(mass_xml);

  exportPose(i.origin, inertial_xml);

  tinyxml2::XMLElement *inertia_xml = inertial_xml->GetDocument()->NewElement("inertia");
  inertia_xml->SetAttribute("ixx", urdf_export_helpers::values2str(i.ixx).c_str());
  inertia_xml->SetAttribute("ixy", urdf_export_helpers::values2str(i.ixy).c_str());
  inertia_xml->SetAttribute("ixz", urdf_export_helpers::values2str(i.ixz).c_str());
  inertia_xml->SetAttribute("iyy", urdf_export_helpers::values2str(i.iyy).c_str());
  inertia_xml->SetAttribute("iyz", urdf_export_helpers::values2str(i.iyz).c_str());
  inertia_xml->SetAttribute("izz", urdf_export_helpers::values2str(i.izz).c_str());
  inertial_xml->LinkEndChild(inertia_xml);

  xml->LinkEndChild(inertial_xml);

  return true;
}

bool exportVisual(Visual &vis, tinyxml2::XMLElement *xml)
{
  // <visual group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </visual>
  tinyxml2::XMLElement * visual_xml = xml->GetDocument()->NewElement("visual");

  exportPose(vis.origin, visual_xml);

  exportGeometry(vis.geometry, visual_xml);

  if (vis.material)
    exportMaterial(*vis.material, visual_xml);

  xml->LinkEndChild(visual_xml);

  return true;
}

bool exportCollision(Collision &col, tinyxml2::XMLElement* xml)
{
  // <collision group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </collision>
  tinyxml2::XMLElement * collision_xml = xml->GetDocument()->NewElement("collision");

  exportPose(col.origin, collision_xml);

  exportGeometry(col.geometry, collision_xml);

  xml->LinkEndChild(collision_xml);

  return true;
}

bool exportLink(Link &link, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement * link_xml = xml->GetDocument()->NewElement("link");
  link_xml->SetAttribute("name", link.name.c_str());

  if (link.inertial)
    exportInertial(*link.inertial, link_xml);
  for (std::size_t i = 0 ; i < link.visual_array.size() ; ++i)
    exportVisual(*link.visual_array[i], link_xml);
  for (std::size_t i = 0 ; i < link.collision_array.size() ; ++i)
    exportCollision(*link.collision_array[i], link_xml);

  xml->LinkEndChild(link_xml);

  return true;
}

bool exportJointDynamics(JointDynamics &jd, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement *dynamics_xml = xml->GetDocument()->NewElement("dynamics");
  dynamics_xml->SetAttribute("damping", urdf_export_helpers::values2str(jd.damping).c_str() );
  dynamics_xml->SetAttribute("friction", urdf_export_helpers::values2str(jd.friction).c_str() );
  xml->LinkEndChild(dynamics_xml);
  return true;
}

bool exportJointLimits(JointLimits &jl, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement *limit_xml = xml->GetDocument()->NewElement("limit");
  // Modern urdfdom uses infinity for unspecified limits, but rejects the
  // string "inf" when parsing XML. Omit those attributes to retain defaults.
  auto setLimit = [limit_xml](const char* name, double value) {
    if(std::isfinite(value)) limit_xml->SetAttribute(name, value);
  };
  setLimit("effort", jl.effort);
  setLimit("velocity", jl.velocity);
  setLimit("lower", jl.lower);
  setLimit("upper", jl.upper);
#ifdef XBOT2IFC_URDFDOM_HAS_EXTENDED_LIMITS
  setLimit("acceleration", jl.acceleration);
  setLimit("deceleration", jl.deceleration);
  setLimit("jerk", jl.jerk);
#endif
  xml->LinkEndChild(limit_xml);
  return true;
}

bool exportJointSafety(JointSafety &js, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement *safety_xml = xml->GetDocument()->NewElement("safety_controller");
  safety_xml->SetAttribute("k_position", urdf_export_helpers::values2str(js.k_position).c_str());
  safety_xml->SetAttribute("k_velocity", urdf_export_helpers::values2str(js.k_velocity).c_str());
  safety_xml->SetAttribute("soft_lower_limit", urdf_export_helpers::values2str(js.soft_lower_limit).c_str());
  safety_xml->SetAttribute("soft_upper_limit", urdf_export_helpers::values2str(js.soft_upper_limit).c_str());
  xml->LinkEndChild(safety_xml);
  return true;
}

bool exportJointCalibration(JointCalibration &jc, tinyxml2::XMLElement* xml)
{
  if (jc.falling || jc.rising)
  {
    tinyxml2::XMLElement *calibration_xml = xml->GetDocument()->NewElement("calibration");
    if (jc.falling)
      calibration_xml->SetAttribute("falling", urdf_export_helpers::values2str(*jc.falling).c_str());
    if (jc.rising)
      calibration_xml->SetAttribute("rising", urdf_export_helpers::values2str(*jc.rising).c_str());
    //calibration_xml->SetAttribute("reference_position", urdf_export_helpers::values2str(jc.reference_position) );
    xml->LinkEndChild(calibration_xml);
  }
  return true;
}

bool exportJointMimic(JointMimic &jm, tinyxml2::XMLElement* xml)
{
  if (!jm.joint_name.empty())
  {
    tinyxml2::XMLElement *mimic_xml = xml->GetDocument()->NewElement("mimic");
    mimic_xml->SetAttribute("offset", urdf_export_helpers::values2str(jm.offset).c_str());
    mimic_xml->SetAttribute("multiplier", urdf_export_helpers::values2str(jm.multiplier).c_str());
    mimic_xml->SetAttribute("joint", jm.joint_name.c_str());
    xml->LinkEndChild(mimic_xml);
  }
  return true;
}

bool exportJoint(Joint &joint, tinyxml2::XMLElement* xml)
{
  tinyxml2::XMLElement * joint_xml = xml->GetDocument()->NewElement("joint");
  joint_xml->SetAttribute("name", joint.name.c_str());
  if (joint.type == urdf::Joint::PLANAR)
    joint_xml->SetAttribute("type", "planar");
  else if (joint.type == urdf::Joint::FLOATING)
    joint_xml->SetAttribute("type", "floating");
  else if (joint.type == urdf::Joint::REVOLUTE)
    joint_xml->SetAttribute("type", "revolute");
  else if (joint.type == urdf::Joint::CONTINUOUS)
    joint_xml->SetAttribute("type", "continuous");
  else if (joint.type == urdf::Joint::PRISMATIC)
    joint_xml->SetAttribute("type", "prismatic");
  else if (joint.type == urdf::Joint::FIXED)
    joint_xml->SetAttribute("type", "fixed");
  else
    CONSOLE_BRIDGE_logError("ERROR:  Joint [%s] type [%d] is not a defined type.\n",joint.name.c_str(), joint.type);

  // origin
  exportPose(joint.parent_to_joint_origin_transform, joint_xml);

  // axis
  tinyxml2::XMLElement * axis_xml = joint_xml->GetDocument()->NewElement("axis");
  axis_xml->SetAttribute("xyz", urdf_export_helpers::values2str(joint.axis).c_str());
  joint_xml->LinkEndChild(axis_xml);

  // parent
  tinyxml2::XMLElement * parent_xml = joint_xml->GetDocument()->NewElement("parent");
  parent_xml->SetAttribute("link", joint.parent_link_name.c_str());
  joint_xml->LinkEndChild(parent_xml);

  // child
  tinyxml2::XMLElement * child_xml = joint_xml->GetDocument()->NewElement("child");
  child_xml->SetAttribute("link", joint.child_link_name.c_str());
  joint_xml->LinkEndChild(child_xml);

  if (joint.dynamics)
    exportJointDynamics(*(joint.dynamics), joint_xml);
  if (joint.limits)
    exportJointLimits(*(joint.limits), joint_xml);
  if (joint.safety)
    exportJointSafety(*(joint.safety), joint_xml);
  if (joint.calibration)
    exportJointCalibration(*(joint.calibration), joint_xml);
  if (joint.mimic)
    exportJointMimic(*(joint.mimic), joint_xml);

  xml->LinkEndChild(joint_xml);
  return true;
}



std::unique_ptr<tinyxml2::XMLDocument> exportURDFInternal(const ModelInterface &model)
{
  auto doc = std::make_unique<tinyxml2::XMLDocument>();

  tinyxml2::XMLElement* robot = doc->NewElement("robot");
  robot->SetAttribute("name", model.name_.c_str());
#ifdef XBOT2IFC_URDFDOM_HAS_EXTENDED_LIMITS
  robot->SetAttribute("version", "1.2");
#elif defined(XBOT2IFC_URDFDOM_HAS_CAPSULE)
  robot->SetAttribute("version", "1.1");
#endif
  doc->LinkEndChild(robot);


  for (std::map<std::string, MaterialSharedPtr>::const_iterator m=model.materials_.begin(); m!=model.materials_.end(); ++m)
  {
    CONSOLE_BRIDGE_logDebug("urdfdom: exporting material [%s]\n",m->second->name.c_str());
    exportMaterial(*(m->second), robot);
  }

  for (std::map<std::string, LinkSharedPtr>::const_iterator l=model.links_.begin(); l!=model.links_.end(); ++l)
  {
    CONSOLE_BRIDGE_logDebug("urdfdom: exporting link [%s]\n",l->second->name.c_str());
    exportLink(*(l->second), robot);
  }

  for (std::map<std::string, JointSharedPtr>::const_iterator j=model.joints_.begin(); j!=model.joints_.end(); ++j)
  {
    CONSOLE_BRIDGE_logDebug("urdfdom: exporting joint [%s]\n",j->second->name.c_str());
    exportJoint(*(j->second), robot);
  }

  return doc;
}

} // namespace

std::unique_ptr<tinyxml2::XMLDocument> exportUrdf(const urdf::ModelInterface& model)
{
    return exportURDFInternal(model);
}
}} // namespace XBot::detail
