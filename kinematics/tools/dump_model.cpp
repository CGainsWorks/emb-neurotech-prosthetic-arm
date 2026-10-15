// Emit the arm model as JSON.
//
// This is the bridge between the C++ library and everything downstream:
// scripts/generate_urdf.py reads it to write the ROS 2 description and
// ros2_ws/src/arm_description has a test that regenerates the URDF from this
// output and fails if it differs from the committed one. Change a link length
// in kinematics/src/model.cpp and the URDF follows or CI stops you.
#include <cstdio>
#include <string>
#include <vector>

#include "arm_kinematics/dynamics.hpp"
#include "arm_kinematics/model.hpp"
#include "json.hpp"

int main() {
    const ak::ArmModel model = ak::a1_arm();

    std::string joints;
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        tools::Object joint(4);
        joint.number_field("index", static_cast<double>(i + 1));
        joint.string_field("name", model.joint_names[i]);
        joint.number_field("theta_offset", model.dh[i].theta_offset);
        joint.number_field("d", model.dh[i].d);
        joint.number_field("a", model.dh[i].a);
        joint.number_field("alpha", model.dh[i].alpha);
        joint.number_field("lower", model.limits[i].lower);
        joint.number_field("upper", model.limits[i].upper);
        joint.number_field("velocity", model.limits[i].velocity);
        joint.number_field("effort", model.limits[i].effort);

        const ak::LinkInertia& link = model.links[i];
        joint.number_field("mass", link.mass);
        joint.field("com", tools::array({link.com.x, link.com.y, link.com.z}));
        joint.field("inertia", tools::array({link.inertia[0], link.inertia[1], link.inertia[2],
                                             link.inertia[3], link.inertia[4], link.inertia[5]}));
        if (i != 0) joints += ",\n";
        joints += tools::Object::pad(4) + joint.str();
    }

    tools::Object tool(4);
    tool.number_field("mass", model.tool.mass);
    tool.field("com", tools::array({model.tool.com.x, model.tool.com.y, model.tool.com.z}));
    tool.field("inertia",
               tools::array({model.tool.inertia[0], model.tool.inertia[1], model.tool.inertia[2],
                             model.tool.inertia[3], model.tool.inertia[4], model.tool.inertia[5]}));

    double total_mass = model.tool.mass;
    for (const auto& link : model.links) total_mass += link.mass;

    tools::Object root;
    root.string_field("model", "A1 six-axis prosthetic arm");
    root.string_field("dh_convention", "standard (distal)");
    root.number_field("joints", static_cast<double>(ak::kNumJoints));
    root.bool_field("spherical_wrist", model.has_spherical_wrist());
    root.number_field("total_mass_kg", total_mass);
    root.field("gravity", tools::array({model.gravity.x, model.gravity.y, model.gravity.z}));
    root.field("chain", "[\n" + joints + "\n" + tools::Object::pad(2) + "]");
    root.field("tool", tool.str());

    std::printf("%s\n", root.str().c_str());
    return 0;
}
