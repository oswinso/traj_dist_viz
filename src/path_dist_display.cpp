#include "traj_dist_viz/path_dist_display.h"

#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreSceneManager.h"
#include "OGRE/OgreSceneNode.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/validate_floats.h"
#include "rviz/visualization_manager.h"

namespace traj_dist_viz {
PathDistDisplay::PathDistDisplay() {
    color_property_ =
        new rviz::ColorProperty("Color", QColor(25, 255, 0), "Color to draw the path.", this);
    alpha_property_ =
        new rviz::FloatProperty("Alpha", 1.0, "Amount of transparency to apply to the path.", this);

    buffer_length_property_ = new rviz::IntProperty(
        "Buffer Length", 1, "Number of paths to display.", this, SLOT(updateBufferLength()));
    buffer_length_property_->setMin(1);
}

PathDistDisplay::~PathDistDisplay() { destroyObjects(); }

void PathDistDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateBufferLength();
}

void PathDistDisplay::reset() {
    MFDClass::reset();
    updateBufferLength();
}

bool validate_floats(const nav_msgs::Path &msg) { return rviz::validateFloats(msg.poses); }

bool validate_floats(const TrajDist &msg) {
    bool valid = true;
    for (const nav_msgs::Path &path : msg.paths) {
        valid &= validate_floats(path);
    }
    return valid;
}

void PathDistDisplay::destroyObjects() {
    // Destroy all simple lines, if any
    for (auto &manual_object : manual_objects_) {
        if (manual_object) {
            manual_object->clear();
            scene_manager_->destroyManualObject(manual_object);
            manual_object = nullptr;  // ensure it doesn't get destroyed again
        }
    }
}

void PathDistDisplay::updateBufferLength() {
    destroyObjects();

    int buffer_length = buffer_length_property_->getInt();
    manual_objects_.resize(buffer_length);
    for (size_t i = 0; i < manual_objects_.size(); i++) {
        Ogre::ManualObject *manual_object = scene_manager_->createManualObject();
        manual_object->setDynamic(true);
        scene_node_->attachObject(manual_object);
        manual_objects_[i] = manual_object;
    }
}

void set_manual_object_vertices(Ogre::ManualObject &manual_object,
                                const nav_msgs::Path &path,
                                const Ogre::Matrix4 &transform,
                                Ogre::ColourValue color) {
    for (uint32_t i = 0; i < path.poses.size(); ++i) {
        const geometry_msgs::Point &pos = path.poses[i].pose.position;
        const Ogre::Vector3 xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);
        manual_object.position(xpos.x, xpos.y, xpos.z);
        manual_object.colour(color);
    }
}

void PathDistDisplay::processMessage(const TrajDist::ConstPtr &msg) {
    size_t buffer_idx = 0;
    // TODO: proper buffer.
    Ogre::ManualObject *manual_object = manual_objects_.at(buffer_idx);

    if (!validate_floats(*msg)) {
        setStatus(rviz::StatusProperty::Error,
                  "Topic",
                  "Message contained invalid floating point values (nans or infs)");
        return;
    }

    // Lookup transform into fixed frame.
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                  msg->header.frame_id.c_str(),
                  qPrintable(fixed_frame_));
    }

    Ogre::Matrix4 transform(orientation);
    transform.setTrans(position);

    // Somehow alpha doesn't work with ManualObject lines... but oh well, Path also has this problem.
    Ogre::ColourValue color = color_property_->getOgreColor();
    color.a = alpha_property_->getFloat();

    size_t total_points = 0;
    for (const auto &path_msg : msg->paths) {
        total_points += path_msg.poses.size();
    }
    manual_object->estimateVertexCount(total_points);

    const size_t n_paths = msg->paths.size();
    const size_t n_sections = manual_object->getNumSections();

    // Reuse sections.
    const size_t n_reused_sections = std::min<size_t>(n_paths, n_sections);
    for (size_t ii = 0; ii < n_reused_sections; ii++) {
        manual_object->beginUpdate(ii);
        set_manual_object_vertices(*manual_object, msg->paths[ii], transform, color);
        manual_object->end();
    }

    // Create new sections if there aren't enough.
    constexpr auto kMaterial = "BaseWhiteNoLighting";
    constexpr auto kOpType = Ogre::RenderOperation::OT_LINE_STRIP;
    const auto kResourceGroupName = Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME;
    const size_t n_new_sections = n_paths > n_sections ? n_paths - n_sections : 0;
    for (size_t ii = 0; ii < n_new_sections; ii++) {
        manual_object->begin(kMaterial, kOpType, kResourceGroupName);
        set_manual_object_vertices(*manual_object, msg->paths[ii], transform, color);
        manual_object->end();
    }

    // Delete sections if there are too many.
    const size_t n_delete_sections = n_sections > n_paths ? n_sections - n_paths : 0;
    for (size_t ii = 0; ii < n_delete_sections; ii++) {
        // No positions = delete.
        manual_object->beginUpdate(n_paths + ii);
        manual_object->end();
    }

    context_->queueRender();
}

}  // namespace traj_dist_viz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(traj_dist_viz::PathDistDisplay, rviz::Display)
// END_TUTORIAL
