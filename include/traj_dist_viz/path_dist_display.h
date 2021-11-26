#pragma once

#ifndef Q_MOC_RUN
#include "rviz/message_filter_display.h"

#include "traj_dist_viz/TrajDist.h"
#endif

namespace Ogre {
class SceneNode;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
}  // namespace rviz

namespace traj_dist_viz {
class PathDistDisplay final : public rviz::MessageFilterDisplay<TrajDist> {
    Q_OBJECT
 public:
    PathDistDisplay();
    ~PathDistDisplay() override;
    void reset() override;

 protected:
    void onInitialize() override;
    void processMessage(const TrajDist::ConstPtr &msg) override;

 private Q_SLOTS:
    void updateBufferLength();

 private:
    void destroyObjects();

    std::vector<Ogre::ManualObject *> manual_objects_;
    rviz::ColorProperty *color_property_;
    rviz::FloatProperty *alpha_property_;
    rviz::IntProperty *buffer_length_property_;
};

}  // namespace traj_dist_viz
