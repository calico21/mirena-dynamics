#pragma once

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_rendering/objects/shape.hpp>

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

#include <mirena_common/msg/entity_list.hpp>

#include <math.h>

namespace mirena
{
  class EntityListDisplay : public rviz_common::MessageFilterDisplay<mirena_common::msg::EntityList>
  {
    Q_OBJECT

    std::vector<std::shared_ptr<rviz_rendering::Shape>> _meshes;
    Ogre::SceneNode *_parent_node;

    rviz_common::properties::BoolProperty *_enable_color_override_p;
    rviz_common::properties::ColorProperty *_color_override_p;

  public:
    EntityListDisplay();
    ~EntityListDisplay() override;

  protected:
    void onInitialize() override;
    void processMessage(mirena_common::msg::EntityList::ConstSharedPtr msg) override;

  private:
    std::shared_ptr<rviz_rendering::Shape> render_entity(const mirena_common::msg::Entity &msg);
    void update_transform(const std_msgs::msg::Header &header);
    void clear_entities();

  private Q_SLOTS:
    void update_color_override();

  };
}