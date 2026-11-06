#include "debug_entity_list.h"

namespace mirena
{

  DebugEntityListDisplay::DebugEntityListDisplay()
  {
    this->_enable_color_override_p = new rviz_common::properties::BoolProperty("Enable Color Override", false, "meow", this);
    this->_color_override_p = new rviz_common::properties::ColorProperty("Entity Color Override", QColor(0, 0, 0, 1), "meowmeow", this, SLOT(update_color_override()));
  }

  DebugEntityListDisplay::~DebugEntityListDisplay()
  {
    clear_entities();
  }

  void DebugEntityListDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    this->_parent_node = this->scene_node_->createChildSceneNode();
  }

  std::shared_ptr<rviz_rendering::Shape> DebugEntityListDisplay::render_entity(const mirena_common::msg::Entity &msg)
  {
    auto cone = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cylinder, this->scene_manager_, this->_parent_node);
    cone->setOrientation(Ogre::Quaternion(std::sqrt(0.5), std::sqrt(0.5), 0, 0));
    cone->setScale(Ogre::Vector3(0.2, 0.4, 0.2));
    cone->setColor(202, 176, 190, 0.4);
    cone->setPosition(Ogre::Vector3(msg.position.x, msg.position.y, msg.position.z));

    if (this->_enable_color_override_p->getBool())
    {
      cone->setColor(this->_color_override_p->getOgreColor());
    }
    else
    {
      float r = 0.7f, g = 0.7f, b = 0.7f; // Default grey

      if (msg.type == "cone_blue")
      {
        r = 49.0f / 255.0f;
        g = 29.0f / 255.0f;
        b = 135.0f / 255.0f;
      }
      else if (msg.type == "cone_yellow")
      {
        r = 189.0f / 255.0f;
        g = 196.0f / 255.0f;
        b = 20.0f / 255.0f;
      }
      else if (msg.type == "cone_big_orange")
      {
        r = 254.0f / 255.0f;
        g = 110.0f / 255.0f;
        b = 20.0f / 255.0f;
        cone->setScale(Ogre::Vector3(0.3, 0.6, 0.3)); // Make it beefy
      }
      else if (msg.type == "cone_orange")
      {
        r = 255.0f / 255.0f;
        g = 110.0f / 255.0f;
        b = 0.0f / 255.0f;
      }
      cone->setColor(r, g, b, 0.7);
    }
    return cone;
  }

  void DebugEntityListDisplay::update_transform(const std_msgs::msg::Header &header)
  {
    // See EntityListDisplay: the frame manager's buffer is the one that gets
    // cleared when sim time restarts at zero, a display-owned one is not.
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!this->context_->getFrameManager()->getTransform(header, position, orientation))
    {
      setMissingTransformToFixedFrame(header.frame_id);
      return;
    }
    setTransformOk();

    this->_parent_node->setPosition(position);
    this->_parent_node->setOrientation(orientation);

    return;
  }

  void DebugEntityListDisplay::clear_entities()
  {
    this->_parent_node->removeAllChildren();
    this->_meshes.clear();
  }

  void DebugEntityListDisplay::processMessage(mirena_common::msg::DebugEntityList::ConstSharedPtr msg)
  {
    this->clear_entities();

    this->update_transform(msg->header);

    for (const auto &entity_msg : msg->entities)
    {
      auto mesh = this->render_entity(entity_msg.ent);
      this->_meshes.push_back(mesh);
    }
  }

  void DebugEntityListDisplay::update_color_override()
  {
    if (!this->_enable_color_override_p->getBool())
    {
      return;
    }
    for (auto &entity : this->_meshes)
    {
      entity->setColor(this->_color_override_p->getOgreColor());
    }
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mirena::DebugEntityListDisplay, rviz_common::Display)