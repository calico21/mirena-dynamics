#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/mesh_shape.hpp>

#include "mirena_common/msg/flat_pose_dist.hpp"

#include <iostream>
#define BELL_Z_SCALE 15
#define BELL_Z_ANALYTIC_MAX 0.16

namespace mirena
{

  class FlatPoseDistDisplay : public rviz_common::MessageFilterDisplay<mirena_common::msg::FlatPoseDist>
  {
  public:
    FlatPoseDistDisplay() {}

    ~FlatPoseDistDisplay() override
    {
      if (bell_shape_){
        bell_shape_.reset();
      }

      if (direction_cone_)
      {
        direction_cone_.reset();
      }
    }

  protected:
    void onInitialize() override
    {
      MFDClass::onInitialize();

      this->main_node_ = this->scene_node_->createChildSceneNode();
      this->direction_cone_node_= this->main_node_->createChildSceneNode();
      
      direction_cone_ = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cone, scene_manager_, this->direction_cone_node_);
      direction_cone_->setScale(Ogre::Vector3(0.5, 1, 0.5)); // Radio 0.5, altura 1.0
      direction_cone_->setColor(1.0, 0.0, 0.0, 1); // Color rojo
      direction_cone_->setPosition(Ogre::Vector3(0, 0, BELL_Z_SCALE*BELL_Z_ANALYTIC_MAX));
      direction_cone_->setOrientation(Ogre::Quaternion(Ogre::Radian(-M_PI_2), Ogre::Vector3(0, 0, 1)));

      this->bell_shape_ = this->createNormalizedGaussianMesh(this->scene_manager_, this->main_node_);
      this->bell_shape_->setColor(163, 170, 190, 0.5); /* Gray, slightly transparent*/
      this->bell_shape_->setScale(Ogre::Vector3(1, 1, BELL_Z_SCALE));


    }

    void processMessage(mirena_common::msg::FlatPoseDist::ConstSharedPtr msg) override
    {
      // Transformar la posición del coche al frame fijo de RViz2. Se usa el
      // frame manager de RViz y no un buffer propio: es el que se vacía cuando
      // el reloj salta hacia atrás, que es lo que ocurre cada vez que se
      // reinicia el simulador y el tiempo de simulación vuelve a cero.
      Ogre::Vector3 origin_position;
      Ogre::Quaternion origin_orientation;
      if (!this->context_->getFrameManager()->getTransform(msg->header, origin_position, origin_orientation))
      {
        setMissingTransformToFixedFrame(msg->header.frame_id);
        return;
      }
      setTransformOk();

      Ogre::Vector3 bell_position(msg->mean.at(0), msg->mean.at(1), 0);
      Ogre::Quaternion bell_orientation(Ogre::Radian(0), Ogre::Vector3::UNIT_Z);

      // Combinar posición y orientación
      Ogre::Vector3 position = origin_position + (origin_orientation * bell_position);
      Ogre::Quaternion orientation = origin_orientation * bell_orientation;

      this->main_node_->setPosition(position);
      this->main_node_->setOrientation(orientation);

      this->bell_shape_->setScale(Ogre::Vector3(msg->covariance.at(0),msg->covariance.at(4), BELL_Z_SCALE));
      this->direction_cone_node_->setOrientation(Ogre::Quaternion(Ogre::Radian(msg->mean.at(2)), Ogre::Vector3::UNIT_Z));
    }

  private:
    Ogre::SceneNode *main_node_;
    Ogre::SceneNode *direction_cone_node_;

    std::shared_ptr<rviz_rendering::MeshShape> bell_shape_;
    std::shared_ptr<rviz_rendering::Shape> direction_cone_; 

    // Create a mesh of a 2-variable N(0, I)
    std::shared_ptr<rviz_rendering::MeshShape> createNormalizedGaussianMesh(
        Ogre::SceneManager *scene_manager,
        Ogre::SceneNode *parent_node
      )
    {
      const int resolution = 50;
      const double range = 2.0;
  
      double dx = (2 * range) / resolution;
      double dy = (2 * range) / resolution;
  
      std::shared_ptr<rviz_rendering::MeshShape> mesh =
          std::make_shared<rviz_rendering::MeshShape>(scene_manager, parent_node);
  
      std::vector<std::vector<int>> vertex_indices(resolution + 1, std::vector<int>(resolution + 1));
  
      mesh->estimateVertexCount((resolution+1)*(resolution+1));

      // Insert vertices
      int vertex_index = 0;
      for (int i = 0; i <= resolution; ++i) {
          for (int j = 0; j <= resolution; ++j) {
              double x = -range + i * dx;
              double y = -range + j * dy;
              double exponent = -0.5 * (x * x + y * y);
              double z = (1.0 / (2.0 * M_PI)) * std::exp(exponent);
  
              mesh->addVertex(Ogre::Vector3(x, y, z));
              vertex_indices[i][j] = vertex_index++;
          }
      }
      mesh->beginTriangles();
  
      // Add triangles
      for (int i = 0; i < resolution; ++i) {
          for (int j = 0; j < resolution; ++j) {
              int v0 = vertex_indices[i][j];
              int v1 = vertex_indices[i + 1][j];
              int v2 = vertex_indices[i][j + 1];
              int v3 = vertex_indices[i + 1][j + 1];
  
              // Triangle 1
              mesh->addTriangle(v0, v1, v2);
  
              // Triangle 2
              mesh->addTriangle(v1, v3, v2);
          }
      }
      mesh->endTriangles();

      return mesh;
    }
  };

} // namespace mirena

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mirena::FlatPoseDistDisplay, rviz_common::Display);
