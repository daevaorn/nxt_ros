#include <rviz/ogre_helpers/shape.h>

//#include <OGRE/OgreSceneNode.h>
//#include <OGRE/OgreSceneManager.h>

#include "nxt_color_display.h"

namespace nxt_rviz_plugin
{
NXTColorDisplay::NXTColorDisplay()
{
  alpha_property_ = new rviz::FloatProperty( "Alpha", .5f,
                                             "Amount of transparency to apply to the circle. "
					     "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateCylinder() ));

  display_length_property_ = new rviz::FloatProperty( "Display Length", .003f,
                                                      "",
                                                      this, SLOT( updateCylinder() ));

  cylinder_ = new rviz::Shape(rviz::Shape::Cylinder, context_->getSceneManager(), scene_node_);

  Ogre::Vector3 scale( 0, 0, 0 );
  cylinder_->setScale( scale );
}

void NXTColorDisplay::updateCylinder()
{
}

void NXTColorDisplay::processMessage(const nxt_msgs::Color::ConstPtr& msg)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  float display_length = display_length_property_->getFloat();

  geometry_msgs::Pose pose;
  pose.position.z = -0.0033;
  pose.position.y = 0;
  pose.position.x = 0.0185 + display_length / 2;
  pose.orientation.x = 0.707;
  pose.orientation.z = -0.707;
  if (!context_->getFrameManager()->transform(msg->header, pose, position, orientation))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
               qPrintable( fixed_frame_ ) );
  }

  cylinder_->setPosition(position);
  cylinder_->setOrientation(orientation);
  Ogre::Vector3 scale( 0.0155, 0.0155, display_length);
  cylinder_->setScale(scale);
  cylinder_->setColor(msg->r, msg->g, msg->b, alpha_property_->getFloat());
}

} // namespace nxt_rviz_plugin  


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nxt_rviz_plugin::NXTColorDisplay, rviz::Display) 
