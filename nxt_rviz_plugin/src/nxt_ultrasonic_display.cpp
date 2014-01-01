#include <rviz/ogre_helpers/shape.h>

#include "nxt_ultrasonic_display.h"
#include "nxt_ultrasonic_visual.h"

namespace nxt_rviz_plugin
{
NXTUltrasonicDisplay::NXTUltrasonicDisplay()
{
  color_property_ = new rviz::ColorProperty( "Color", QColor( .1f, 1.f, .0f ),
                                             "Color to draw the range.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", .5f,
                                             "Amount of transparency to apply to the range. 0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  visual_.reset(new NXTUltrasonicVisual( context_->getSceneManager(), scene_node_ ));
}

void NXTUltrasonicDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue color = color_property_->getOgreColor();
  float alpha = alpha_property_->getFloat();
  
  visual_->setColor(color.r, color.g, color.b, alpha);
}

void NXTUltrasonicDisplay::processMessage(const nxt_msgs::Range::ConstPtr& msg)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  
  pose.position.z = pose.position.y = 0;
  pose.position.x = msg->range / 2;
  pose.orientation.x = 0.707;
  pose.orientation.z = -0.707;
  
  if (!context_->getFrameManager()->transform(msg->header, pose, position, orientation))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
	       qPrintable(fixed_frame_) );
  }

  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation); 
}

} // namespace nxt_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nxt_rviz_plugin::NXTUltrasonicDisplay, rviz::Display)
