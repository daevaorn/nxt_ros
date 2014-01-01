#include <rviz/ogre_helpers/shape.h>

#include "nxt_color_display.h"
#include "nxt_color_visual.h"

namespace nxt_rviz_plugin
{

NXTColorDisplay::NXTColorDisplay()
{
  alpha_property_ = new rviz::FloatProperty( "Alpha", .5f,
                                             "Amount of transparency to apply to the circle. "
					     "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateAlpha() ));

  display_length_property_ = new rviz::FloatProperty( "Display Length", .003f,
                                                      "",
                                                      this, SLOT( updateDisplayLength() ));

  visual_.reset(new NXTColorVisual( context_->getSceneManager(), scene_node_ ));
}

void NXTColorDisplay::updateAlpha()
{
  visual_->setAlpha( alpha_property_->getFloat() );
}

void NXTColorDisplay::updateDisplayLength()
{
  visual_->setDisplayLength( display_length_property_->getFloat() );
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

  visual_->setMessage( msg );
  visual_->setDisplayLength( display_length );
  visual_->setAlpha( alpha_property_->getFloat() );
  visual_->setFramePosition( position );
  visual_->setFrameOrientation( orientation );
}

} // namespace nxt_rviz_plugin  


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nxt_rviz_plugin::NXTColorDisplay, rviz::Display) 
