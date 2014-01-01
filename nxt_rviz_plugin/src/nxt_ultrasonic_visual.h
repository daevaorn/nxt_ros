#ifndef NXT_ULTRASONIC_VISUAL_H
#define NXT_ULTRASONIC_VISUAL_H

#include <nxt_msgs/Range.h>

#include <boost/shared_ptr.hpp>

namespace rviz
{
class Shape;
}

namespace Ogre
{
  class SceneManager;
  class SceneNode;
  class Vector3;
  class Quaternion;
}

namespace nxt_rviz_plugin
{

class NXTUltrasonicVisual 
{
public:
  NXTUltrasonicVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
  virtual ~NXTUltrasonicVisual();

  void setMessage( const nxt_msgs::Range::ConstPtr& msg );
  void setColor( float r, float g, float b, float alpha );

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

private:
  nxt_msgs::Range::ConstPtr msg_;
  boost::shared_ptr<rviz::Shape> cone_;

  Ogre::SceneNode* frame_node_;

  Ogre::SceneManager* scene_manager_;
};

} // namespace nxt_rviz_plugin

#endif /* NXT_ULTRASONIC_VISUAL_H */

