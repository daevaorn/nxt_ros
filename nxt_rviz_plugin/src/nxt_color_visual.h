#ifndef NXT_COLOR_VISUAL_H
#define NXT_COLOR_VISUAL_H

#include <nxt_msgs/Color.h>

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

class NXTColorVisual 
{
public:
  NXTColorVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
  virtual ~NXTColorVisual();

  void setMessage( const nxt_msgs::Color::ConstPtr& msg );
  void setAlpha( float alpha );
  void setDisplayLength( float display_length );

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

private:
  nxt_msgs::Color::ConstPtr msg_;
  boost::shared_ptr<rviz::Shape> cylinder_;

  Ogre::SceneNode* frame_node_;

  Ogre::SceneManager* scene_manager_;
};

} // namespace nxt_rviz_plugin

#endif /* NXT_COLOR_VISUAL_H */

