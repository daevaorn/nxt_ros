#include <rviz/ogre_helpers/shape.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "nxt_ultrasonic_visual.h"

namespace nxt_rviz_plugin
{

NXTUltrasonicVisual::NXTUltrasonicVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  frame_node_ = parent_node->createChildSceneNode();

  cone_.reset( new rviz::Shape( rviz::Shape::Cone, scene_manager_, frame_node_ ) );

  Ogre::Vector3 scale( 0, 0, 0 );
  cone_->setScale( scale );
}

NXTUltrasonicVisual::~NXTUltrasonicVisual()
{
  scene_manager_->destroySceneNode( frame_node_ );
}

void NXTUltrasonicVisual::setMessage( const nxt_msgs::Range::ConstPtr& msg )
{
  msg_ = msg;

  Ogre::Vector3 scale( sin(msg->spread_angle) * msg->range, sin(msg->spread_angle) * msg->range , msg->range );
  cone_->setScale(scale);
}

void NXTUltrasonicVisual::setColor( float r, float g, float b, float alpha )
{
  cone_->setColor( r, g, b, alpha );
}

void NXTUltrasonicVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void NXTUltrasonicVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

}
