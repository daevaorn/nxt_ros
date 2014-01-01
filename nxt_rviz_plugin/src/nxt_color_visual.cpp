#include <rviz/ogre_helpers/shape.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "nxt_color_visual.h"

namespace nxt_rviz_plugin
{

NXTColorVisual::NXTColorVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  frame_node_ = parent_node->createChildSceneNode();

  cylinder_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_));

  Ogre::Vector3 scale( 0, 0, 0 );
  cylinder_->setScale( scale );
}

NXTColorVisual::~NXTColorVisual()
{
  scene_manager_->destroySceneNode( frame_node_ );
}

void NXTColorVisual::setMessage( const nxt_msgs::Color::ConstPtr& msg )
{
  msg_ = msg;
}

void NXTColorVisual::setAlpha( float alpha )
{
  cylinder_->setColor( msg_->r, msg_->g, msg_->b, alpha );
}

void NXTColorVisual::setDisplayLength( float display_length )
{
  Ogre::Vector3 scale( 0.0155, 0.0155, display_length );
  cylinder_->setScale( scale );
}

void NXTColorVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void NXTColorVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

}
