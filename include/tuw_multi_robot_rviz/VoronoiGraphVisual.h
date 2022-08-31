/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VORONOIGRAPHVISUAL_H
#define VORONOIGRAPHVISUAL_H

#include <boost/bind/bind.hpp>

#include <swarm_robots_msgs/Graph.h>
#include <geometry_msgs/Vector3.h>

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/billboard_line.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Shape;
class EnumProperty;
}

namespace tuw_multi_robot_rviz
{

class VoronoiGraphVisual
{
public:

  VoronoiGraphVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  virtual ~VoronoiGraphVisual();

  void setMessage( const swarm_robots_msgs::Graph::ConstPtr& msg );

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  void setPathColor( Ogre::ColourValue color );
  void setPointScale ( float scale );
  void setPathWidth ( float width );
  void setPathStype (int style);

private:

  std::vector<boost::shared_ptr<rviz::BillboardLine>> billboardline;
  std::vector<boost::shared_ptr<rviz::Line> > pathLine;
  std::vector<boost::shared_ptr<rviz::Shape> > crossingShape;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  Ogre::ColourValue colorPath_;

  float path_width_;
  float scalePoint_;

  enum LineStyle
  {
    LINES,
    BILLBOARDS
  };

  LineStyle path_style_;

};

}

#endif // IMU_VISUAL_H