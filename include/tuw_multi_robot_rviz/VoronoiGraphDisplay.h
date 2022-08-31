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

#ifndef VORONOIGRAPHDISPLAY_H
#define VORONOIGRAPHDISPLAY_H

#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <swarm_robots_msgs/Graph.h>


#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <nav_msgs/Path.h>
#include <rviz/ogre_helpers/arrow.h>
#endif


#include <rviz/properties/enum_property.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class EnumProperty;
class FloatProperty;
class IntProperty;
}

namespace tuw_multi_robot_rviz
{

class VoronoiGraphVisual;


class VoronoiGraphDisplay: public rviz::MessageFilterDisplay<swarm_robots_msgs::Graph>
{
Q_OBJECT
public:

  VoronoiGraphDisplay();
  virtual ~VoronoiGraphDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateHistoryLength();

  void updateStyle();
  void updatePathWidth();
  void updatePathColor();

  void updatePointScale();
  
  
private:
  void processMessage( const swarm_robots_msgs::Graph::ConstPtr& msg );

  boost::circular_buffer<boost::shared_ptr<VoronoiGraphVisual> > visuals_;

  rviz::IntProperty* history_length_property_;

  rviz::EnumProperty* path_style_property_;
  rviz::FloatProperty* path_width_property_;
  rviz::ColorProperty* path_color_property_;

  rviz::FloatProperty* scale_point_property_;
  
  enum LineStyle {
    LINES,
    BILLBOARDS
  };
};

}

#endif
