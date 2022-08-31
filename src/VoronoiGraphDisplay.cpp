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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>


#include <tuw_multi_robot_rviz/VoronoiGraphDisplay.h>
#include <tuw_multi_robot_rviz/VoronoiGraphVisual.h>

namespace tuw_multi_robot_rviz
{

VoronoiGraphDisplay::VoronoiGraphDisplay() 
{
    history_length_property_ = new rviz::IntProperty ( "History Length", 1,
                                                        "Number of prior measurements to display.",
                                                        this, SLOT ( updateHistoryLength() ) );
    history_length_property_->setMin ( 1 );
    history_length_property_->setMax ( 100000 );

    path_style_property_ = new rviz::EnumProperty ( "Line Style", "Lines", "The rendering operation to use to draw the grid lines.",
                                                        this, SLOT(updateStyle()));
    path_style_property_->addOption("Lines", LINES);
    path_style_property_->addOption("Billboards", BILLBOARDS);

    path_width_property_ = new rviz::FloatProperty("Line Width", 0.5, "The width, in meters, of each path line, Only works with the 'Billboards' style", this, 
                                                        SLOT(updatePathWidth()), this);
    path_width_property_->setMin(0.001);
    path_width_property_->hide();


    path_color_property_ = new rviz::ColorProperty ( "Path Color", QColor ( 30, 30, 30 ),
                                                        "Color to draw the path.",
                                                        this, SLOT ( updatePathColor() ) );

    scale_point_property_ = new rviz::FloatProperty ( "Path Points Scale", 0.5,
                                                        "Scale of the path points.",
                                                        this, SLOT ( updatePointScale() ) );
    scale_point_property_->setMin ( 0 );
    scale_point_property_->setMax ( 1 );

}


void VoronoiGraphDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
}

VoronoiGraphDisplay::~VoronoiGraphDisplay() {
}

void VoronoiGraphDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
}

void VoronoiGraphDisplay::updatePathColor() {
    Ogre::ColourValue color = path_color_property_->getOgreColor();
    for ( auto& visualsI: visuals_ ) { visualsI->setPathColor ( color ); }
}

void VoronoiGraphDisplay::updateHistoryLength() {
    visuals_.rset_capacity ( history_length_property_->getInt() );
}

void VoronoiGraphDisplay::updateStyle()
{
    auto style = path_style_property_->getOptionInt();
    
    for(auto &visualsI : visuals_){
        visualsI->setPathStype(style);
    }

    switch((LineStyle)style)
    {
        case LINES:
            path_width_property_->hide();
        break;
        case BILLBOARDS:
            path_width_property_->show();
        break;

        break;
    }

}

void VoronoiGraphDisplay::updatePointScale()
{
	float scale = scale_point_property_->getFloat();
	for ( auto& visualsI: visuals_ ) { visualsI->setPointScale ( scale ); }
}

void VoronoiGraphDisplay::updatePathWidth()
{
	float scale = path_width_property_->getFloat();
	for ( auto& visualsI: visuals_ ) { visualsI->setPathWidth ( scale ); }
}

void VoronoiGraphDisplay::processMessage ( const  swarm_robots_msgs::Graph::ConstPtr& msg ) 
{
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if ( !context_->getFrameManager()->getTransform ( msg->header.frame_id,
                                                        msg->header.stamp, position, orientation ) )
    {
        ROS_DEBUG ( "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), qPrintable ( fixed_frame_ ) );
        return;
    }

    boost::shared_ptr<VoronoiGraphVisual> visual;
    if ( visuals_.full() ) {
        visual = visuals_.front();
    } else {
        visual.reset ( new VoronoiGraphVisual ( context_->getSceneManager(), scene_node_ ) );
    }

    visual->setPathColor( path_color_property_->getOgreColor() );
    visual->setPointScale(scale_point_property_->getFloat());
    visual->setPathStype(path_style_property_->getOptionInt());
    visual->setPathWidth(path_width_property_->getFloat());


    visual->setMessage          ( msg );
    visual->setFramePosition    ( position );
    visual->setFrameOrientation ( orientation );
    


    visuals_.push_back ( visual );
}


} 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tuw_multi_robot_rviz::VoronoiGraphDisplay,rviz::Display )
