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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <swarm_robots_msgs/Vertex.h>
#include <tuw_multi_robot_rviz/VoronoiGraphVisual.h>

namespace tuw_multi_robot_rviz
{

VoronoiGraphVisual::VoronoiGraphVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
{
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();

    colorPath_ = Ogre::ColourValue(255, 0, 0);
    scalePoint_ = 0.1;
    path_width_ = 1;

    path_style_ = LINES;
}

VoronoiGraphVisual::~VoronoiGraphVisual()
{
    scene_manager_->destroySceneNode(frame_node_);
}

void VoronoiGraphVisual::setMessage(const swarm_robots_msgs::Graph::ConstPtr &msg)
{
    static double timeOld_;
    if (timeOld_ == msg->header.stamp.toSec())
    {
        return;
    }
    timeOld_ = msg->header.stamp.toSec();


    switch (path_style_)
    {
    case LINES:
        std::cout << "lines" << std::endl;
        billboardline.clear();
        pathLine.resize(msg->vertices.size());

        for(size_t i = 0; i < pathLine.size(); i ++)
        {
            swarm_robots_msgs::Vertex seg = msg->vertices[i];
            geometry_msgs::Point p1 = seg.path.front();
            geometry_msgs::Point p2 = seg.path.back();
            pathLine[i].reset(new rviz::Line(scene_manager_, frame_node_));
            pathLine[i]->setColor(colorPath_);
            pathLine[i]->setPoints(Ogre::Vector3((p1.x) , (p1.y) , p1.z), Ogre::Vector3((p2.x) , (p2.y) , p2.z));
            pathLine[i]->setScale(Ogre::Vector3(1.0, 1.0, 1.0));
        }

        break;
    case BILLBOARDS:
        pathLine.clear();
        billboardline.resize(msg->vertices.size());

        for(size_t i = 0; i < billboardline.size(); i ++)
        {
            swarm_robots_msgs::Vertex seg = msg->vertices[i];
            geometry_msgs::Point p1 = seg.path.front();
            geometry_msgs::Point p2 = seg.path.back();
            billboardline[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
            billboardline[i]->setNumLines(1);
            billboardline[i]->setLineWidth(path_width_);
            billboardline[i]->addPoint(Ogre::Vector3((p1.x) , (p1.y) , p1.z), colorPath_);
            billboardline[i]->addPoint(Ogre::Vector3(p2.x, p2.y, p2.z), colorPath_);
        }

        break;
    default:
        break;
    }

    crossingShape.resize(pathLine.size() * 2);
    for (size_t i = 0; i < pathLine.size(); ++i)
    {
        swarm_robots_msgs::Vertex seg = msg->vertices[i];
        geometry_msgs::Point p1 = seg.path.front();
        geometry_msgs::Point p2 = seg.path.back();

        Ogre::Quaternion rotation;
        rotation.x = 1;
        rotation.y = 0;
        rotation.z = 0;
        rotation.w = 0;
        Ogre::Quaternion rotation2 = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::PI / 2.), Ogre::Vector3::UNIT_Y);

        crossingShape[2 * i].reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
        crossingShape[2 * i]->setColor(colorPath_);
        crossingShape[2 * i]->setPosition(Ogre::Vector3((p1.x) , (p1.y) , p1.z));
        crossingShape[2 * i]->setOrientation(rotation * rotation2);
        crossingShape[2 * i]->setScale(Ogre::Vector3(scalePoint_, scalePoint_, scalePoint_));

        crossingShape[2 * i + 1].reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
        crossingShape[2 * i + 1]->setColor(colorPath_);
        crossingShape[2 * i + 1]->setPosition(Ogre::Vector3((p2.x) , (p2.y) , p2.z));
        crossingShape[2 * i + 1]->setOrientation(rotation * rotation2);
        crossingShape[2 * i + 1]->setScale(Ogre::Vector3(scalePoint_, scalePoint_, scalePoint_));
    }
}

void VoronoiGraphVisual::setFramePosition(const Ogre::Vector3 &position)
{
    frame_node_->setPosition(position);
}

void VoronoiGraphVisual::setFrameOrientation(const Ogre::Quaternion &orientation)
{
    frame_node_->setOrientation(orientation);
}

void VoronoiGraphVisual::setPathColor(Ogre::ColourValue color)
{
    colorPath_ = color;
    for (auto &pathXYi : pathLine)
    {
        pathXYi->setColor(colorPath_);
    }
}

void VoronoiGraphVisual::setPointScale(float scale)
{
    scalePoint_ = scale;
    for (auto &pathThetai : crossingShape)
    {
        pathThetai->setScale(Ogre::Vector3(scalePoint_, scalePoint_, scalePoint_));
    }
}

void VoronoiGraphVisual::setPathWidth(float width)
{
    path_width_ = width;
    for(auto &path : billboardline){
        path->setLineWidth(path_width_);
    }
}

void VoronoiGraphVisual::setPathStype (int style)
{
    path_style_ = (LineStyle)style;
    pathLine.clear();
    billboardline.clear();
}


} // namespace tuw_multi_robot_rviz
