/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef _EvalPlugin_GRASPIT_INTERFACE_H_
#define _EvalPlugin_GRASPIT_INTERFACE_H_

#include <map>
#include <QObject>
#include <QtGui>

//GraspIt! includes
#include <include/plugin.h>

class EGPlanner;
class GraspPlanningState;
class GraspableBody;
class Hand;

//! Main class, combining a ROS node with a GraspIt! interface
/*! Note that this class inherits from GraspIt's Plugin class and implements the necessary functions to serve
  as a GraspIt plugin. See include/plugin.h in the GraspIt code for the base class.

  Provides a number of ROS services that directly operate on the GraspIt world, such as loading objects or
  obstacles, simulating 3D scans of objects, etc.

  In particular, note that this class uses the mainLoop() function to perform the ROS event management calls.
*/
class EvalPlugin : public QObject, public Plugin
{

    Q_OBJECT

public:
  //! Inits ROS, but (for now) without passing any arguments
  EvalPlugin();
  //! Deletes the node handle and the db manager
  ~EvalPlugin();
  //! Creates the node handles, advertises services, connects to the database
  virtual int init(int argc, char **argv);
  //! Simply calls ros::spinOnce() to process the ROS event loop
  virtual int mainLoop();

private:
  EGPlanner *mPlanner;
  GraspPlanningState *mHandObjectState;
  GraspableBody *mObject;
  Hand *mHand;
  bool plannerStarted;
  bool plannerFinished;
  bool evaluatingGrasps;

  QString gt_mesh_filepath;
  QString completed_mesh_filepath;
  QString result_filepath;
  int scale;
  bool render_it;
  double jaccard;

  int num_steps;

};



#endif
