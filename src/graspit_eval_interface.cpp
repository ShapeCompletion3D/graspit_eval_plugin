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

#include "include/graspit_eval_interface.h"

#include <boost/foreach.hpp>
#include <cmath>


#include <include/world.h>
#include <include/body.h>
#include <include/robot.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>

#include <include/EGPlanner/egPlanner.h>
#include <include/EGPlanner/simAnnPlanner.h>
#include <include/EGPlanner/searchState.h>
#include <include/EGPlanner/searchEnergy.h>

#include <include/grasp.h>

#include <cmdline/cmdline.h>



EvalPlugin::EvalPlugin() :
    mPlanner(NULL),
    plannerStarted(false),
    plannerFinished(false),
    evaluatingGrasps(false),
    num_steps(40000)
{
}

EvalPlugin::~EvalPlugin()
{
}

//------------------------- Main class  -------------------------------

int EvalPlugin::init(int argc, char **argv)
{
    cmdline::parser *parser = new cmdline::parser();

    parser->add<std::string>("completed_mesh_filepath", 'c', "completed_mesh_filepath",  false);
    parser->add<std::string>("gt_mesh_filepath", 'g', "gt_mesh_filepath", false);

    parser->parse(argc, argv);


    gt_mesh_filepath = QString::fromStdString(parser->get<std::string>("gt_mesh_filepath"));
    std::cout << "gt_mesh_filepath: " << gt_mesh_filepath.toStdString().c_str() << "\n" ;
    std::cout << argv << std::endl;


    completed_mesh_filepath = QString::fromStdString(parser->get<std::string>("completed_mesh_filepath"));
    std::cout << "completed_mesh_filepath: " << completed_mesh_filepath.toStdString().c_str() << "\n" ;


  return 0;
}

int EvalPlugin::mainLoop()
{
    //save grasps and reset planner
    if(plannerStarted && plannerFinished && (!evaluatingGrasps))
    {
        std::cout << "FINISHED Planning\n" ;

        //remove completed body
        std::cout << "About to Remove Body\n" ;
        graspItGUI->getMainWorld()->destroyElement(graspItGUI->getMainWorld()->getGB(0));
        std::cout << "Removed Body\n" ;

        //add gt body
        std::cout << "About to add GT Body: " << gt_mesh_filepath.toStdString().c_str() << "\n" ;
        graspItGUI->getMainWorld()->importBody("GraspableBody", gt_mesh_filepath);
        std::cout << "Added GT Body\n" ;
        evaluatingGrasps = true;

        SearchEnergy *mEnergyCalculator = new SearchEnergy();
        mEnergyCalculator->setType(ENERGY_CONTACT_QUALITY);
        mEnergyCalculator->setContactType(CONTACT_PRESET);

        int num_grasps = mPlanner->getListSize();
        std::cout << "Found " << num_grasps << " Grasps. " << std::endl;
        for(int i=0; i < num_grasps; i++)
        {
            GraspPlanningState gps = mPlanner->getGrasp(i);
            gps.setObject(graspItGUI->getMainWorld()->getGB(0));
            bool legal;
            double new_energy;
            double old_energy = gps.getEnergy();
            mEnergyCalculator->analyzeState(legal,new_energy, &gps);
            gps.execute(mHand);

            if(legal)
            {
                std::cout << "Grasp is Legal: " << legal << std::endl;
                std::cout << "new energy: " << new_energy << std::endl;
                std::cout << "original energy: " << old_energy << std::endl;
            }

        }

    }
    //start planner
    else if (!plannerStarted)
    {
        std::cout << "Starting Planner\n" ;

        graspItGUI->getMainWorld()->importBody("GraspableBody", completed_mesh_filepath);

        mHand = graspItGUI->getMainWorld()->getCurrentHand();
        mObject = graspItGUI->getMainWorld()->getGB(0);
        mHand->getGrasp()->setObjectNoUpdate(mObject);
        mHand->getGrasp()->setGravity(false);

        mHandObjectState = new GraspPlanningState(mHand);
        mHandObjectState->setObject(mObject);
        mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
        mHandObjectState->setRefTran(mObject->getTran());
        mHandObjectState->reset();

        mPlanner = new SimAnnPlanner(mHand);
        ((SimAnnPlanner*)mPlanner)->setModelState(mHandObjectState);

        mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
        mPlanner->setContactType(CONTACT_PRESET);
        mPlanner->setMaxSteps(num_steps);

        mPlanner->resetPlanner();

        mPlanner->startPlanner();
        plannerStarted = true;
    }
    //let planner run.
    else if( (plannerStarted) && !plannerFinished )
    {
        std::cout << mPlanner->getCurrentStep() << std::endl;
        if ( mPlanner->getCurrentStep() >= num_steps)
        {
            mPlanner->stopPlanner();
            plannerFinished=true;
        }
    }

  return 0;
}
