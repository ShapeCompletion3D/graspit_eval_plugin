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
#include <fstream>


#include <include/world.h>
#include <include/body.h>
#include <include/robot.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>

#include <include/EGPlanner/egPlanner.h>
#include <include/EGPlanner/simAnnPlanner.h>
#include <include/EGPlanner/searchState.h>
#include <include/EGPlanner/searchEnergy.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/Vertices.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <include/grasp.h>

#include <cmdline/cmdline.h>

#include <include/triangle.h>

EvalPlugin::EvalPlugin() :
    mPlanner(NULL),
    plannerStarted(false),
    plannerFinished(false),
    evaluatingGrasps(false),
    num_steps(70000)
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
    parser->add<std::string>("result_filepath", 'z', "result_filepath", false);
    parser->add<int>("scale", 's', "scale", false);
    parser->add<double>("jaccard", 'j', "jaccard", false);
    parser->add<bool>("render", 'l', "render", false);

    parser->parse(argc, argv);

    gt_mesh_filepath = QString::fromStdString(parser->get<std::string>("gt_mesh_filepath"));
    std::cout << "gt_mesh_filepath: " << gt_mesh_filepath.toStdString().c_str() << "\n" ;
    std::cout << argv << std::endl;

    if (parser->exist("scale"))
    {
        scale = parser->get<int>("scale");
        std::cout << "scale: " << scale << "\n" ;
    }
    else
    {
        scale = 1;
    }

    if (parser->exist("render"))
    {
        render_it = parser->get<bool>("render");
        std::cout << "render: " << render_it << "\n" ;
    }
    else
    {
        render_it = false;
    }

    if (parser->exist("jaccard"))
    {
        jaccard = parser->get<double>("jaccard");
        std::cout << "jaccard: " << jaccard << "\n" ;
    }
    else
    {
        jaccard = 0.0;
    }




    completed_mesh_filepath = QString::fromStdString(parser->get<std::string>("completed_mesh_filepath"));
    std::cout << "completed_mesh_filepath: " << completed_mesh_filepath.toStdString().c_str() << "\n" ;


    result_filepath = QString::fromStdString(parser->get<std::string>("result_filepath"));

  return 0;
}

int EvalPlugin::mainLoop()
{
    //save grasps and reset planner
    if(plannerStarted && plannerFinished && (!evaluatingGrasps))
    {
        std::cout << "FINISHED Planning\n" ;

        SearchEnergy *mEnergyCalculator = new SearchEnergy();
        mEnergyCalculator->setType(ENERGY_CONTACT_QUALITY);
        mEnergyCalculator->setContactType(CONTACT_PRESET);

        int num_grasps = mPlanner->getListSize();
        std::cout << "Found " << num_grasps << " Grasps. " << std::endl;

        for(int i=0; i < num_grasps; i++)
        {
            GraspPlanningState gps = mPlanner->getGrasp(i);
            gps.execute(mHand);
            mHand->autoGrasp(render_it, 1.0, false);
            bool is_legal;
            double new_planned_energy;

            mEnergyCalculator->analyzeCurrentPosture(mHand,graspItGUI->getMainWorld()->getGB(0),is_legal,new_planned_energy,false );
            gps.setEnergy(new_planned_energy);
        }

        //remove completed body
        std::cout << "About to Remove Body\n" ;
        graspItGUI->getMainWorld()->destroyElement(graspItGUI->getMainWorld()->getGB(0));
        std::cout << "Removed Body\n" ;

        //add gt body
        std::cout << "About to add GT Body: " << gt_mesh_filepath.toStdString().c_str() << "\n" ;
        graspItGUI->getMainWorld()->importBody("GraspableBody", gt_mesh_filepath);
        graspItGUI->getMainWorld()->getGB(0)->setGeometryScaling(scale,scale,scale);
        std::cout << "Added GT Body\n" ;
        evaluatingGrasps = true;

        std::ofstream outfile;
        outfile.open(result_filepath.toStdString().c_str(), std::ios_base::app);
        outfile << "Planned Meshfile,";
        outfile << "Ground Truth Meshfile,";
        outfile << "Grasp Legal,";
        outfile << "Planned Quality,";
        outfile << "Evaluated Quality,";
        outfile << "Planned Joint Values,";
        outfile << "Evaluated Joint Values,";
        outfile << "Planned Pose Values,";
        outfile << "Evaluated Pose Values,";
        outfile << "Jaccard" << std::endl;

        for(int i=0; i < num_grasps; i++)
        {
            GraspPlanningState gps = mPlanner->getGrasp(i);
            gps.setObject(graspItGUI->getMainWorld()->getGB(0));

            double desiredVals [mHand->getNumDOF()];
            double desiredSteps [mHand->getNumDOF()];
            desiredSteps[0] = 20.0;
            desiredSteps[1] = 20.0;
            desiredSteps[2] = 20.0;
            desiredSteps[3] = 20.0;
            bool legal;
            double new_energy;
            double old_energy = gps.getEnergy();
            //mEnergyCalculator->analyzeState(legal,new_energy, &gps);
            //disable collisions
            graspItGUI->getMainWorld()->toggleAllCollisions(false);
            usleep(10000);
            gps.execute(mHand);
            mHand->getDOFVals(desiredVals);

            double initial_joint_values [mHand->getNumJoints()];
            mHand->getJointValues(initial_joint_values);

            transf initial_tran = mHand->getPalm()->getTran();

            if(render_it)
            {
                graspItGUI->getIVmgr()->getViewer()->render();
                usleep(1000000);
            }
            //mHand->quickOpen();
            mHand->autoGrasp(true, -10.0, false);

            if(render_it)
            {
                graspItGUI->getIVmgr()->getViewer()->render();
                usleep(1000000);
            }

            mHand->approachToContact(-200.0);
            if(render_it)
            {
                graspItGUI->getIVmgr()->getViewer()->render();
                usleep(1000000);
            }

            graspItGUI->getMainWorld()->toggleAllCollisions(true);
            mHand->approachToContact(200);
            if(render_it)
            {
                graspItGUI->getIVmgr()->getViewer()->render();
                usleep(1000000);
            }
            //gps.execute(mHand);
            //possible
            //mHand->moveDOFToContacts();
            //mHand->checkSetDOFVals();
            //moveDOFToContacts
            //mHand->jumpDOFToContact();
            //mHand->moveDOFToContacts(double *desiredVals, double *desiredSteps, bool stopAtContact, bool renderIt);
            mHand->moveDOFToContacts( desiredVals,  desiredSteps, false, true);

            if(render_it)
            {
                graspItGUI->getIVmgr()->getViewer()->render();
                usleep(1000000);
            }
            mHand->autoGrasp(render_it, 1.0, false);

            double final_joint_values [mHand->getNumJoints()];
            mHand->getJointValues(final_joint_values);
            transf final_tran = mHand->getPalm()->getTran();

            mEnergyCalculator->analyzeCurrentPosture(mHand,graspItGUI->getMainWorld()->getGB(0),legal,new_energy,false );

//            outfile << "Planned Meshfile,";
            outfile << completed_mesh_filepath.toStdString().c_str() << ",";
//            outfile << "Ground Truth Meshfile,";
            outfile <<  gt_mesh_filepath.toStdString().c_str() << ",";
//            outfile << "Grasp Legal,";
            outfile  << legal << ",";
//            outfile << "Planned Quality,";
            outfile << old_energy <<  ",";
//            outfile << "Evaluated Quality,";
            outfile << new_energy << ",";
//            outfile << "Planned Joint Values,";
            for (int j =0; j < mHand->getNumJoints(); j++)
            {
                outfile << initial_joint_values[j] << ";" ;
            }
            outfile << ",";

//            outfile << "Evaluated Joint Values,";
           for (int j =0; j < mHand->getNumJoints(); j++)
           {
               outfile << final_joint_values[j] << ";" ;
           }
           outfile << ",";
//            outfile << "Planned Pose Values,";
           outfile << initial_tran.translation().x() << ";";
           outfile << initial_tran.translation().y() << ";";
           outfile << initial_tran.translation().z() << ";";
           outfile << initial_tran.rotation().w << ";";
           outfile << initial_tran.rotation().x << ";";
           outfile << initial_tran.rotation().y << ";";
           outfile << initial_tran.rotation().z << ";";
           outfile << ",";
//            outfile << "Evaluated Pose Values" << std::endl;
           outfile << final_tran.translation().x() << ";";
           outfile << final_tran.translation().y() << ";";
           outfile << final_tran.translation().z() << ";";
           outfile << final_tran.rotation().w << ";";
           outfile << final_tran.rotation().x << ";";
           outfile << final_tran.rotation().y << ";";
           outfile << final_tran.rotation().z << ";";

           outfile << ",";

            //mEnergyCalculator->analyzeState(l2, new_energy, &gps);

           outfile << jaccard;

           outfile << std::endl;

        }

        outfile.close();


        assert(false);

    }
    //start planner
    else if (!plannerStarted)
    {
        std::cout << "Starting Planner\n" ;

        graspItGUI->getMainWorld()->importBody("GraspableBody", completed_mesh_filepath);
        graspItGUI->getMainWorld()->getGB(0)->setGeometryScaling(scale,scale,scale);

        mObject = graspItGUI->getMainWorld()->getGB(0);

        mHand = graspItGUI->getMainWorld()->getCurrentHand();
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
        if ( mPlanner->getCurrentStep() >= num_steps)
        {
            mPlanner->stopPlanner();
            plannerFinished=true;
        }
    }

  return 0;
}
