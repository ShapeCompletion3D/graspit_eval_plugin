# graspit_eval_plugin
Graspit Plugin: plan grasps on a completed object, evaluate them on the ground truth

This repo enables a user to plan grasps on a completion of an object, and then evaluate those grasps on the ground truth mesh model.  If you want to record the exact joint states rather than only the dof values, uncomment the relevant lines in this package:
https://github.com/ShapeCompletion3D/graspit_eval_plugin/blob/master/src/graspit_eval_interface.cpp#L215

, and make the getJointValues method in the Robot class in graspit public:
https://github.com/graspit-simulator/graspit/blob/master/include/robot.h#L214
