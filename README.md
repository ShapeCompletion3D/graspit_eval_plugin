# graspit_eval_plugin
Graspit Plugin: plan grasps on a completed object, evaluate them on the ground truth

This repo enables a user to plan grasps on a completion of an object, and then evaluate those grasps on the ground truth mesh model.  If you want to record the exact joint states rather than only the dof values, uncomment the relevant lines in this package, and make the getJoints method in the Robot class in graspit public.
