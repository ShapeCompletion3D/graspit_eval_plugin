#include "include/graspit_eval_interface.h".h"

extern "C" Plugin* createPlugin() {
   //ros::init(NULL, NULL,"graspit_eval");
  return new EvalPlugin();
}

extern "C" std::string getType() {
  return "graspit_eval_plugin";
}
