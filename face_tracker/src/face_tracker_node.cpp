#include "face_tracker.h"



int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "face_tracker_node");

  // node handler
  ros::NodeHandle n;

  //Instantiate facetracker object
  face_tracker::faceTracker ft(n);

  ft.execute_kallman();
  
  return 0;
}
