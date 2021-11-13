#task for a given robot
spds_msgs/RobotTarget target
spds_msgs/Path path
int32 criticalPoint #path point beyond which robot should not proceed (-1 means no critical point)
int32 criticalRobotID #robot responsible for "releasing" the criticalPoint
int32 releasingPoint #point criticalRobotID should reach to release criticalPoint
spds_msgs/RobotConstraints constraints
spds_msgs/DeltaTVec dts
spds_msgs/CoordinatorTimeVec cts
bool update # if this task is based on a previous task (e.g. updated with a new path path or coordination times etc.)
bool abort # if the current task should be aborted - the vehicle will enter the WAIT_STATE
