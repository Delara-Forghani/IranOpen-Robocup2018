import ros.py;
import roslib;
import smach;
import smach_ros;
class Goal_generation(smach.State):


 def __init__(self):
     smach.State.__init__(self,outcomes=["goal_generated","end_of_explore","timeout"]);
#any inputkey and output key

 def execute(self,userdata):
  rospy.loginfo("generation state");
  a="first";
  if a=="first":
     return "goal_generated";
  elif a=="second":
     return "reach_to_goal";
  else:
     return "timeout"; 

############


class Move_towards_goal(smach.State):
 def __init__(self):
     smach.State.__init__(self,outcomes=["reach_to_goal","victim_recognized","timeout"]);
#any inputkey and output key????

 def execute(self,userdata):
  rospy.loginfo("move to goal state");
  a="first";
  if a=="first":
     return "reach_to_goal";
  elif a=="second":
     return "victim_recognized";
  elif a=="third":
     return "timeout";


############

class A_victim_has_been_seen(smach.State):
 def __init__(self):
     smach.State.__init__(self,outcomes=["victim_marked"]);
#any inputkey and output key????

 def execute(self,userdata):
  rospy.loginfo("see a victim state");
     return "victim_marked";
  
############
  
class Move_towards_victim_to_park(smach.State):
 def __init__(self):
     smach.State.__init__(self,outcomes=["victim_recognized2","timeout2"]);
#any inputkey and output key????

 def execute(self,userdata):
  rospy.loginfo("see a victim state");
  a="first";
  if a=="first":
     return "victim_recognized2";
  elif a=="second":
     return "timeout2";

############

class End_of_rescue_mission(smach.State):
 def __init__(self):
     smach.State.__init__(self,outcomes=["shutdown"]);
#any inputkey and output key????

 def execute(self,userdata):
  rospy.loginfo("see a victim state");
     return "shutdown";

###########
def main():
   state_machine = smach.StateMachine(outcomes=["Done"]);   #outcomes=["hold", "shut_down"]
   with state_machine:
     smach.StateMachine.add("Goal_generation",Goal_generation(),transitions={"goal_generated":"Move_towards_goal","end_of_explore":"Move_towards_victim_to_park","timeout":"Move_towards_victim_to_park"},);
#remapping??
      smach.StateMachine.add("Move_towards_goal",Move_towards_goal(),transitions={"reach_to_goal":"Goal_generation","victim_recognized":"A_victim_has_been_seen","timeout":"Move_towards_victim_to_park"},);
       
      smach.StateMachine.add("A_victim_has_been_seen",A_victim_has_been_seen(),transitions={"victim_marked":"Move_towards_goal"},);

      smach.StateMachine.add("Move_towards_victim_to_park", Move_towards_victim_to_park(),transitions={"victim_recognized2":"A_victim_has_been_seen","timeout2":"End_of_rescue_mission"},);
 

       smach.StateMachine.add("End_of_rescue_mission",End_of_rescue_mission(),transitions={"shutdown":"Done"},);

rescue = state_machine.execute();



if __name__ == '__main__':
  
    main();

 

             
    
