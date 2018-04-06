#!/usr/bin/env python

import rospy;
import math;
import threading;
import numpy as np;
from nav_msgs.srv import GetPlan ,GetPlanRequest,GetMap;
from nav_msgs.msg import Path , OccupancyGrid, Odometry;
from geometry_msgs.msg import Point , PoseStamped
from std_msgs.msg import Bool;
from burgard.srv import goal_report;
import roslib;
import actionlib;
from actionlib_msgs.msg import *;
from move_base_msgs.msg import *;
from frontier_search import *

name_space="robot1";
robot_number=0;
number_of_robots=0;
######################
merged_map_lock=threading.Lock()
merged_map=None;
#####################
goal_clients=[];
#############
odom_subscriber=None;
robot_x=0;
robot_y=0;
#################
beta=1;
alpha=25;
utility=100;
laser_range=30;
#################
my_server=None;
goals_list=[];
goals_list_lock=threading.Lock();
OurWrapper=[];
#############################
checking_goals_flag=False;
checking_goals_subscriber=None;
checking_goals_publisher=None;
############################
###############################
move_client_=None
move_client_goal_=None;
goal_pose=PoseStamped();
current_goal_status = 0 ; # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
move_base_status_subscriber=None;
#########################
visited_goals_list=[];

class MyWrapper:
    def __init__(self):
        self.robot_number=int(name_space[-1]);
        self.goal_server=rospy.Service("/"+name_space[-1]+"/robot_goal", goal_report, self.set_Goal);
        self.map_clients=[];
        for i in range(number_of_robots):
            rospy.wait_for_service("/robot"+str(i)+"/dynamic_map")
            try:
                self.map_clients.append(rospy.ServiceProxy("/robot"+str(i)+"/dynamic_map", GetMap));
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        self.map_subscriber=rospy.Subscriber("/"+name_space[-1]+"/inbox_Map", Data_Map, self.set_Map);
    def set_Map(self):
        global merged_map_lock;
        global merged_map;
        merged_map_lock.acquire();
        map_data=self.map_clients[0];
        merged_map=map_data.map;
        for i in range(1,number_of_robots):
            map_data=self.map_clients[i];
            temp_map=np.array([map_data.map.data,merged_map.data]);
            merged_map.data=list(np.max(temp_map,axis=0));
        merged_map_lock.release();
    def set_Goal(self, goal_data):
        global goals_list,goals_list_lock;
        goals_list_lock.acquire()
        a=int(goal_data.robot_name[-1]);
        if(a>self.robot_number):
            a=a-1;
        print(name_space,"new goal from",self.robot_name);
        goals_list[a]=Point(goal_data.goal_x,goal_data.goal_y,0.0);
        goals_list_lock.release();
        return goal_reportResponse(0);

################################################
################################################
def frontier_is_new(new_frontier,frontiers_list):
    for i in frontiers_list:
        if(((new_frontier[0]-i[0]) ** 2 + (new_frontier[1]-i[1]) ** 2)<5):
            return False;
    return True;

def get_frontiers(map_data):
        global robot_x,robot_y;
        frontiers=[];
        fsc=FrontierSearch(map_data,5,"centroid");
        frontiers=fsc.searchFrom(Point(robot_x,robot_y,0.0));
        return list(frontiers);
        '''
        map_width=int( map_data.info.width); #max of x
        map_height=int(map_data.info.height);#max of y
        map_size=map_height*map_width;
        temp_list=[-1,0,1];
        for y in range(1,map_height-1):
            for x in range(1,map_width-1):
                counter=0;
                if map_data.data[(y*map_width)+x]==0:
                    for i in temp_list:
                        for j in temp_list:
                            if not(j==i and i==0):
                                if(map_data.data[(y+i)*map_width+(x+j)]<0):
                                    counter+=1;
                                if(map_data.data[(y+i)*map_width+(x+j)]>10):
                                    counter=10;
                    if (counter==3):
                        temp_x=(x)*map_data.info.resolution+map_data.info.origin.position.x;
                        temp_y=(y)*map_data.info.resolution+map_data.info.origin.position.y;
                        if(frontier_is_new([temp_x,temp_y],frontiers)==True):
                            frontiers.append([temp_x,temp_y]);


        print("this is number of fronteirs", len(frontiers))
        return list(frontiers);
        '''


################################################
################################################
def callback_goal_status(data,data2):
    global current_goal_status;
    current_goal_status=True;
    return;
    if len(data.status_list)==0 :
        return;
    current_goal_status = data.status_list[len(data.status_list) - 2].status;

def move_base_tools():
    global move_client_goal_;
    global move_client_;
    global name_space;
    global move_base_status_subscriber;
    move_client_=actionlib.SimpleActionClient("/"+name_space+"/move_base", MoveBaseAction);
    move_client_goal_=MoveBaseGoal();
    print(name_space,"move base tools are ok")
    #move_base_status_subscriber=rospy.Subscriber("/"+name_space+"/move_base/status", GoalStatusArray, callback_goal_status);


################################################
################################################################################################
################################################



def send_goal(goal_x,goal_y):
    global OurWrapper;
    global name_space;
    global move_client_;
    global move_client_goal_;
    global goal_pose;
    global goal_clients;
    for i in goal_clients:
        i(goal_x,goal_y,name_space);

        # set goal
    goal_pose.pose.position.x = goal_x;
    goal_pose.pose.position.y = goal_y;
    goal_pose.pose.orientation.w = 1.0;
    goal_pose.pose.orientation.z = 0;
    goal_pose.header.frame_id = "/map";
    goal_pose.header.stamp = rospy.Time.now();
        # send goal
    move_client_.cancel_goals_at_and_before_time(rospy.Time.now());
    move_client_goal_.target_pose=goal_pose;
    rospy.sleep(0.5);
    checking_goals_publisher.publish(Bool(False));
    #move_client_.send_goal_and_wait(goal=move_client_goal_,execute_timeout = rospy.Duration(300),preempt_timeout = rospy.Duration(1));
    move_client_.send_goal(goal=move_client_goal_,done_cb=callback_goal_status);
    print(name_space,"sent goal");
    goal_pose.header.seq =goal_pose.header.seq+1 ;
################################################
################################################

def odom_callback(odom_data):
    global robot_x,robot_y;
    robot_x=odom_data.pose.pose.position.x;
    robot_y=odom_data.pose.pose.position.y;
################################################
################################################

def burgard():
    global merged_map_lock;
    global merged_map;
    global name_space;
    global goals_list;
    global goals_list_lock;
    global alpha;
    global checking_goals_publisher,checking_goals_flag;
    global current_goal_status;
    global OurWrapper;
    global visited_goals_list;
    while(merged_map==None):
        pass;
    while not rospy.is_shutdown():
        OurWrapper.set_Map();
        merged_map_lock.acquire();
        print(name_space,"going for frointiers")
        frontiers=get_frontiers(merged_map);
        merged_map_lock.release();
        if (len(frontiers)==0):
            print(name_space,"no new frontiers");
            exit();
        #frontiers=compute_frontier_distance(frontiers);
        print(name_space,"we have frontiers",len(frontiers));
        if (len(frontiers)==0):
            print(name_space,"no path to frointiers");
            exit();
        rate = rospy.Rate(0.5);

        for k in range(int(name_space[-1]),robot_number):
            while goals_list[k]==None:
                pass;

        checking_goals_publisher.publish(Bool(True));
        rate = rospy.Rate(0.5);
        while(not checking_goals_flag):
            rate.sleep();
        rospy.sleep(0.5);
        for i in range(0,len(frontiers)):
            goals_list_lock.acquire();
            for j in goals_list:
                if(j==None):continue;
                temp_distance=math.sqrt( (j.x-frontiers[i].travel_point.x)**2 +  (j.y-frontiers[i].travel_point.y)**2);
                if(temp_distance<=laser_range):
                    print(name_space,"before increment",str(frontiers[i].min_distance))
                    frontiers[i].min_distance+=alpha*(1-temp_distance/laser_range);
                    print(name_space,"after ",str(frontiers[i].min_distance))

                else:
                    print(name_space," goal out of range ",str(j.x),str(j.y));
            for j in visited_goals_list:
                    if(j==None):continue;
                    temp_distance=math.sqrt( (j.x-frontiers[i].travel_point.x)**2 +  (j.y-frontiers[i].travel_point.y)**2);
                    if(temp_distance<=laser_range):
                        print(name_space,"before increment",str(frontiers[i].min_distance))
                        frontiers[i].min_distance+=alpha*(1-temp_distance/laser_range)+10;
                        print(name_space,"after ",str(frontiers[i].min_distance))
                    else:
                        print(name_space," goal out of range ",str(j.x),str(j.y));
            goals_list_lock.release();

        print(name_space,"sorting");
        frontiers.sort(key=lambda node: node.min_distance);
        print(name_space,"worst frontier",frontiers[-1].min_distance,"  best frontier",frontiers[0].min_distance);
        print(name_space,"  goal is ",str(frontiers[0].travel_point.x),str(frontiers[0].travel_point.y));
        current_goal_status=False;
        send_goal(frontiers[0].travel_point.x,frontiers[0].travel_point.y);
        visited_goals_list.append(Point(frontiers[0].travel_point.x,frontiers[0].travel_point.y,0.0));
        rospy.sleep(3.0);
        time_counter=0;
        while current_goal_status==False and time_counter<140:
            rate.sleep();
            time_counter+=2;
        current_goal_status=False;

def checking_goals_response_callback(input_data):
    global checking_goals_flag;
    checking_goals_flag=input_data.data;


def main():
    global name_space,robot_number,number_of_robots;
    global merged_map,goals_list,OurWrapper;
    global odom_subscriber;
    global goal_clients;
    global checking_goals_subscriber,checking_goals_publisher;
    rospy.init_node("burgard_exploration_node");
    name_space = rospy.get_param("namespace", default="robot1");
    robot_number=int(name_space[-1]);
    number_of_robots=(int(rospy.get_param("number_of_robots", default=1)));

    for i in range (0,number_of_robots):
        if (i==robot_number):continue;
        goals_list.append(None);

    OurWrapper=MyWrapper();
    for i in range (0,number_of_robots):
        if (i==robot_number):continue;
        rospy.wait_for_service("/"+str(i)+"/robot_goal")
        try:
            goal_clients.append(rospy.ServiceProxy("/"+str(i)+"/robot_goal", goal_report));
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    move_base_tools();
    map_subscriber=rospy.Subscriber("/"+name_space+"/map", OccupancyGrid, map_callback);
    odom_subscriber=rospy.Subscriber("/"+name_space+"/odom", Odometry, odom_callback);
    checking_goals_subscriber=rospy.Subscriber("/"+name_space+"/checking_goals_response", Bool, checking_goals_response_callback);
    checking_goals_publisher=rospy.Publisher("/"+name_space+"/checking_goals_request", Bool,queue_size=15);
    burgard();
    rospy.spin();

if __name__ == '__main__':
    main();
