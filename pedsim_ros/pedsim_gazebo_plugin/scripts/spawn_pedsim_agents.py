#!/usr/bin/env python
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
from rospkg import RosPack
from pedsim_msgs.msg import AgentStates
from gazebo_msgs.srv import DeleteModel

# xml file containing a gazebo model to represent agent, currently is represented by a cubic but can be changed
global xml_file
immortal_pedestrians = True
actor_limit = 25

def actor_poses_callback(actors):

    # default code
    if immortal_pedestrians:
        for actor in actors.agent_states:
            actor_id = str( actor.id )
            actor_pose = actor.pose
            rospy.loginfo("Spawning model: actor_id = %s", actor_id)

            model_pose = Pose(Point(x= actor_pose.position.x,
                                y= actor_pose.position.y,
                                z= actor_pose.position.z),
                            Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )

            spawn_model(actor_id, xml_string, "", model_pose, "world")
        rospy.signal_shutdown("all agents have been spawned !")


    # code for continuous spawning and despawning
    else:
    
        # delete actors
        if len(existing_actors) > actor_limit:
            delete_model_prox(existing_actors[1]) 
            removed_actors.append(existing_actors[1])
            existing_actors.pop(1)


        for actor in actors.agent_states:
            actor_id = str( actor.id )

            # only spawn models that haven't already been created         
            if actor_id in existing_actors:
                pass
            elif actor_id in removed_actors:
                pass
            else:
                actor_pose = actor.pose
                rospy.loginfo("Spawning model: actor_id = %s", actor_id)

                model_pose = Pose(Point(x= actor_pose.position.x,
                                y= actor_pose.position.y,
                                z= actor_pose.position.z),
                            Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )

                spawn_model(actor_id, xml_string, "", model_pose, "world")
                existing_actors.append(actor_id)
        



if __name__ == '__main__':

    existing_actors = []
    removed_actors = []
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    rospy.init_node("spawn_pedsim_agents")
    immortal_pedestrians = rospy.get_param("/spawn_pedsim_agents/immortal_peds")
    actor_limit = rospy.get_param("/spawn_pedsim_agents/actor_limit")

    rospack1 = RosPack()
    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
    default_actor_model_file = pkg_path + "/models/actor_model.sdf"

    actor_model_file = rospy.get_param('~actor_model_file', default_actor_model_file)
    file_xml = open(actor_model_file)
    xml_string = file_xml.read()

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    print("service: spawn_sdf_model is available ....")
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, actor_poses_callback)

    rospy.spin()
