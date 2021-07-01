#!/usr/bin/env python
import rospy
from next_best_sense.srv import GetClosestNode, GetClosestNodeRequest, GetClosestNodeResponse
from std_msgs.msg import String
import threading

current_node = None
mutex = threading.Lock()  

def _node_callback(msg):
    mutex.acquire()
    global current_node
    current_node = msg.data
    mutex.release()

def _get_closest_node(request):
    return GetClosestNodeResponse(closest_node = current_node)


if __name__ == '__main__':
    rospy.init_node("closest_node")

    rospy.Subscriber("/closest_node", String, _node_callback, queue_size=10)
    closest_node_service = rospy.Service('/get_closest_node', GetClosestNode, _get_closest_node)
    rospy.spin()