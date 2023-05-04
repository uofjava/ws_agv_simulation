import rospy
import yaml
import json
import inject
import paho.mqtt.client as mqtt
from pgk_agv.util import lookup_object


class Bridge(object):
    _mqtt_client = inject.attr(mqtt.Client)

class RosToMqttBridge(Bridge):
    def __init__(self,params) -> None:
        # rospy.logerr(params)
        self._topic_from = params.get('topic_from','test')
        self._to_mqtt_tcp = params.get('to_mqtt_tcp', 'test')
        self._registerconfig = params.get('registerconfig', [])
        rospy.Subscriber(self._topic_from,lookup_object(object_path=params.get('msg_type','std_msgs.msg:Int32MultiArray'), package='mqtt_node'),self._callback_ros)

    def _callback_ros(self, msg):
        rospy.loginfo('ROS recevived from {} data {}'.format(self._topic_from, msg))    
        self._publish(msg)

    def _publish(self, msg):
        data = yaml.load(str(msg)).get('data',[])
        # rospy.logwarn(self._registerconfig)
        # rospy.logwarn(data)
        # rospy.logwarn(type(data))
        d = []
        for reconfig in self._registerconfig:
            # rospy.logwarn(reconfig)
            first = reconfig.get('address',0)
            end = first + reconfig.get('count',0) 
            # rospy.logwarn(first)
            # rospy.logwarn(end)
            d.append({
                reconfig.get('tag','info'): data[first: end]
            })
        payload_data = json.dumps(d)
        # rospy.logwarn(payload_data)
        self._mqtt_client.publish(topic=self._to_mqtt_tcp, payload=payload_data)

