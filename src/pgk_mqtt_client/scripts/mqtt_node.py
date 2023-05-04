import rospy
import inject
import json
import yaml
import paho.mqtt.client as mqtt
from typing import Dict
from std_msgs.msg import Int32MultiArray


def mqtt_client_create(param:Dict) -> mqtt.Client:
    client_params = param.get('client',{})
    client = mqtt.Client(**client_params)
    account_params = param.get('account',{})
    if account_params:
        client.username_pw_set(**account_params)
    return client    


def mqtt_node():
    rospy.init_node('mqtt_node')
    params = rospy.get_param("~",{})
    mqtt_params = params.get("mqtt",{})
    conn_params = mqtt_params.get("connection",{})
    rospy.loginfo(conn_params)
    mqtt_client = mqtt_client_create(mqtt_params)

    def config(binder):
        binder.bind(mqtt.Client ,mqtt_client)
    inject.configure(config)  

    RosToMqttBridge(mqtt_params.get("bridge",[]))


    mqtt_client.on_connect = _on_connect
    mqtt_client.on_disconnect = _on_disconnect
    mqtt_client.connect(**conn_params)
    mqtt_client.loop_start()
    rospy.on_shutdown(mqtt_client.disconnect)
    rospy.on_shutdown(mqtt_client.loop_stop)
    rospy.spin()


def _on_connect(client, userdata, flags, rc):
    rospy.loginfo('MQTT connected')

def _on_disconnect(client, userdata, response_code):
    rospy.loginfo('MQTT disconnected')


class Bridge(object):
    _mqtt_client = inject.attr(mqtt.Client)

class RosToMqttBridge(Bridge):
    def __init__(self,params) -> None:
        rospy.loginfo(params)
        self._topic_from = '/hc_1'
        rospy.Subscriber(self._topic_from,Int32MultiArray,self._callback_ros)

    def _callback_ros(self, msg):
        rospy.loginfo('ROS recevived from {} data {}'.format(self._topic_from, msg))    
        self._publish(msg)

    def _publish(self, msg):
        data = yaml.load(str(msg))
        payload_data = json.dumps(data)
        # rospy.loginfo(payload_data)
        self._mqtt_client.publish(topic=self._topic_from, payload=payload_data)


if __name__=="__main__":
    mqtt_node()    