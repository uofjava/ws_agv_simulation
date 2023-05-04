import rospy
import yaml
import inject
import paho.mqtt.client as mqtt
from pgk_agv.util import lookup_object


def create_config(mqtt_client):
    def config(binder):
        binder.bind(mqtt.Client, mqtt_client)
    return config


def mqtt_client_node():
    params = rospy.get_param("~",None)
    if(params == None):
        with open('/home/yyt/share/ws_agv_simulation/src/pgk_agv/config/mqtt_params.yml','rb') as f:
            # yaml文件通过---分节，多个节组合成一个列表
            date = yaml.safe_load_all(f)
            # salf_load_all方法得到的是一个迭代器，需要使用list()方法转换为列表
            params = list(date)[0]
    # rospy.logwarn(params)        
    mqtt_client = lookup_object(object_path='mqtt_node.mqtt_client:mqtt_client_creat',package='mqtt_node')(params)

        # dependency injection
    config = create_config(
        mqtt_client)
    inject.configure(config)

    mqtt_client.on_connect = _on_connect
    mqtt_client.on_disconnect = _on_disconnect
    con_params = params.get('mqtt',{}).get('connection')
    # rospy.logerr(con_params)
    
    mqtt_client.connect(host=con_params.get('host','127.0.0.1'), port=con_params.get('port',1883),keepalive = 60)

    for bri in params.get('mqtt',{}).get('bridge',[]):
            # rospy.logwarn(bri)
            lookup_object(object_path='mqtt_node.mqtt_bridge:RosToMqttBridge',package='mqtt_node')(bri)


    # start MQTT loop
    mqtt_client.loop_start()

    # register shutdown callback and spin
    rospy.on_shutdown(mqtt_client.disconnect)
    rospy.on_shutdown(mqtt_client.loop_stop)
    rospy.spin()




def _on_connect(client, userdata, flags, response_code):
    rospy.loginfo('MQTT connected')


def _on_disconnect(client, userdata, response_code):
    rospy.loginfo('MQTT disconnected')


__all__ = ['mqtt_client_node']    