import rospy
import inject

import paho.mqtt.client as mqtt
from typing import Dict
import random

from pgk_agv.util import lookup_object



def mqtt_client_creat(params: Dict) -> mqtt.Client:
    mqtt_params = params.get("mqtt",{})
    rospy.logwarn(mqtt_params)

    client_params = mqtt_params.get('client',{})
    account_params = mqtt_params.get('account',{})
    client = mqtt.Client(client_id= f'hc-mqtt-{random.randint(0, 1000)}' ,protocol=client_params.get('protocol',4))

    if account_params:
        client.username_pw_set(**account_params)

    return client



__all__ = ['mqtt_client_creat']