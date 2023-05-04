import rospy
from mqtt_node.app import mqtt_client_node

if __name__ == '__main__':
    rospy.init_node('hc_mqtt')
    try:
        mqtt_client_node()
    except rospy.ROSInterruptException:
        pass 
    rospy.spin()    