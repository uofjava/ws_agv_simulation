import rospy
from pgk_agv.app import modbus_client_node

if __name__ == '__main__':
    rospy.init_node('hc_robot_modbus')
    try:
        modbus_client_node()
    except rospy.ROSInterruptException:
        pass 
    rospy.spin()    