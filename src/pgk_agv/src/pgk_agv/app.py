import rospy
import yaml
from pgk_agv.util import lookup_object

def modbus_client_node():
    params = rospy.get_param("~",None)
    if(params == None):
        with open('/home/yyt/share/ws_agv_simulation/src/pgk_agv/config/modbus_params.yml','rb') as f:
            # yaml文件通过---分节，多个节组合成一个列表
            date = yaml.safe_load_all(f)
            # salf_load_all方法得到的是一个迭代器，需要使用list()方法转换为列表
            params =list(date)[0]
    # rospy.loginfo(params)        
    robots_params = params.get('modbus',{}).get('servers',[])        
    rospy.loginfo(robots_params)
    # mbc = ModbusClient(robots_params[0])
    # mbc.startListening()

    # configure bridges
    robots = []
    for robot_args in robots_params:
        robots.append(lookup_object(object_path='pgk_agv.modbus_wrapper_client:ModbusClient')(robot_args))

    for robot in robots:
        robot.startListening()


    
__all__ = ['modbus_client_node']    