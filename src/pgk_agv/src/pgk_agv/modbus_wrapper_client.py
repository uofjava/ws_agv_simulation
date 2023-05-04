import rospy
from threading import Lock
from pgk_agv.util import lookup_object



class ModbusClient():
    def __init__(self,params):
        rospy.loginfo(params)  
        try:
            self.client = lookup_object(object_path='pymodbus.client:ModbusTcpClient')(params.get('host','127.0.0.1'),params.get('port',502))
            self.client.connect()
        except Exception as e:
            rospy.logwarn("could not get a modbus connection %s",str(e))
            return
          
        self.__rate = params.get('rate', 1)
        self.__slave = params.get('slave', 1)
        self.post = lookup_object(object_path="pgk_agv.post_threading:Post",package='pgk_agv')(self)
        self.__registers = params.get('registerconfig',[])
        # rospy.loginfo(self.__registers)
        self.__input = lookup_object(object_path='std_msgs.msg:Int32MultiArray',package='pgk_agv')()       
        self.__input.data = [0 for i in range(params.get('registerAllCount', 1))]
        self.__mutex = Lock()

        self.__pub = rospy.Publisher(
            params.get('id','test'),
            lookup_object(object_path='std_msgs.msg:Int32MultiArray',package='pgk_agv'), 
            queue_size=500, 
            latch=True)
        
        rospy.on_shutdown(self.closeConnection)


    def readRegisters(self,address=None,num_registers=None):
        tmp = []
        with self.__mutex:            
            try:
                for reg in self.__registers:
                    # rospy.logwarn(reg)
                    respon = self.client.read_holding_registers(address=reg.get('address',0),count=reg.get('count',0),slave=self.__slave)
                    if respon.registers:
                        # data = respon.registers
                        for i in range(reg.get('count',0)):
                            # rospy.logwarn(i)
                            # rospy.loginfo(type(respon.registers))
                            # rospy.loginfo(respon.registers[i])
                            tmp.insert((i+reg.get('address',0)) , respon.registers[i])
            except Exception as e:
                rospy.logwarn("Could not read on address %d. Exception: %s",reg.get('address'),str(e))
        return tmp



    def __updateModbusInput(self,delay=0):
        
        rospy.sleep(delay)
        self.listener_stopped = False
        self.stop_listener = False
        update = True
        while not rospy.is_shutdown() and self.stop_listener is False:
            if self.client.is_socket_open():
                try: 
                    if not rospy.is_shutdown() :
                        tmp =  self.readRegisters()
                        # rospy.logerr("modbus info %s",self.client)
                        if tmp is None:
                            rospy.logwarn("modbus is disconnection! %s", self.client)
                            rospy.sleep(2)
                            continue
                        # rospy.logwarn("__updateModbusInput tmp is %s ", self.client)
                        # rospy.logwarn("__updateModbusInput self.__input.data is %s ", str(self.__input.data))
                        if tmp != self.__input.data:
                            update = True
                            self.__input.data = tmp
                        else:
                            update = False 
                except Exception as e:
                    rospy.logwarn("Could not read holding register. %s", str(e))
                    raise e
                    rospy.sleep(2)
                if update:
                    if self.__pub.get_num_connections() > 0:
                        try:
                            self.__pub.publish(self.__input)
                        except Exception as e:
                            rospy.logwarn("Could not publish message. Exception: %s",str(e))
                            raise e
            else:
                rospy.loginfo('reconnection %s',self.client)
                rospy.sleep(10)
                if self.client:
                    self.client.connect()
            rospy.Rate(self.__rate).sleep()
        self.listener_stopped = True


    def startListening(self):
        """
            Non blocking call for starting the listener for the readable modbus server registers 
        """
        #start reading the modbus
        self.post.__updateModbusInput()



    def closeConnection(self):
        """
            Closes the connection to the modbus
        """
        self.client.close()    