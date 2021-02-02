
class Device:

    def __init__(self,robot_unique_name,device_name):
        self.robot_unique_name = robot_unique_name
        self.device_name = device_name
        self.frame_id= self.get_service_name("")


    def get_service_name(self,service_name,device=-1):
        if service_name != "":
            return "/{0}/{1}/{2}".format(self.robot_unique_name,
            self.device_name if device == -1 else device,
            service_name) 
        else: 
            return "/{0}/{1}".format(self.robot_unique_name,
            self.device_name if device == -1 else device) 
    
    def update_device(self,time):
        pass

    def setup_device(self):
        pass

    