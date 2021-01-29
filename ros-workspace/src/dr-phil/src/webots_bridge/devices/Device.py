
class Device:
    def __init__(self,robot_unique_name,device_name):
        self.robot_unique_name = robot_unique_name
        self.device_name = device_name

    def get_service_name(self,service_name,device=-1):
        return "/{0}/{1}/{2}".format(self.robot_unique_name,
            self.device_name if device == -1 else device,
            service_name)
    
    def update_device(self,dt):
        pass

    def setup_device(self):
        pass