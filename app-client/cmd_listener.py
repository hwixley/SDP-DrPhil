#!/usr/bin/python3

""" Client for the dr-phil mobile app

----------------

requires proper .eng setup as described in the readme.md 


https://firestore.googleapis.com/v1/projects/drphil-bdadb/databases/(default)/documents/robots/test1
"""
import requests
import sys
from config import RID,ROS_WS_HOST,ROS_WS_PORT
from time import sleep
import json
import roslibpy

class CmdListener():
    BASE_URL_RDATA = "https://firestore.googleapis.com/v1/projects/drphil-bdadb/databases/(default)/documents/robots"

    def __init__(self,robot_id,ros_port,ros_host) -> None:
        self.robot_id = robot_id
        self.data = None
        self.schedule = None 
        self.schedule_hash = None
        self.ros_port = ros_port
        self.ros_host = ros_host

        print("Initializing ros connection")
        self.ros = roslibpy.Ros(host=ros_host,port=ros_port)
        self.ros.run()
        self.talker = roslibpy.Topic(self.ros, '/app/rdata', 'std_msgs/String')


    def get_rdata(self) -> dict:
        try:
            url = "{}/{}".format(CmdListener.BASE_URL_RDATA,self.robot_id)
            print("GET: {}".format(url))
            response = requests.get(url).content

            data = json.loads(response)
            self.data=response
            hash_string = hash(json.dumps(data,sort_keys=True,ensure_ascii=True))
            self.schedule = data
            self.schedule_hash = hash_string

            return data
        except Exception as e:
            print(e)
            return {}

    def data_valid(self,data):
        if data is None:
            return False

        fields = data.get("fields",None)
        if fields is not None:
            rid = fields.get("rid",None)
            if rid is not None:
                string = rid.get("stringValue",None)
                if string == self.robot_id:
                    return True
        
        return False




    def spin(self):
        
        if not self.ros.is_connected:
            print("ros disconnected, exitting")
            self.ros.terminate()
            sys.exit(1)

        prev_hash = self.schedule_hash
        data = self.get_rdata()

        period = 5
        if self.data_valid(data):
            period = 60
            print("Received valid schedule with hash: {}".format(self.schedule_hash))

            if prev_hash != self.schedule_hash:
                self.forward_schedule()
        else:
            print ("Received invalid data: {}".format(data))


        sleep(period)

    def forward_schedule(self):
        print("Schedule changed, forwarding to dr-phil")
        # pretty print with json
        print("New schedule: {}".format(json.dumps(self.schedule,sort_keys=True,indent=4)))
        self.talker.publish(roslibpy.Message({'data':str(self.data)}))

if __name__ == "__main__":

    listener = CmdListener(RID,ROS_WS_PORT,ROS_WS_HOST)

    try:
        while(True):
            listener.spin()
    except Exception as E:
        print(E)
        sys.exit(1)
