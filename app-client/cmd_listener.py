#!/usr/bin/python3

""" Client for the dr-phil mobile app

----------------

requires proper .eng setup as described in the readme.md 


https://firestore.googleapis.com/v1/projects/drphil-bdadb/databases/(default)/documents/robots/test1
"""
import requests
import sys
from config import RID,ROS_WS_HOST,ROS_WS_PORT
from data import CleaningTime
from time import sleep,time
import json
import roslibpy
import os
from copy import deepcopy

class CmdListener():
    BASE_URL_RDATA = "https://firestore.googleapis.com/v1/projects/drphil-bdadb/databases/(default)/documents/robots"

    def __init__(self,robot_id,ros_port,ros_host) -> None:
        self.robot_id = robot_id
        self.data = None
        self.schedule = None 
        self.schedule_hash = None
        self.ros_port = ros_port
        self.ros_host = ros_host

        self.update_period = 60 * 60
        self.publish_period = 5
        self.last_update_time = 0
        self.last_publish_time = 0

        self.cache_filename= "sched-cached.json"
        print("Initializing ros connection")
        self.ros = roslibpy.Ros(host=ros_host,port=ros_port)
        self.ros.run()
        self.talker = roslibpy.Topic(self.ros, '/app/rdata', 'dr_phil_hardware/CleaningSchedule')
        
        # try to retrieve cached schedule
        (sched,hashstored) = self.retrieve_stored_schedule()
        if sched is not None:
            print("found stored schedule")
            self.schedule = sched 
            self.schedule_hash = hashstored
            self.skip_next_update = True 
        else:
            self.skip_next_update = False

    def hash_response(self,response):
        return hash(json.dumps(response,sort_keys=True,ensure_ascii=True))

    def store_schedule(self):
        print("storing schedule")
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.cache_filename)

        copy = deepcopy(self.schedule)
        copy["hash"] = self.schedule_hash

        with open(path, 'w') as outfile:
            json.dump(copy, outfile,indent=4)
        

    def retrieve_stored_schedule(self):
        """returns the stored schedule if it exists in a tuple with its hash or a double None

        Returns:
            [type]: [description]
        """
        print("retrieving stored schedule")
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.cache_filename)

        try:
            with open(path, 'r') as outfile:
                sched =  json.load(outfile)
                hash_stored = sched["hash"]
                del sched["hash"]

                return (sched,hash_stored)
        except Exception as E:
            return None,None


        

    def get_rdata(self) -> dict:
        try:
            url = "{}/{}".format(CmdListener.BASE_URL_RDATA,self.robot_id)
            print("GET: {}".format(url))
            response = requests.get(url).content
            data = json.loads(response)
            
            self.data=response
            hash_string = self.hash_response(data)
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


    def to_rosmsg(self,data : dict):
        
        weekdays = CleaningTime(data["fields"]["weekdays"])
        weekdays = weekdays.get_dict()

        weekends = CleaningTime(data["fields"]["weekends"]).get_dict()
        return_time = data["fields"]["returnTime"].get("stringValue","0")
        msg = {
            "return_time":return_time,
            "weekdays": weekdays,
            "weekends": weekends,
            }
        return roslibpy.Message(msg)

    def update_schedule(self):
        time_now = time()
        if not time_now - self.last_update_time > self.update_period:
            return 
        else:
            self.last_update_time = time_now

        if self.skip_next_update:
            self.skip_next_update = False
            return 

        print("Checking for schedule changes")

        prev_hash = self.schedule_hash
        data = self.get_rdata()

        if self.data_valid(data):
            print("Received valid schedule with hash: {}".format(self.schedule_hash))
            if prev_hash != self.schedule_hash:
                print("Schedule has changed")
                print("New schedule: {}".format(json.dumps(self.schedule,sort_keys=True,indent=4)))
                self.store_schedule()
        else:
            print ("Received invalid data: {}".format(data))


    def publish_schedule(self):
        time_now = time()
        if not time_now - self.last_publish_time > self.publish_period:
            return 
        else:
            self.last_publish_time = time_now

        if self.schedule is None:
            return 

        print("Publishing schedule")

        self.forward_schedule()

    def spin(self):

        if not self.ros.is_connected:
            print("ros disconnected, exitting")
            self.ros.terminate()
            sys.exit(1)

        self.update_schedule()
        self.publish_schedule()


        sleep(1)

    def forward_schedule(self):
        self.talker.publish(self.to_rosmsg(self.schedule))

if __name__ == "__main__":

    listener = CmdListener(RID,ROS_WS_PORT,ROS_WS_HOST)

    try:
        while(True):
            listener.spin()
    except Exception as E:
        print(E)
        sys.exit(1)
