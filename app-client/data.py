
import json

def get_nested(data : dict,keys,default):
    """retrieves the nested key or the default if any key doesnt exist

    Args:
        keys ([type]): [description]
        default ([type]): [description]
    """
    if len(keys) == 0:
        return data
        
    d = data.get(keys[0],default)
    if d != default:
        return get_nested(d,keys[1:],default)





class CleaningTime():

    def __init__(self,data):
        """ creates cleaning time instance from json received via api """

        (hon,mon) = (0,0)
        (hoff,moff) = (0,0)
        interval = 0
        values =  get_nested(data,["arrayValue","values"],None)
        if values is not None and len(values) == 3:
            string_on = values[0]["stringValue"]
            string_off = values[1]["stringValue"]
            interval = values[2]["stringValue"]

            (hon,mon) = tuple([int(x) for x in string_on.split(":")])
            (hoff,moff) = tuple([int(x) for x in string_off.split(":")])
            interval = int(interval)

        self.hour_on = hon
        self.minute_on = mon
        self.hour_off = hoff
        self.minute_off = moff
        self.interval = interval
    def get_dict(self):
        return {
            "hour_on":self.hour_on,
            "hour_off":self.hour_off,
            "minute_off":self.minute_off,
            "minute_on":self.minute_on,
            "interval":self.interval
        }

class ReturnTime():

    def __init__(self,data):
        """ creates cleaning time instance from json received via api """

        (yyyy,mm,dd) = (0,0,0)
        (h,m) = (0,0)
        return_type = ""
        return_duration = 0
        string_value_return_time =  get_nested(data,["returnTime","stringValue"],None)
        string_value_return_duration = get_nested(data,["returnDuration","stringValue"],None)
        if string_value_return_time is not None and string_value_return_duration is not None:
            
            if "" == string_value_return_time:
                return_type = ""
            elif "ASAP" in string_value_return_time:
                return_type = "ASAP"
            else:
                (date,time) = string_value_return_time.split(" ")
                (h,m) = tuple([int(x) for x in time.split(":")])
                (yyyy,mm,dd) = tuple([int(x) for x in date.split("-")])
                return_type = "TIME"
                
            if "REST" in string_value_return_duration:
                return_duration = -1 # special value for "infinite" time
            elif not "" == string_value_return_duration:
                return_duration = int(string_value_return_duration)
            else:
                return_duration = 0 # zero is no command 

        self.year  = yyyy
        self.month = mm
        self.day = dd
        self.hour = h
        self.minute = m
        self.return_type = return_type
        self.return_duration = return_duration
    def get_dict(self):
        return {
            "year":self.year,
            "month":self.month,
            "day":self.day,
            "hour":self.hour,
            "minute":self.minute,
            "return_type":self.return_type,
            "return_duration":self.return_duration
        }


class Status():

    def __init__(self,dict_srv_request) -> None:
        self.request = dict_srv_request

    def get_dict(self):

        data = {"fields":{
            "status":{
                "integerValue":str(self.request["status"])
            },
            "battery":{
                "doubleValue":self.request["battery_level"]
            },
            "disinfectant":{
                "doubleValue":self.request["disinfectant_level"]
            }
        }}

        return data