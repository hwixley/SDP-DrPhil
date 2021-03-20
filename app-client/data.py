


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
