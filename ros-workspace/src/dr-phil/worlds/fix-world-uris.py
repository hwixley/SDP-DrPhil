import os

pwd = os.path.dirname(os.path.realpath(__file__))
if not pwd.__contains__("/ros-workspace/src/dr-phil/worlds"):
    print("ERROR: Please execute this in the correct directory (<path-to-SDP-repo>/ros-workspace/src/dr-phil/worlds) as specified in the README")
    exit(0)

pwd = pwd.replace("/ros-workspace/src/dr-phil/worlds", "")

fileNames = ["slantRoom", "zShapedRoom", "cShapedRoom", "longCShapedRoom", "spiralShapedRoom"]
worldTypes = ["-wDoor", "-wDoor-wObstacles"]

for fileName in fileNames:
    for worldType in worldTypes:
        if fileName == "spiralShapedRoom" and worldType == "-wDoor-wObstacles":
            exit(0)
        else:
            fName = fileName + worldType + ".world"
            f = open(fName, "r+")
            fileContents = f.read()
            fContents = fileContents.replace("/home/hwixley/Documents/Year3/SDP/GitRepo", pwd)
            f.write(fContents)
            f.close()
