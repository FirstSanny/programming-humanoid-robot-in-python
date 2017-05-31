'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
import numpy as np

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.start = 0
        self.end = 0
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        
        (names, times, keys) = keyframes
        self.curTime = self.perception.time
        timeDiff = self.curTime - self.start

        if self.end == 0:
            for valueIndex in range(0, len(names)):
                nameAtIndex = names[valueIndex]
                timesAtIndex = times[valueIndex]
                keysAtIndex = keys[valueIndex]
                if timesAtIndex[len(timesAtIndex)-1]> self.end:
                   self.end = timesAtIndex[len(timesAtIndex)-1]


        if self.end < timeDiff or self.start == 0:
            self.start = self.curTime
            timeDiff = self.curTime - self.start


        for valueIndex in range(0, len(names)):
            nameAtIndex = names[valueIndex]
            timesAtIndex = times[valueIndex]
            keysAtIndex = keys[valueIndex]

            if timeDiff < timesAtIndex[0]:
                
                if not nameAtIndex in self.perception.joint:
                    continue

                p0 = (timeDiff, self.perception.joint[nameAtIndex])
                p1 = (timesAtIndex[0] + keysAtIndex[0][1][1], keysAtIndex[0][0] + keysAtIndex[0][1][2])
                p2 = (timesAtIndex[0], keysAtIndex[0][0]) #
                p3 = (timesAtIndex[0] + keysAtIndex[0][1][1], keysAtIndex[0][0] + keysAtIndex[0][1][2])

                t = (timeDiff - self.start) / (timesAtIndex[0] - self.start)

               
            else:

                if timeDiff >= timesAtIndex[-1]:
                    continue

                i = 0
                while timeDiff > timesAtIndex[i]:
                    i += 1

                i = i - 1

                p0 = (timesAtIndex[i], keysAtIndex[i][0])
                p1 = (timesAtIndex[i] + keysAtIndex[i][2][1], keysAtIndex[i][0] + keysAtIndex[i][2][2])
                p2 = (timesAtIndex[i+1], keysAtIndex[i+1][0]) 
                p3 = (timesAtIndex[i+1] + keysAtIndex[i+1][1][1], keysAtIndex[i+1][0] + keysAtIndex[i+1][1][2]) 

                t = (timeDiff - timesAtIndex[i]) / (timesAtIndex[i+1] - timesAtIndex[i])
                
            # t in [0,1]    
            if t > 1.:
                t = 1.
            elif t < 0.:
                t = 0.

            # bezier
            target_joints[nameAtIndex] = ((1-t) ** 3) * p0[1] + 3 * ((1-t) ** 2) * t * p1[1] + 3 * (1-t) * (t ** 2) * p2[1] + (t**3) * p3[1]
    

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
