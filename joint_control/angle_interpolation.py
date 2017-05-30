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

eps = 1e-6

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.start = self.perception.time
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        
        time = self.perception.time - self.start
        (names, times, keys) = keyframes
        
        for i, name in enumerate(names):
            lTimes = times[i]
            
            if time<lTimes[0] or lTimes[-1]<time:
                continue
            
            #get keys
            eIndex = len([x for x in lTimes if x<time])
            sIndex = eIndex - 1
            ekeys = keys[i][eIndex]
            skeys = keys[i][sIndex]
            
            # direct points
            (p0x, p0y) = (lTimes[sIndex], skeys[0])
            (p3x, p3y) = (lTimes[eIndex], ekeys[0])
            # direction points
            (p1x, p1y) = (p0x + skeys[2][1], p0y + skeys[2][2])
            (p2x, p2y) = (p3x + ekeys[1][1], p3y + ekeys[1][2])
            
            # bezier
            bezierMatrix = np.array([[1,0,0,0],[-3,3,0,0],[3,-6,3,0],[-1,3,-3,1]])
            x = np.array([p0x,p1x,p2x,p3x])
            y = np.array([p0y,p1y,p2y,p3y])
            
            #get solutions within error margin
            coefficientsX = np.dot(bezierMatrix, x)
            coefficientsX[0] -= time
            solutions = np.polynomial.polynomial.polyroots(coefficientsX)
            
            solutions = [x.real for x in solutions if -(eps)<=x.real<=1+(eps) and x.imag == 0] 
            
            #possible more solutions cause errror margin
            if len(solutions) > 1: 
                 solutions = np.asarray([(x,np.abs(x-0.5)) for x in solutions],dtype = [("value", float),("distance", float)]) # closest to 0.5 (center of [0,1])
                 solutions = np.sort(solutions, order="distance") #sorts in ascending order according to distance to 0.5
                 t = solutions[0][0]
            else: #one solution
                t = solutions[0]
          
            # t in [0,1]
            if t < 0.: 
                t = 0.
            if t > 1.: 
                t = 1.
              
            #results
            coefficientsY = np.dot(bezierMatrix, y)
            result = np.dot(np.array([1, t, t**2, t**3]),coefficientsY)
            target_joints[name] = result

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
