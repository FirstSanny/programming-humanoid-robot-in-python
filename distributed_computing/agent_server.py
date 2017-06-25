'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import threading
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    server = SimpleXMLRPCServer(("localhost", 8000), requestHandler=RequestHandler)
    server.register_introspection_functions()
    server.register_multicall_functions()()
    print "Server started at localhost:8000"
    
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.target_joints[joint_name]
    server.register_function(get_angle, 'get_angle')
    print "registered 'get_angle'"
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle
    server.register_function(set_angle, 'set_angle')
    print "registered 'set_angle'"

    def get_posture(self):
        '''return current posture of robot'''
        # we don't inherit from PostureRecognition.. can this even work?
        return self.posture
    server.register_function(get_posture, 'get_posture')
    print "registered 'get_posture'"

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        # how to block here?
    server.register_function(execute_keyframes, 'execute_keyframes')
    print "registered 'execute_keyframes'"
        

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name]
    server.register_function(get_transform, 'get_transform')
    print "registered 'get_transform'"

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(self, effector_name, transform)
    server.register_function(set_transform, 'set_transform')
    print "registered 'set_transforms'"
    
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    print "server started serving via thread"

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

