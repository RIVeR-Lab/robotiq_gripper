#! /usr/bin/env python

""" ROS Node to control Robotiq AirPick through URCap """
# Based on: https://github.com/felixvd/robotiq/blob/56ccac8cffd1e4411ae761351cc5b048c15fa132/robotiq_control/src/robotiq_control/cmodel_urcap.py
# and: https://dof.robotiq.com/discussion/1962/programming-options-ur16e-2f-85#latest
# The communication is established through the port 63352 opened by Robotiq gripper URCap to receive URScript.
# With this method, we can control the gripper while it's attached to the UR robot.
# Other options could be MODBUS RTU (Direct USB connection to PC needed), RS-485 Tool communication (Special I/O coupling needed)


import socket 
import rospy
import threading
import time

class Register:
    """
    Registers provided by Robotiq to control the gripper
    """
    # Input/Output (readable/writtable) "registers".
    ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
    GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe), 0 to pause, 1 to execute
    ATR = 'ATR'  # atr : auto-release (emergency slow move)
    ADR = 'ADR'  # adr : auto-release direction (open(1) or close(0) during auto-release)
    FOR = 'FOR'  # for : force (0-255). Used for minimum pressure in vacuum gripper
    SPE = 'SPE'  # spe : speed (0-255). Used for timeout in vacuum gripper
    POS = 'POS'  # pos : position (0-255), 0 = open. Used for maximum pressure in vacuum gripper
    MOD = 'MOD'  # mod : mode (0 = auto mode, 1 = advanced mode).
    ## The shutoff distance is ignored 

    # Input only registers 
    STA = 'STA'  # status (0 = not activated, 1 = operational)
    OBJ = 'OBJ'  # object detection (0 = unkown, 1 = object detected(min), 2 = object detected(maximum), 3 = no object detected)
    FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)

    ENCODING = 'ASCII' # 'UTF-8' works too

class Gripper(object):
    def __init__(self, address):
        self.command_lock = threading.Lock()
        self.socket = None
        self.connect(address)

    def connect(self, host, port = 63352, socket_timeout = 2.0):
        """
        Connect the gripper to UR robot at given port exposed by URCap
        """
        rospy.loginfo('Gripper connecting to {}:{}'.format(host,port)) 
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((host, port))
            self.socket.settimeout(socket_timeout)
        except OSError as error:
            rospy.logerr("Error connecting to {}: {}".format(port, error.strerror))
            exit(-1)

        rospy.loginfo("Gripper connected successfully")
    
    def disconnect(self):
        self.socket.close()
        rospy.loginfo("Gripper connection closed")

    def vacuum_grip(self):
        return self.vacuum_grip_raw(False, 60, 40, 1000)

    def vacuum_grip_raw(self, advanced_mode, max_pressure, min_pressure, timeout):
        """
        Send full request grip command
        """
        # Reset gripper, clear fault state
        self._req_set_var("ACT", 0)
        self._req_set_var("GTO", 0)
        self._req_set_var("ACT", 1)

        # Turn on/off advanced mode
        if advanced_mode:
            self._req_set_var("MOD", 1)
        else:
            self._req_set_var("MOD", 0)

        # Set maximum/minimum pressure and timeout
        cmd_dict = {}
        cmd_dict['POS'] = self.scale(max_pressure, [0, 100], [100, 0])
        cmd_dict['FOR'] = self.scale(min_pressure, [0, 100], [100, 0])
        cmd_dict['SPE'] = self.scale(timeout, [0, 25500], [0, 255])
        if self._req_set_vars(cmd_dict):
            # Execute gripping
            self._set_gto_and_wait(1)
            return True
        else:
            return False

    def vacuum_release(self):
        return self.vacuum_release_raw(False, True)

    def vacuum_release_raw(self, advanced_mode, wait_for_object_released):
        """
        Send full request release command
        """
        max_pressure = 255
        min_pressure = 0
        timeout = 255

        # Reset gripper, clear fault state.
        # Because release always happens after grip (theoretically), we don't need to reset the states
        # self._req_set_var("GTO", 0)
        # self._req_set_var("ACT", 1)

        # Turn on/off advanced mode
        if advanced_mode:
            self._req_set_var("MOD", 1)
        else:
            self._req_set_var("MOD", 0)

        # Set maximum/minimum pressure and timeout
        cmd_dict = {}
        cmd_dict['POS'] = self.scale(max_pressure, [0, 255], [0, 255])
        cmd_dict['FOR'] = self.scale(min_pressure, [0, 255], [0, 255])
        cmd_dict['SPE'] = self.scale(timeout, [0, 255], [0, 255])
        self._req_set_vars(cmd_dict)
        self._set_gto_and_wait(1)

        if wait_for_object_released:
            while (self._req_is_object_detected()):
                self._req_set_vars(cmd_dict)
                self._set_gto_and_wait(1)
                time.sleep(0.5)
        
        return True

    def _set_gto_and_wait(self, val):
        self._req_set_var("GTO", val)
        while(self._req_get_var("GTO") != val):
            time.sleep(0.008)

    def _req_set_var(self, command, value):
        """
        Set value for single variable
        """
        return self._req_set_vars({command: value})

    def _req_set_vars(self, cmd_dict):
        """
        Send socket request to set variable

        Params
            command: a dictionary containing both the variable name and value
        """
        with self.command_lock:
            cmd = "SET"
            for variable, value in cmd_dict.items():
                cmd += " " + variable + " " + str(int(value))
            
            cmd += "\n"
            rospy.logdebug("send command: {}".format(cmd))
            self.socket.sendall(cmd.encode(Register.ENCODING))
            resp = self.socket.recv(1024)

        return self._is_ack(resp)
    
    def _req_get_var(self, command):
        """
        Send socket request to get variable
        params
            command: the name string of the variable
        """

        with self.command_lock:
            cmd = "GET " + command + "\n"
            self.socket.sendall(cmd.encode(Register.ENCODING))
            resp = self.socket.recv(1024)
        # expect data of the form 'VAR x', where VAR is an echo of the variable name, and X the value
        # note some special variables (like FLT) may send 2 bytes, instead of an integer. We assume integer here 
        var_name, value_str = resp.decode(Register.ENCODING).split()
        if var_name != command:
            raise ValueError("Unexpected response " + str(command) + " does not match '" + command + "'")
        value = int(value_str)
        return value

    def _is_ack(self, resp):
        return resp == b"ack"

    def _req_is_object_detected(self):
        """
        Check whether the object is detected
        """
        return self._req_get_var("OBJ")

    def scale(self, value, rawRange, scaledRange):
        """
        Copied from URScript implementation to scale the input value
        """
        def computeSlope(inputRange, outputRange):
            outputRangeDelta = outputRange[1] - outputRange[0]
            inputRangeDelta = inputRange[1] - inputRange[0]
  
            if (inputRangeDelta == 0):
                return 0
            else:
                return outputRangeDelta / inputRangeDelta
  
        def computeIntercept(slope, inputRange, outputRange):
            return outputRange[0] - (slope * inputRange[0])
  
        def clipScaledValue(outputScaledValue, outputRange):
            if (outputRange[0] < outputRange[1]):
                return clipWhenLowerLimitIsLessThanHigher(outputScaledValue, outputRange)
            else:
                return clipWhenLowerLimitIsGreaterThanHigherLimit(outputScaledValue, outputRange)
  
        def clipWhenLowerLimitIsGreaterThanHigherLimit(outputScaledValue, outputRange):
            if (outputScaledValue < outputRange[1]):
                return outputRange[1]
            elif (outputScaledValue > outputRange[0]):
                return outputRange[0]
            else:
                return outputScaledValue
  
        def clipWhenLowerLimitIsLessThanHigher(outputScaledValue, outputRange):
            if (outputScaledValue < outputRange[0]):
                return outputRange[0]
            elif (outputScaledValue > outputRange[1]):
                return outputRange[1]
            else:
                return outputScaledValue
  
        slope = computeSlope(rawRange, scaledRange)
        intercept = computeIntercept(slope, rawRange, scaledRange)
        scaledValue = slope * value + intercept
        return clipScaledValue(scaledValue, scaledRange)


        


