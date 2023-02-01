""" Command-line simulator for a simple 2D robot.

Author: Andrew Rozniakowsi
Version: 1/23/15

"""
import sys

class RobotSim(object):
    """Simulation of a simple 2D robot. """

    def __init__(self):
        """ The robot always begins at (0,0). """
        self.x = 0
        self.y = 0

    def execute_program(self, file_name):
        """ Read and execute the program stored in the indicated file. 
        The robots x and y positions will be updated. 
        No return value. 
        """
        definition = []
        f = open(file_name) 
        lines_array = f.readlines()
        
        for i in range(len(lines_array)):
            if lines_array[i].startswith('def'):
                string = lines_array[i]
                rule_name = string[4:]
                for x in range(i + 1, len(lines_array)):
                    if lines_array[x] == 'end\n':   
                        for z in range(i + 1, x):
                            definition.append(lines_array[z])  
                rule = macro(rule_name, definition)                          
                
      
    def get_location(self):
        """ Return the robot's current location as an (x, y) tuple. """
        return self.x, self.y

class macro(object):

    """Creates an object for each macro."""
    def __init__(self, rule_name, definition):
        self.rule_name = rule_name
        self.definition = definition

    """def read_macro(self)"""
        
def main():
    """ Simulate the execution of the robot on a program.  

    usage: robot_sim.py PROGRAM_FILE_NAME

    """
    robot = RobotSim()
    robot.execute_program(sys.argv[1])
    print "ROBOT FINAL POSITION: ", robot.get_location()    

if __name__ == "__main__":
    main()
