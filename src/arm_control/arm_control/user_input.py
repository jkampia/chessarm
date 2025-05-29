import math as m

class UserInputHandler:


    def __init__(self, robot):
        
        self.robot = robot
        
        self.command_list = ["help - Show this help message", 
                "home - Move robot to home position",
                "print - print robot end effector position",
                "ik - Move to an x,y,z,r,p position in space",
                "fk - Enter a joint angle in radians"]
    

    def parseUserInput(self, user_input):

        list = user_input.split()

        if list[0] == "help":
            for command in self.command_list:
                print(command)

        elif list[0] == "home":
            self.robot.joint_angles = self.robot.home_angles.copy()

        elif list[0] == "print":
            self.printListFormatted(self.robot.joint_coordinates[5])

        elif list[0] == "fk":
            if len(list) != 3:
                print("Invalid command. Usage: fk <joint> <angle>")
                return
            joint = int(list[1]) 
            new_angle = m.radians(float(list[2]))
            self.robot.joint_angles[joint] = new_angle

        elif list[0] == "ik":
            if len(list) < 4 or len(list) > 6:
                print("Invalid command. Usage: ik <x> <y> <z> <r> <p>")
                return
            x = float(list[1])
            y = float(list[2])
            z = float(list[3])
            r = float(list[4]) if len(list) == 5 else 0.0
            p = float(list[5]) if len(list) == 6 else 0.0
            self.robot.joint_angles = self.robot.solveIK([x, y, z, r, p])
            self.printListFormatted(self.robot.joint_angles)

    
    def printListFormatted(self, list):
        formatted = [f"{x:.3f}" for x in list]
        print(formatted)

        

    
    