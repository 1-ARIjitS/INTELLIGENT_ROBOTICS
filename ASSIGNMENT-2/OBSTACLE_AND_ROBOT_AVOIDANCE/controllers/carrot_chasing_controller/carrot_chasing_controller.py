from controller import Robot, Supervisor, Field
import math
from sympy import Polygon, Point, Ellipse
#create the Robot instance.
# robot= Robot()

def line(source, goal):
    line={}
    
    # line parallel to x axis slope is 
    if((goal[1] - source[1])==0):
        line['a']= 0.0
        line['b']= 1.0
        line['c']= -source[1]
        
    # line parallel to y axis slope infinite
    elif((goal[0] - source[0])==0):
        line['a']= 1.0
        line['b']= 0.0
        line['c']= -source[0]
        
    # any general line 
    else:    
        line['a']= (goal[1] - source[1])
        line['b']= (source[0] - goal[0])
        line['c']= (source[1]*goal[0] - source[0]*goal[1])
    
    return line
    
def angle(source, goal):
    y= goal[1]-source[1]
    x= goal[0]-source[0]
    return math.atan2(y,x)
    
def distance(source, goal):
    x= goal[0]-source[0]
    y= goal[1]-source[1]
    dist= (x**2 + y**2)**0.5
    return dist
        
def sensing_radius(source):
    x1= source[0]
    y1= source[1]
    
    circle= {}
    circle['A']= 1.0
    circle['B']= 1.0 
    circle['C']= 2*x1
    circle['D']= 2*y1
    circle['E']= (x1**2 + y1**2 + 4)
    
    #equation of the circle is Ax^2 + By^2 + Cx + Dy + E = 0
    return circle
    
def run_robot(robot):
    #get the time step of the current world. 

    timestep= int(robot.getBasicTimeStep())
    max_speed= 6.28
    req_speed=0.5

    # print("Hello")

    # goal_pos=[-0.92, 1.1, 0.0]
    # source_pos=[1.05, -0.884, 0.0]

    epuck_node= robot.getFromDef('e_puck_sec1')
    epuck_trans_field= epuck_node.getField('translation')
    epuck_rot_field= epuck_node.getField('rotation')
    
    # print("World")

    left_motor= robot.getDevice('left wheel motor') 
    right_motor= robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf')) 
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf')) 
    right_motor.setVelocity(0.0)
    
    # reading values of dstance sensors
    dist_sensor=[]
    for i in range(8):
        sensor_name= 'ps'+str(i)
        dist_sensor.append(robot.getDevice(sensor_name))
        dist_sensor[i].enable(timestep)
        
    #Main loop: -perform simulation steps until Webots is stopping the controller 
    # i=0
    history_pos=[]
    # x=[]
     
    while robot.step(timestep) != -1:
        #print("hello")
        # printing distance sensor readings
        
        # for i in range(8):
            # print("sensor: ps{} , value: {}".format(i, dist_sensor[i].getValue()))
        
        # wall logic
        front_wall= dist_sensor[0].getValue()>80 or dist_sensor[7].getValue()>80 or dist_sensor[6].getValue()>80
        right_wall= dist_sensor[2].getValue()>80 or dist_sensor[1].getValue()>80    

        # setting up speed of motors
        left_speed= 0.5*max_speed
        right_speed= 0.5*max_speed
     
        # get the position of epuck from the supervisor method and store it
        pos= epuck_node.getPosition()
        rot= epuck_rot_field.getSFRotation()
                
        # calculating the current and required angle
        if len(history_pos)>0:
            line_sec1= line(pos,history_pos)
            # print("line specifications for sec robot 1- ", line_sec1)
            
            circle_sec1= sensing_radius(pos)
            # print("circle specifications for sec robot 1- ", circle_sec1)
            
            if(front_wall):
                print("turn left to avoid front wall")
                # epuck_rot_field.setSFRotation([rot[0], rot[1], rot[2], -1.5716])
                left_speed = -0.5*max_speed
                right_speed = 0.5*max_speed
            else: 
                left_speed = 0.5*max_speed
                right_speed = 0.5*max_speed         
        
        left_motor.setVelocity(left_speed) 
        right_motor.setVelocity(right_speed)
        
        # stop if you reach the target location
        if(pos[0]<-0.65):
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print("target reached")
            break
           
        # print("current position for sec robot 1- ", pos)
        #print("rotation for sec robot 1- ", rot)
        # print("previous position for sec robot 1- ", history_pos)
        history_pos= pos
        # print("-----------------------------------------------------------------------------------------------------------------------------------------------")
        # i+=1
      

if __name__ == "__main__":
    robot_sec1= Supervisor() 
    run_robot(robot_sec1)