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
    
def run_robot(robot):
    #get the time step of the current world. 

    timestep= int(robot.getBasicTimeStep())
    max_speed= 6.28

    print("Hello")

    goal_pos=[-0.12, -0.11, 0.0]
    source_pos=[1.12, -0.13, 0.0]

    epuck_node= robot.getFromDef('e_puck')
    epuck_trans_field= epuck_node.getField('translation')
    epuck_rot_field= epuck_node.getField('rotation')
    
    print("World")

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
        
        for i in range(8):
            print("sensor: ps{} , value: {}".format(i, dist_sensor[i].getValue()))
        
        # wall logic
        front_wall= dist_sensor[0].getValue()>80 or dist_sensor[7].getValue()>80 or dist_sensor[6].getValue()>80
        right_wall= dist_sensor[2].getValue()>80 or dist_sensor[1].getValue()>80    
        
        # front_wall= dist_sensor[7].getValue()>80 
        # right_wall= dist_sensor[2].getValue()>80 
        
        # front_wall= dist_sensor[0].getValue()>80 or dist_sensor[7].getValue()>80 or dist_sensor[6].getValue()>80
        # right_wall= dist_sensor[2].getValue()>80 or dist_sensor[3].getValue()>80 or dist_sensor[1].getValue()>80
        
        # setting up speed of motors
        left_speed= 0.5*max_speed
        right_speed= 0.5*max_speed
     
        # get the position of epuck from the supervisor method and store it
        pos= epuck_node.getPosition()
        #rot= epuck_node.getRotation()
        rot= epuck_rot_field.getSFRotation()
        # history_pos.append(pos)
        
        # calculating the line from current position to target position
        line_s_g= line(pos,goal_pos)
        print("line specifications- ", line_s_g)
        
        # calculating the current and required angle
        if len(history_pos)>0:
            current_angle= angle(pos, history_pos)
            print("current angle- ", current_angle)
            target_angle= angle(pos, goal_pos)
            print("target angle- ", target_angle)
            
            # if(front_wall):
                # left_speed= -max_speed
                # right_speed= max_speed
            # program logic in case of wall detection
            angle_criteria= (abs(target_angle - current_angle) < 3.0 or abs(target_angle - current_angle) > 5.0)
            if( angle_criteria and not front_wall and not right_wall):
                # if(target_angle > current_angle):
                    print("turn right towards goal")
                    epuck_rot_field.setSFRotation([rot[0], rot[1], rot[2], target_angle])
                    # epuck_rot_field.setSFRotation([rot[0], rot[1], rot[2], target_angle])
                    # right_speed = -max_speed
                    # #left_speed = 0.5*max_speed
                # else:
                    # print("turn left to avoid obstacle")
                    # left_speed = -0.25*max_speed
                    # #right_speed = 0.5*max_speed
            
            elif(front_wall):
                print("turn left to avoid front wall")
                # epuck_rot_field.setSFRotation([rot[0], rot[1], rot[2], -1.5716])
                left_speed = -0.5*max_speed
                right_speed = 0.5*max_speed
            
            else: 
                left_speed = 0.5*max_speed
                right_speed = 0.5*max_speed
                
            # if(front_wall):
                # left_speed = -max_speed
                # right_speed = max_speed
            # else:
                # if(right_wall):
                    # left_speed= max_speed
                    # right_speed= max_speed
                # if( not front_wall and not right_wall):
                    # epuck_rot_field.setSFRotation([rot[0], rot[1], rot[2], target_angle])         
        
        left_motor.setVelocity(left_speed) 
        right_motor.setVelocity(right_speed)
        
        # stop if you reach the target location
        if(distance(pos, goal_pos)<0.1):
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print("target reached")
            break
            
        # if(pos==[0.9009537942597012, -0.12982638643164254, -6.394534785326804e-05]):
            # left_speed=0
            # right_speed=0
            # left_motor.setVelocity(left_speed) 
            # right_motor.setVelocity(right_speed)
            # print("exit!!!")
            # break

        print("current position- ", pos)
        print("rotation- ", rot)
        print("previous position- ", history_pos)
        history_pos= pos
        # i+=1
      

if __name__ == "__main__":
    robot= Supervisor() 
    run_robot(robot)