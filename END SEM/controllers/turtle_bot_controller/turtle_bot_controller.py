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

    goal_pos=[0.7, -1.23, 0.0]
    source_pos=[-0.7, -1.2, 0.0]

    turt_node= robot.getFromDef('turt_bot')
    turt_trans_field= turt_node.getField('translation')
    turt_rot_field= turt_node.getField('rotation')
    
    create0_node= robot.getFromDef('create0')
    create0_trans_field= create0_node.getField('translation')
    create0_rot_field= create0_node.getField('rotation')
    
    create1_node= robot.getFromDef('create0')
    create1_trans_field= create1_node.getField('translation')
    create1_rot_field= create1_node.getField('rotation')
    
    print("World")

    left_motor= robot.getDevice('left wheel motor') 
    right_motor= robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf')) 
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf')) 
    right_motor.setVelocity(0.0)
    
    turt_lidar= robot.getDevice("LDS-01")
    turt_lidar.enable(timestep)
    turt_lidar.enablePointCloud() 
    
    history_pos=[]
    # x=[]
    
    turt_rot_field.setSFRotation([0,0,-1,-5.30718e-06])
    while robot.step(timestep) != -1:        
        range_image= turt_lidar.getRangeImage()
        print(len(range_image))
        #print(type(range_image[0]))
        print(range_image.count(float('inf')))
        print(range_image)
        
        left_speed= 0.5*max_speed
        right_speed= 0.5*max_speed
     
        # get the position of turtle, create0 and create1 from the supervisor method and store it
        pos= turt_node.getPosition()
        rot= turt_rot_field.getSFRotation()
        
        create0_pos= create0_node.getPosition()
        create0_rot= create0_rot_field.getSFRotation()
        
        create1_pos= create1_node.getPosition()
        create1_rot= create1_rot_field.getSFRotation()
        
        # history_pos.append(pos)
        
        dist_turt_create0= distance(pos, create0_pos)
        dist_turt_create1= distance(pos, create1_pos)
        
        # calculating the line from current position to target position
        line_s_g= line(pos,goal_pos)
        print("line specifications- ", line_s_g)
        
        # calculating the current and required angle
        if len(history_pos)>0:
            current_angle= angle(pos, history_pos)
            print("current angle- ", current_angle)
            target_angle= angle(pos, goal_pos)
            print("target angle- ", target_angle)
            
            # if( range_image.count(float('inf'))<=300 ):
                # print("hi")
                # left_speed= -max_speed*0.5
                # right_speed= max_speed*0.5
            
            if(dist_turt_create0<0.5 or dist_turt_create1<0.5):
                left_speed= 0
                right_speed= 0
            if( range_image[55]==float('inf')  and range_image.count(float('inf'))<=215 ):
                print("turn left")
                left_speed= -max_speed
                right_speed= max_speed
            if( current_angle<-1.6 and target_angle<-1.25):
                print("turn right")
                turt_rot_field.setSFRotation([rot[0], rot[1], rot[2], -0.0412753])  
            if(current_angle>3.00 and target_angle<-1.54):
                turt_rot_field.setSFRotation([-0.003458, -0.0676777, -0.997701, target_angle])   
            # if( pos[0]<-0.11 and pos[0]>-0.12):
                # print("turn right")
                # turt_rot_field.setSFRotation([rot[0], rot[1], rot[2], -0.0412753])     
            # if( target_angle<-1.22 and target_angle>-1.27 and range_image[200]==float('inf') and range_image.count(float('inf'))>=320):
                # left_speed= max_speed*0.5
                # right_speed= -max_speed*0.5
            # if( target_angle<-1.31 and range_image.count(float('inf'))>=340):
                # turt_rot_field.setSFRotation([rot[0], rot[1], rot[2], -0.0412753])
            # if( target_angle<-1.31 and range_image.count(float('inf'))>=340):
                # turt_rot_field.setSFRotation([rot[0], rot[1], rot[2], -0.0412753])         
            # if(range_image.count(float('inf'))>=319 and range_image[31]!=float('inf')):
                # turt_rot_field.setSFRotation([rot[0], rot[1], rot[2], target_angle])                  
            # else:
                # left_speed= max_speed*0.5
                # right_speed= max_speed*0.5           
        
        left_motor.setVelocity(left_speed) 
        right_motor.setVelocity(right_speed)
        
        # stop if you reach the target location
        if(distance(pos, goal_pos)<0.1):
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print("target reached")
            break

        print("current position- ", pos)
        print("rotation- ", rot)
        print("previous position- ", history_pos)
        history_pos= pos
        # i+=1
      

if __name__ == "__main__":
    robot= Supervisor() 
    run_robot(robot)