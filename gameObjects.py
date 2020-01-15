###Library for game objects to use in the simulator
import numpy as np
import pygame
import math

#returns angle between two vectors
def angle_between(v1, v2):
    v1_u = v1 / np.linalg.norm(v1)      #unitvector in direction v1
    v2_u = v2 / np.linalg.norm(v2)      #same for v2
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))     #calculate angle, clip keeps the value in [-1,1] so arcos doesnt throw error from rounding

def rotate_vector(v : np.array, angle):
    rot_matrix = np.matrix([[np.cos(angle), -1*np.sin(angle)],
                            [np.sin(angle), np.cos(angle)]])
    v_rotated = v.dot(rot_matrix)
    return v_rotated

#reflects Vector v on surface described by normal unit vector n
def reflect_vector(v, n):
    v_r = v - 2* np.dot(v,n) * n
    return v_r



#base class for generic Object that can be placed on screen and moved around
class GameObject():
    #constructor
    def __init__(self, position : np.array):
        self.position = position                        #position is a Vecor (pos_x, pos_y) stored as np.array
        self.velocity = np.array([0,0])                 #same for velocity (v_x, v_y)
        self.accelaration = np.array([0,0])             #acceleration (a_x, a_y)
        self.rotation = 0                               #rotation as angular velocity (positive is counterclockwise

        #self.mass = mass 
        #mass could be used for calculating acceleration from force. 
        #As mass of robot arm and puk are constant we can use a factor (with a little randomness that occurs in the real world through imperfect material etc)
        #for acceleration after collision.
        #F1=F2 <=> a1*m1 = a2*m2 a1/a2 = m2/m1 = const
        #maybe usefull to implement for other games but not in this project
        #We will ignore friction from air, as distances are short.

    #move position
    def move(self):
        self.velocity = self.velocity + self.accelaration              #increase speed according to accelaration 
        #Timestep einbauen für Skalierung (elapsed_time)
        self.position = self.position + self.velocity                  #move position according to speed
        self.accelaration = self.accelaration * 0                      #reset accelaration for next frame's calculation

    #what happens every frame
    def update(self):
        self.move()
    
    #method for accelerating the object
    def accelerate(self, a : np.array):
        self.accelaration = self.accelaration + a

class Ball(GameObject):    
    #options
    wallCollisionFactor = 0.7          #remaining % of speed on wall contact
    friction = 0.3                     #factor for rotation change on contact

    def __init__(self, position : np.array, radius = 1):
        GameObject.__init__(self, position)
        self.radius = radius
        self.wallCollisionFactor = Ball.wallCollisionFactor
        self.hitLeft = False
        self.hitRight = False

    def update(self, screenSize):
        self.move()
        self.checkBoundary(screenSize)
        
    def render(self,screen):
        try:
            pos = self.position.astype(int)
        except:
            print("haha")
        pygame.draw.circle(screen, (255,255,255), pos, self.radius, 0)    #fill circle (width = 0) white


    #Man könnte noch Rotation und Reibung abhängig von Normalkraft machen um das Modell zu Verbessern

    def checkBoundary(self, screenSize):
        ##collision on left/right wall
        if self.position[0] - self.radius < 1:                      #left edge
            if self.checkGoal(screenSize[1]):                       #check if pos_y is in range for scoring goal
                self.hitLeft = True
                
            self.velocity[0] *= -1*self.wallCollisionFactor         #negate direction => 180° turn
            self.position[0] = self.radius + 2                      #reset position to be inside playing field
            temp_vel = self.velocity[1] *-1                         #temp is lateral velocity relative to wall
            self.velocity[1] -= self.rotation * Ball.friction       #reduce outgoing speed according to rotation and friction
            self.rotation = temp_vel * Ball.friction                #change rotation

        elif self.position[0] + self.radius > screenSize[0] - 1:    #right edge
            if self.checkGoal(screenSize[1]):
                self.hitRight = True

            self.velocity[0] *= -1*self.wallCollisionFactor
            self.position[0] = screenSize[0] - self.radius - 2
            temp_vel = self.velocity[1]
            self.velocity[1] += self.rotation * Ball.friction
            self.rotation = temp_vel * Ball.friction

        ##collision top / bootom
        if self.position[1] - self.radius < 1 :                     #top edge
            self.velocity[1] *= -1*self.wallCollisionFactor
            self.position[1] = self.radius + 2
            temp_vel = self.velocity[0]
            self.velocity[0] += self.rotation * Ball.friction
            self.rotation = temp_vel * Ball.friction
        elif self.position[1] + self.radius > screenSize[1] - 1:    #bottom edge
            self.velocity[1] *= -1*self.wallCollisionFactor
            self.position[1] = screenSize[1] - self.radius - 2
            temp_vel = self.velocity[0]
            self.velocity[0] += self.rotation * Ball.friction
            self.rotation = temp_vel * Ball.friction * -1


    def checkGoal(self,screenHeight):
        if self.position[1] + self.radius < screenHeight / 2 + 60 and self.position[1] - self.radius > screenHeight / 2 - 60:   
            return True
        else:
            return False

class Paddle(GameObject):
    def __init__(self, position, paddleSize):
        GameObject.__init__(self, position)
        self.radius = paddleSize

    def update(self, screenSize, ball):
        #constraints in movement to be implemented
        self.move()
        self.checkBoundary(screenSize)
        self.velocity = self.velocity * 0.98

        #Collision with ball:
        dist = np.linalg.norm(self.position - ball.position)
        if dist <= self.radius + ball.radius:
            a_vector = (ball.position - self.position) / dist    #normalized acceleration vector, orthogonal to contact point
            a_vector = a_vector * np.linalg.norm(self.velocity)*2    #multiply by paddle speed for now
            ##add better physics
            ball.accelerate(a_vector)
            ball.position = ball.position + a_vector * 1.5
            #change ball direction/speed
            pass

    def render(self, screen):
        pygame.draw.circle(screen, (255,255,255), self.position.astype(int), self.radius, 0)    #fill circle (width = 0) white

    def checkBoundary(self, screenSize):
        #stay in field
        if self.position[0] < 0:
            self.velocity[0] *= -1
            self.position[0]=0
        elif self.position[0] > screenSize[0]:
            self.velocity[0] *= -1
            self.position[0]=screenSize[0]

        if self.position[1] < 0:
            self.velocity[1] *= -1
            self.position[1]=0
        elif self.position[1] > screenSize[1]:
            self.velocity[1] *= -1
            self.position[1]=screenSize[1]

class RobotArm(GameObject):
    max_angle = 110 / 360 * 2 * np.pi
    collisionFactor = 0.2                         #remaining % or speed after collision
    friction = 0.2
    #Values of the robot arm
    #Winkelweite = 110 #Grad
    #Winkelgeschwindigkeit = 60°/0,14s +-10%  , errechnet 0.17s mit Beschleunigung bei ausgestrecktem Arm (maximum Widerstandwoment)
    #Diese ist nach ca. 0.03s erreicht in denen 6.2° überstrichen werden, davor konstante beschleunigung
    #Drehmoment_Servo = 3Nm +- 10%

    def __init__(self, root_position, alpha = np.pi/2, beta = np.pi/2, mirrored = False):
        ###geometry
        self.mirrored = mirrored
        scale = 608/800                 #pixel / mm
        self.l1 = 220 * scale           #lenght of first arm in mm * scale -> length in px
        self.l2 = 150 * scale           #lenght of second arm in px
        self.alpha = alpha
        self.beta = beta
        self.root_p = root_position     #position of root
        self.radius = 15
        self.calc_positions()
        ##position
        self.velocity = np.array([0,0])  
        self.U_a = 2       #Voltage on servo controlling alpha
        self.U_b = 5       #same for beta
        self.setTarget()
        
    #calculate positions of joint and paddle based on angle via trigonometry
    def calc_positions(self):
        if not self.mirrored:
            joint_x = self.root_p[0] + self.l1 * np.sin(self.alpha)
            joint_y = self.root_p[1] + self.l1 * np.cos(self.alpha)
            paddle_x = joint_x + self.l2 * np.sin(self.alpha + self.beta)
            paddle_y = joint_y + self.l2 * np.cos(self.alpha + self.beta)
            self.joint_p = np.array([joint_x, joint_y])
            self.paddle_p = np.array([paddle_x, paddle_y])
        else:
            joint_x = self.root_p[0] - self.l1 * np.sin(self.alpha)
            joint_y = self.root_p[1] - self.l1 * np.cos(self.alpha)
            paddle_x = joint_x - self.l2 * np.sin(self.alpha + self.beta)
            paddle_y = joint_y - self.l2 * np.cos(self.alpha + self.beta)
            self.joint_p = np.array([joint_x, joint_y])
            self.paddle_p = np.array([paddle_x, paddle_y])

    def get_paddle_position(self):
        return self.paddle_p

    def update(self, time_lapse, ball):
        pos_old = self.paddle_p
        self.calc_positions()
        self.setTarget()            #set Target ggf mit radomness beim Einstellen der Voltage hinzufügen, weil Servos ungenau
        self.move(time_lapse)
        self.velocity = (self.paddle_p - pos_old) / time_lapse

       ##collision with ball
        dist = np.linalg.norm(self.paddle_p - ball.position)            #distance between paddle and ball       
        if dist <= self.radius + ball.radius:                           #on collision:  
            a_vector_norm = (ball.position - self.paddle_p) / dist      #acceleration unit vector, orthogonal to contact area 
            #angle with ball and distance                               #determines rotation direction
            ##Bounce 
            ball.velocity = reflect_vector(ball.velocity, a_vector_norm) * RobotArm.collisionFactor
            #!Änderung der Rotation nach Fallunterscheidung hinzufügen

            ##Acceleration by Paddle
            if not np.allclose(self.velocity, np.array([0,0])):             #if paddle is moving (velocity != 0, with tolerance)
                speed = np.linalg.norm(self.velocity)
                v_vector_norm = self.velocity / speed
                a_angle = np.arccos(np.clip(np.dot(a_vector_norm, v_vector_norm), -1.0, 1.0))  #calculate angle between normal and velocity vector
                print("dist: ", self.paddle_p - ball.position)
                print("angle: ", round(a_angle / np.pi *360, 2))
                print("v: ", self.velocity)
                v_rad = speed * np.cos(a_angle)               #radial part of velocity
                v_lat = speed * np.sin(a_angle)               #lateral part of velocity
                a_vector = a_vector_norm * v_rad * 15         #Factor subject to change
                ball.accelerate(a_vector)
                print("v_rad: ", v_rad)
                print("a_vector: ", a_vector)
                print("-----------")
                #H!ier muss noch Fallunterscheidung rein für Rotation
                angle_diff = a_vector_norm[0] * v_vector_norm[1] - a_vector_norm[0] * v_vector_norm[1]
                if angle_diff > 0:
                    ball.rotation = ball.rotation + v_lat * RobotArm.friction
                else:
                    ball.rotation = ball.rotation - v_lat * RobotArm.friction

            ball.position = ball.position + a_vector_norm * 10               #set ball outside of paddle
            

    def setTarget(self):
        self.target_a = self.U_a / 5 * RobotArm.max_angle 
        self.target_b = self.U_b / 5 * RobotArm.max_angle

    def move(self, time_lapse):
        ###Rotate
        rot_a = time_lapse/1000 * np.pi / 0.42              #how far we can rotate in current frame
        if abs(self.alpha - self.target_a) < rot_a:         #if we can reach the target position, do that and stop
            self.alpha = self.target_a
        elif self.alpha < self.target_a:                    #else, move towards target angle
            self.alpha = self.alpha + rot_a
        elif self.alpha > self.target_a:                    
            self.alpha = self.alpha - rot_a

        rot_b = time_lapse/1000 * np.pi / 0.42              #same for second Servo
        if abs(self.beta - self.target_b) < rot_b:
            self.beta = self.target_b
        elif self.beta < self.target_b:
            self.beta = self.beta + rot_b
        elif self.beta > self.target_b:
            self.beta = self.beta - rot_b



    def render(self,screen):
        pygame.draw.circle(screen, (255,255,255), self.joint_p.astype(int), 6, 0)
        pygame.draw.circle(screen, (255,255,255), self.paddle_p.astype(int), 15, 0)
        pygame.draw.circle(screen, (255,255,255), self.root_p.astype(int), 6, 0)
        pygame.draw.line(screen, (255,0,0), self.joint_p.astype(int), self.root_p.astype(int), 7)
        pygame.draw.line(screen, (0,0,255), self.joint_p.astype(int), self.paddle_p.astype(int), 4)
