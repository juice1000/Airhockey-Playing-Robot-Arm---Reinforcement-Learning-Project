import gameObjects as go
import numpy as np
from random import randrange
import pygame
from pygame.locals import *
from video import make_video
import reinforcementLearningAgent as rlAgent
import math

###Options:
#ScreenSize as (width, height) in pixels
screenSize = (608,304)      #realer Tisch [mm]: 800x400 ~ 608x304 px in der cam (scale = 608 / 800)
paddle_size = 20
paddle_speed = 1.3
ballRadius = 8
goal_size = 100
voltage_step = 0.2       #amount by which voltage can be changed per frame
reward_p1 = 0
reward_p2 = 0
state_p1 = 0
state_p2 = 0

###simulator options (reduce time and turn off rendering for faster training)
render = True   #display on/off
tickTime = 64   #max frames
                #Kamera: 16ms = 1 frame

#constants for later use
_white = (255,255,255)
_black = (0,0,0)
pygame.font.init()
myFont = pygame.font.SysFont("Times New Roman", 15)

#Point counters, for both sides
pointsLeftPlayer = 0
pointsRightPlayer = 0
reward_p1 = 0
reward_p2 = 0


###function to show values on screen for diagnostics
def showText(screen, robotArm, ball):
    Ua_display = myFont.render("U_a: " + str(round(robotArm.U_a, 2)), 1, _white)
    Ub_display = myFont.render("U_b: " + str(round(robotArm.U_b, 2)), 1, _white)
    screen.blit(Ua_display, (screenSize[0] - 70, 10))
    screen.blit(Ub_display, (screenSize[0] - 70, 30))
    rot_display = myFont.render("state Player1: " + str(round(state_p1,1)),1, _white)
    rot2_display = myFont.render("state Player2: " + str(round(state_p2,1)),1, _white)
    speed_x_display = myFont.render("Reward P1: " + str(round(reward_p1,1)),1, _white)
    speed_y_display = myFont.render("Reward P2: " + str(round(reward_p2,1)),1, _white)
    screen.blit(speed_x_display, (10,10))
    screen.blit(speed_y_display, (10,30))
    screen.blit(rot_display, (10,50))
    screen.blit(rot2_display, (10,70))
    score_display = myFont.render(str(pointsLeftPlayer) + " : " + str(pointsRightPlayer),1, _white)
    screen.blit(score_display,(280, 10))

def draw_goals(screen, screenSize):
    pygame.draw.line(screen, (0,255,0), (0, screenSize[1] / 2 - goal_size / 2), (0, screenSize[1] / 2 + goal_size / 2) , 5)
    pygame.draw.line(screen, (0,255,0), (screenSize[0], screenSize[1] / 2 - goal_size / 2), (screenSize[0], screenSize[1] / 2 + 60), 5)
    

def reset_complete():
    numbX = randrange(50)
    numbY = randrange(150)
    ball = go.Ball(np.array([130+numbX,82+numbY]), ballRadius)
    reward_p1 = 0
    reward_p2 = 0
    pointsLeftPlayer = 0
    pointsRightPlayer = 0
    state_p1 = 0
    state_p2 = 0
    return ball
    
def calculate_distance_to_gate_p1(ball):
    pos = ball.position
    dist = distance = math.sqrt( ((ball.position[0]-0)**2)+((ball.position[1]-152)**2) )
    return dist

def calculate_distance_to_gate_p2(ball):
    pos = ball.position
    dist = math.sqrt( ((ball.position[0]-680)**2)+((ball.position[1]-152)**2))
    return dist

def reset_p1():
    numbX = randrange(50)
    numbY = randrange(150)
    ball = go.Ball(np.array([130+numbX,82+numbY]), ballRadius)
    return ball

def reset_p2():
    numbX = randrange(50)
    numbY = randrange(150)
    ball = go.Ball(np.array([450+numbX,82+numbY]), ballRadius)
    return ball

def get_state():
    return (state_p1, state_p2)

def get_reward_p1():
    return reward_p1

def get_reward_p2():
    return reward_p2

def calculate_distance_player_to_ball(robot, ball):
    pos_ball = ball.position
    pos_robot = robot.get_paddle_position()
    return np.linalg.norm(pos_robot - pos_ball) 
    
def enumToAction(robot, enum):
    if enum == 0:
        robot.U_a += voltage_step
        if robot.U_a > 5:
            robot.U_a = 5
    if enum == 1:
        robot.U_a -= voltage_step
        if robot.U_a < 0:
            robot.U_a = 0
    if enum == 2:
        robot.U_b += voltage_step
        if robot.U_b > 5:
            robot.U_b = 5
    if enum == 3:
        robot.U_b -= voltage_step
        if robot.U_b < 0:
            robot.U_b = 0


###main function
def main(screenSize):
###globals for access
    global pointsLeftPlayer
    global pointsRightPlayer
    global time_elapsed             #time since last frame
    global reward_p1
    global reward_p2
    global state_p1
    global state_p2
    training_modus = False

###initialize all the things
    pygame.init()
    screen = pygame.display.set_mode(screenSize)
    clock = pygame.time.Clock()
    ball =  go.Ball(np.array([180,152]), ballRadius)
#    paddle = go.Paddle(np.array([screenSize[0]-100, 152]),paddle_size)
    robot = go.RobotArm(np.array([-70, screenSize[1] / 2]))
    robot_r = go.RobotArm(np.array([screenSize[0] + 70,screenSize[1] / 2]), mirrored = True)
    video = False                               #record video of screen. Toggle ingame via "v"
    save_screen = make_video(screen)            #initialize recording

    running = True
###game loop
    while running:
        time_elapsed = clock.tick(tickTime)     #get time since last tick
        for event in pygame.event.get():        #get Event-Input
            if event.type == QUIT:              #alt+F4 = Quit, closing window works as well
                running = False
            if event.type == KEYDOWN:           
                if event.key == pygame.K_v:     #toggle recording of video with "v"
                    video = not video
                if event.key == pygame.K_r:     # r: reset ball
                    ball = reset_complete()
                

    ###keyboard input
        keys = pygame.key.get_pressed()         #checking pressed keys, returns TRUE if pressed        

        #if time_elapsed % 30 == 0:
        #    ball = reset_p1()
        #Paddle
#        if keys[pygame.K_UP]:
#            paddle.accelerate(np.array([0, -1*paddle_speed]))
#        if keys[pygame.K_DOWN]:
#            paddle.accelerate(np.array([0,paddle_speed]))
#        if keys[pygame.K_LEFT]:
#            paddle.accelerate(np.array([-1*paddle_speed, 0]))
#        if keys[pygame.K_RIGHT]:
#            paddle.accelerate(np.array([paddle_speed,0]))

        if(training_modus == False):
            #robot arm left side: WASD
            if keys[pygame.K_d]:
                if robot.paddle_p[1] < screenSize[1]-30:
                    robot.U_a -= voltage_step
                    if robot.U_a < 0:
                        robot.U_a = 0
            if keys[pygame.K_a]:
                if robot.paddle_p[1] > 30:  
                    robot.U_a += voltage_step
                    if robot.U_a > 5:
                        robot.U_a = 5
            if keys[pygame.K_w]:
                if robot.paddle_p[1] > 30:
                    robot.U_b += voltage_step
                    if robot.U_b > 5:
                        robot.U_b = 5
            if keys[pygame.K_s]:
                if robot.paddle_p[1] < screenSize[1] -30:
                    robot.U_b -= voltage_step
                    if robot.U_b < 0:
                        robot.U_b = 0

            #robot arm right side: Arrow Keys
            if keys[pygame.K_RIGHT]:
                robot_r.U_a -= voltage_step
                if robot_r.U_a < 0:
                    robot_r.U_a = 0
            if keys[pygame.K_LEFT]:
                robot_r.U_a += voltage_step
                if robot_r.U_a > 5:
                    robot_r.U_a = 5
            if keys[pygame.K_UP]:
                robot_r.U_b += voltage_step
                if robot_r.U_b > 5:
                    robot_r.U_b = 5
            if keys[pygame.K_DOWN]:
                robot_r.U_b -= voltage_step
                if robot_r.U_b < 0:
                    robot_r.U_b = 0
        else:
            next_action_p1 = np.argmax(agent1.get_next_action(state_p1))
            next_action_p2 = np.argmax(agent2.get_next_action(state_p2))
            enumToAction(robot, next_action_p1)
            enumToAction(robot_r, next_action_p2)
            if keys[pygame.K_q]:
                agent1.save_q_table('q_table_player1')
                agent2.save_q_table('q_table_player2')

        #start training
        if keys[pygame.K_t]:
            agent1 = rlAgent.Agent()
            agent2 = rlAgent.Agent()
            training_modus = True

            next_action_p1 = np.argmax(agent1.get_next_action(state_p1))
            next_action_p2 = np.argmax(agent2.get_next_action(state_p2))
            enumToAction(robot, next_action_p1)
            enumToAction(robot_r, next_action_p2)
           
    ###updates
#        paddle.update(screenSize, ball)
        ball.update(screenSize)
        robot.update(time_elapsed, ball)
        robot_r.update(time_elapsed,ball)

        reward_p1 = (calculate_distance_to_gate_p1(ball) * 0.001) - (calculate_distance_player_to_ball(robot, ball) * 0.002)
        reward_p2 = (calculate_distance_to_gate_p2(ball) * 0.001) - (calculate_distance_player_to_ball(robot_r, ball) * 0.002)
        ##check for goals
        if ball.hitRight: #Statt Abfrage ein pygame event erstellen, das auslöst bei Treffer
            pointsLeftPlayer += 1
            ball = reset_p2()
            reward_p1 = 5
            reward_p2 = -5
        if ball.hitLeft:
            pointsRightPlayer += 1
            ball = reset_p1()
            reward_p2 = 5
            reward_p1 = -5

        old_state_p1 = state_p1
        old_state_p2 = state_p2
        state_p1 = int(round(ball.position[0],0) + round(ball.position[1],0)) + int(round(calculate_distance_player_to_ball(robot, ball),0))
        state_p2 = int(round(ball.position[0],0) + round(ball.position[1],0)) + int(round(calculate_distance_player_to_ball(robot_r, ball),0))
            
        if(training_modus == True):
            agent1.update(old_state_p1, state_p1, next_action_p1, reward_p1)
            agent2.update(old_state_p2, state_p2, next_action_p2, reward_p2)


    ###render
        if render == True:
            screen.fill((0,0,0))
            ball.render(screen)
#           paddle.render(screen)
            robot.render(screen)
            showText(screen, robot, ball)
            draw_goals(screen,screenSize)
            robot_r.render(screen)
            pygame.display.flip()

            if video == True:
                next(save_screen)

    pygame.quit()

main(screenSize)
