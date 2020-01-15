"""
Timo Jantzen
12.05.2019
This is a simple 1 player pong vs computer game I wrote to familiarize myself with rendering, movement and collision of Objects in Python.
In this version, the paddle can'T change direction of the ball.
It was created during a university project on reinforcement learning as a first step to build an environment
for training AI Agents that should later control a robot arm on a miniuature air hockey table.
While this game has no fancy AI Stuff or real physics yet, it's still kind of fun.
"""

import pygame
import numpy as np
from pygame.locals import *
from random import randrange

###Options:
goal = 3                   #points it takes to win
screenSize = (600,400)      #ScreenSize as (width, height) in pixels
ballSpeed = 6
paddleSpeed = 4
paddle_size = 100
ballRadius = 8
tickTime = 64               #ms for a frame, or tick as it's called for pygame events

###constants for later use
_white = (255,255,255)
_black = (0,0,0)

#Point counters, for both sides
pointsLeftPlayer = 0
pointsRightPlayer = 0
    
class Ball():
#options
    radius = ballRadius
    global pointsLeftPlayer
    global pointsRightPlayer

    def __init__(self, position):
        ###ball starts at 45° angle in 1 out of 4 ways at random
        v_x = ballSpeed * randrange(-1,2,2)     #gives eihter -1 or 1 at random
        v_y = ballSpeed * randrange(-1,2,2)
        self.position = position
        self.velocity = np.array([v_x,v_y])
        self.radius = Ball.radius
        self.hit_left_egde = False
        self.hit_right_edge = False

    def update(self, PlayerPaddle, AIPaddle):
        self.move()
        ###collision check - for walls, a circle always collides if distance between his center and wall is less than his radius
        if self.position[0] + self.radius < 1:                      #collide with left wall
            self.hit_left_egde = True
            self.velocity[0] *= -1

        if self.position[0] + self.radius > screenSize[0] - 1:      #right wall
            self.hit_right_edge = True
            self.velocity[0] *= -1

        if self.position[1] - self.radius < 1 or self.position[1] + self.radius > screenSize[1] - 1:     #bounce on top/bottom wall
            #angle_out is equal to angle_in for now
            self.velocity[1] *= -1

        ##collision with paddle
        #player paddle / right
        if self.position[0]+self.radius + 5 > PlayerPaddle.x:
            if abs(self.position[1]-PlayerPaddle.y) < PlayerPaddle.height/2 + self.radius :
                self.position[0] = PlayerPaddle.x - self.radius - 5
                self.velocity[0] *= -1
                    #add acceleration function later
        
        #ai paddle / left
        if self.position[0]-self.radius - 5 < AIPaddle.x:
            if abs(self.position[1]-AIPaddle.y) < AIPaddle.height/2 + self.radius -2 :
                self.position[0] = AIPaddle.x +self.radius + 5
                self.velocity[0] *= -1

    def move(self):
        self.position += self.velocity

    def render(self,screen):
        pygame.draw.circle(screen, (255,255,255), (int(self.position[0]),int(self.position[1])), self.radius, 0)    #Füllung des Kreises (width = 0 füllt den Kreis)
       # pygame.draw.circle(screen, (0,0,0), self.position, self.radius, 1)          #Schwarzer Rand


class PlayerPaddle:
    #paddle kriegen die funktion für collision mit dem ball?
    #könnte cool sein das hier zu haben um punkte zu zählen usw beim treffen
    
    def __init__(self):
        #positio, player is on the right
        self.x = screenSize[0]-5
        self.y =int(screenSize[1]*0.5)
        #player paddle dimensions
        self.height = paddle_size
        self.width = 10
        self.rect = pygame.Rect(0, self.y - int(self.height*0.5), self.width, self.height)
        self.color = _white

        #player paddle speed
        self.speed = paddleSpeed
        self.direction = 0

    def update(self):

        #Bewegung
        self.y += self.direction*self.speed
        #Falls man schon am Rand ist, nicht bewegen.
        self.rect.center = (self.x, self.y)
        if self.rect.top < 0:
            self.rect.top = 0
        if self.rect.bottom > screenSize[1]-1:
            self.rect.bottom = screenSize[1]-1

    def render(self, screen):
        pygame.draw.rect(screen, self.color, self.rect, 0)
        pygame.draw.rect(screen, (0,0,0), self.rect, 1)

class AIPaddle:
    def __init__(self):

        self.x = 5
        self.y = int(screenSize[1]*0.5)

        #ai paddle dimensions
        self.height = paddle_size
        self.width = 10

        self.rect = pygame.Rect(0, self.y - int(self.height*0.5), self.width, self.height)
        self.color = _white
        #ai paddle speed
        self.speed = paddleSpeed

    def update(self, ball):
        if ball.position[1]  < self.rect.top - ball.radius:
            self.y -= self.speed
        elif ball.position[1] > self.rect.bottom + ball.radius:
            self.y += self.speed

        self.rect.center = (self.x, self.y)

    def render(self, screen):
        pygame.draw.rect(screen, self.color, self.rect, 0)
        pygame.draw.rect(screen, (0,0,0), self.rect, 1)

    

def main(screenSize):

    pygame.init()
    global pointsLeftPlayer
    global pointsRightPlayer
    screen = pygame.display.set_mode(screenSize)

    #initialize clock object. Damit wird die z.B. zeit zwischen frames getrackt.
    clock = pygame.time.Clock()
    startx = screenSize[0]/2
    starty = screenSize[1]/2
    startPosition = (startx, starty)
    ball = Ball(startPosition)
    ball.acclerate(np.array([-10,0]))

    ai_paddle = AIPaddle()
    player_paddle = PlayerPaddle()

    running = True
    
    while running:

        clock.tick(tickTime)

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

            if event.type == KEYDOWN:
                if event.key == K_UP:
                    player_paddle.direction = -1
                elif event.key == K_DOWN:
                    player_paddle.direction = 1
            if event.type == KEYUP:
                if event.key == K_UP and player_paddle.direction == -1:
                    player_paddle.direction = 0
                elif event.key == K_DOWN and player_paddle.direction == 1:
                    player_paddle.direction = 0



        ai_paddle.update(ball)
        player_paddle.update()
        ball.update(player_paddle, ai_paddle)

        if ball.hit_left_egde:
            pointsRightPlayer += 1
            ball.__init__(startPosition)
            if pointsRightPlayer == goal:
                print ('You Won')
                running = False
        elif ball.hit_right_edge:
            pointsLeftPlayer += 1
            ball.__init__(startPosition)
            if pointsLeftPlayer == goal:
                print ('You lost')
                running = False


        screen.fill((0,0,0))

        ai_paddle.render(screen)
        player_paddle.render(screen)
        ball.render(screen)

        pygame.display.flip()

    pygame.quit()

main(screenSize)