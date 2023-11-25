from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.screenmanager import ScreenManager, Screen

from kivy.properties import (
    NumericProperty, ReferenceListProperty, ObjectProperty
)
from kivy.vector import Vector
from kivy.clock import Clock


class PongPaddle(Widget):
    score = NumericProperty(0)

    def bounce_ball(self, ball):
        if self.collide_widget(ball):
            vx, vy = ball.velocity
            offset = (ball.center_y - self.center_y) / (self.height / 2)
            bounced = Vector(-1 * vx, vy)
            vel = bounced * 1.1
            ball.velocity = vel.x, vel.y + offset


class PongBall(Widget):
    velocity_x = NumericProperty(0)
    velocity_y = NumericProperty(0)
    velocity = ReferenceListProperty(velocity_x, velocity_y)

    def move(self):
        self.pos = Vector(*self.velocity) + self.pos


class PongGame(Widget):

    ball = PongBall()
    player1 = PongPaddle()
    player2 = PongPaddle()

    def serve_ball(self, vel=(4, 0)):
        # print(dir(self.ball))
        self.ball.center = self.center
        self.ball.velocity = vel

    def update(self, dt):
        self.ball.move()

        # bounce off paddles
        self.player1.bounce_ball(self.ball)
        self.player2.bounce_ball(self.ball)


        # bounce ball off bottom or top
        if (self.ball.y < self.y) or (self.ball.top > self.top):
            self.ball.velocity_y *= -1

        # went off to a side to score point?
        if self.ball.x < self.x:
            self.player2.score += 1
            self.serve_ball(vel=(4, 0))
        if self.ball.right > self.width:
            self.player1.score += 1
            self.serve_ball(vel=(-4, 0))


        err1 = (self.ball.y - self.player2.center_y);
        # delta = err - err2

        self.player2.center_y += err1*0.1
    def on_touch_move(self, touch):
        if touch.x < self.width / 3:
            self.player1.center_y = touch.y
        # if touch.x > self.width - self.width / 3:
        #     self.player2.center_y = touch.y


class PotatoWindow(Widget):
    def update(self,dt):
        print(dt)

class ShaderViewer(Widget):
    pass

class Screen_One(Screen):
    def update(self,dt):
        print(dt)


class Screen_Two(Screen):
    def update(self,dt):
        print(dt)

class PotatoApp(App):
    def build(self):
        my_app = PotatoWindow()
        Clock.schedule_interval(my_app.update, 1.0 / 60.0)
        return my_app


if __name__ == '__main__':
    PotatoApp().run()