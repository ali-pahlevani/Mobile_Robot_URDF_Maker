from PyQt5.QtCore import Qt
from PyQt5.QtOpenGL import QGLWidget
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

class OpenGLWidget(QGLWidget):
    def __init__(self, parent=None):
        super(OpenGLWidget, self).__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)
        
        self.rotation_x = 0
        self.rotation_y = 0
        self.zoom = -5

        # Default dimensions
        self.L = 1.2  
        self.W = 0.8
        self.H = 0.3

        # Default wheel and lidar sizes
        self.wheel_radius = 0.22
        self.wheel_width  = 0.12
        self.lidar_radius = 0.1
        self.lidar_height = 0.08
        self.camera_size = (0.08, 0.2, 0.08)

        # Default colors (RGB tuples)
        self.chassis_color = (0.5, 0.5, 0.5)
        self.wheel_color   = (0.0, 0.0, 0.0)
        self.lidar_color   = (1.0, 0.0, 0.0)
        self.camera_color  = (0.0, 0.0, 1.0)
        
    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.7, 0.7, 0.7, 1.0)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, self.zoom)
        glRotatef(self.rotation_x, 1, 0, 0)
        glRotatef(self.rotation_y, 0, 1, 0)

        # Draw the chassis (cube) with dynamic color
        glPushMatrix()
        glColor3f(*self.chassis_color)
        glScalef(self.L, self.W, self.H)
        glutSolidCube(1.0)
        glPopMatrix()

        # Draw a coordinate frame attached to the chassis
        # The chassis is centered at the origin, so the coordinate frame is drawn from the origin
        glPushMatrix()
        glLineWidth(6.0)  # Make the axes thicker for visibility
        glBegin(GL_LINES)
        # x-axis in red
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(1.5, 0, 0) # self.L * 2.0
        # y-axis in green
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 1.5, 0) # self.W * 2.0
        # z-axis in blue
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 1.5) # self.H * 2.0
        glEnd()
        glLineWidth(1.0)  # Reset line width
        glPopMatrix()

        # Draw the wheels (cylinders) with dynamic size and color
        glColor3f(*self.wheel_color)
        wheel_positions = [
            (self.L / 2, self.W / 2 + self.wheel_width / 2, -self.H / 2),
            (self.L / 2, -self.W / 2 - self.wheel_width / 2, -self.H / 2),
            (-self.L / 2, self.W / 2 + self.wheel_width / 2, -self.H / 2),
            (-self.L / 2, -self.W / 2 - self.wheel_width / 2, -self.H / 2),
        ]
        for pos in wheel_positions:
            glPushMatrix()
            glTranslatef(*pos)
            glRotatef(90, 1, 0, 0)
            glTranslatef(0, 0, -self.wheel_width/2)  # Recenters the wheel width
            glutSolidCylinder(self.wheel_radius, self.wheel_width, 20, 20)
            glPopMatrix()

        # Draw the lidar (cylinder) with dynamic size and color
        glColor3f(*self.lidar_color)
        glPushMatrix()
        glTranslatef(0.0, 0.0, self.H / 2)
        #glRotatef(180, 1, 0, 0)
        glutSolidCylinder(self.lidar_radius, self.lidar_height, 20, 20)
        glPopMatrix()

        # Draw the camera (cube) with dynamic size and color
        glColor3f(*self.camera_color)
        glPushMatrix()
        glTranslatef(self.L / 2, 0.0, 0.0)
        glScalef(*self.camera_size)
        glutSolidCube(1.0)
        glPopMatrix()

    def mousePressEvent(self, event):
        self.last_pos = event.pos()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            dx = event.x() - self.last_pos.x()
            dy = event.y() - self.last_pos.y()
            self.rotation_x += dy
            self.rotation_y += dx
            self.last_pos = event.pos()
            self.update()

    def wheelEvent(self, event):
        # Adjust zoom based on the wheel delta; 120 is a typical delta step
        delta = event.angleDelta().y() / 120  
        self.zoom += delta * 0.5
        self.update()

    def updateRobotModel(self, L, W, H, wheel_radius, wheel_width, lidar_radius, lidar_height, camera_size,
                         chassis_color, wheel_color, lidar_color, camera_color):
        # Update the robot model dimensions and visual properties
        self.L = L
        self.W = W
        self.H = H
        self.wheel_radius = float(wheel_radius)
        self.wheel_width  = float(wheel_width)
        self.lidar_radius = float(lidar_radius)
        self.lidar_height = float(lidar_height)
        self.camera_size = tuple(map(float, camera_size.split()))
        self.chassis_color = chassis_color
        self.wheel_color   = wheel_color
        self.lidar_color   = lidar_color
        self.camera_color  = camera_color
        self.update()
