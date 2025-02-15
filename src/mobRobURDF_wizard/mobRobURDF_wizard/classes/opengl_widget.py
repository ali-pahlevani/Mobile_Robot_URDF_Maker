import numpy as np
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
        
        # Using a rotation matrix (for camera rotation)
        self.zoom = -5
        self.rotation = np.identity(4, dtype=np.float32)  # Accumulated rotation matrix
        self.lastPos3D = None  # Last mapped mouse position (on the virtual sphere)

        # Default dimensions
        self.L = 1.2  
        self.W = 0.8
        self.H = 0.3

        # Default wheel and sensors sizes
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
        gluPerspective(45, w / h if h != 0 else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, self.zoom)
        # Apply the accumulated rotation
        glMultMatrixf(self.rotation.T)  # Note: OpenGL expects column-major order

        # Draw the chassis (cube) with dynamic color
        glPushMatrix()
        glColor3f(*self.chassis_color)
        glScalef(self.L, self.W, self.H)
        glutSolidCube(1.0)
        glPopMatrix()

        # Draw a coordinate frame attached to the chassis
        glPushMatrix()
        glLineWidth(6.0)  # Thicker axes for visibility
        glBegin(GL_LINES)
        # x-axis in red
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(1.5, 0, 0)
        # y-axis in green
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 1.5, 0)
        # z-axis in blue
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 1.5)
        glEnd()
        glLineWidth(1.0)
        glPopMatrix()

        # Draw the wheels (cylinders)
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
            glTranslatef(0, 0, -self.wheel_width/2)
            glutSolidCylinder(self.wheel_radius, self.wheel_width, 20, 20)
            glPopMatrix()

        # Draw the lidar (cylinder)
        glColor3f(*self.lidar_color)
        glPushMatrix()
        glTranslatef(0.0, 0.0, self.H / 2)
        glutSolidCylinder(self.lidar_radius, self.lidar_height, 20, 20)
        glPopMatrix()

        # Draw the camera (cube)
        glColor3f(*self.camera_color)
        glPushMatrix()
        glTranslatef(self.L / 2 + self.camera_size[0] / 2, 0.0, 0.0)
        glScalef(*self.camera_size)
        glutSolidCube(1.0)
        glPopMatrix()

    def _map_to_sphere(self, x, y):
        # Map the 2D point (x,y) from widget coordinates to a 3D point on the unit sphere
        width = self.width()
        height = self.height()
        # Avoid division by zero
        if width <= 0 or height <= 0:
            return np.array([0.0, 0.0, 0.0], dtype=np.float32)
        # Normalize x and y to range [-1, 1]
        nx = (2.0 * x - width) / width
        ny = (height - 2.0 * y) / height  # invert y to match OpenGL's coordinate system
        length2 = nx * nx + ny * ny
        if length2 > 1.0:
            norm = 1.0 / np.sqrt(length2)
            return np.array([nx * norm, ny * norm, 0.0], dtype=np.float32)
        else:
            return np.array([nx, ny, np.sqrt(1.0 - length2)], dtype=np.float32)
    
    def mousePressEvent(self, event):
        # Map the starting mouse position onto the sphere
        self.lastPos3D = self._map_to_sphere(event.x(), event.y())

    def mouseMoveEvent(self, event):
        # Only rotate if the left mouse button is pressed
        if not (event.buttons() & Qt.LeftButton):
            return

        currentPos3D = self._map_to_sphere(event.x(), event.y())
        # Guard: ensure self.lastPos3D is a valid 3-element vector
        if self.lastPos3D is None or np.array(self.lastPos3D).ndim != 1 or np.array(self.lastPos3D).size != 3:
            self.lastPos3D = currentPos3D
            return

        # Compute the rotation axis via the cross product
        axis = np.cross(self.lastPos3D, currentPos3D)
        # Compute the angle between the two positions
        dot = np.clip(np.dot(self.lastPos3D, currentPos3D), -1.0, 1.0)
        angle = np.arccos(dot)
        # Only update if there is a significant rotation
        if np.linalg.norm(axis) > 1e-5:
            axis = axis / np.linalg.norm(axis)
            c = np.cos(angle)
            s = np.sin(angle)
            t = 1 - c
            ax, ay, az = axis
            R = np.array([
                [t*ax*ax + c,   t*ax*ay - s*az, t*ax*az + s*ay, 0],
                [t*ax*ay + s*az, t*ay*ay + c,   t*ay*az - s*ax, 0],
                [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c,    0],
                [0,             0,              0,              1]
            ], dtype=np.float32)
            self.rotation = R @ self.rotation

        self.lastPos3D = currentPos3D
        self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120  
        self.zoom += delta * 0.5
        self.update()

    def updateRobotModel(self, L, W, H, wheel_radius, wheel_width, lidar_radius, lidar_height, camera_size,
                         chassis_color, wheel_color, lidar_color, camera_color):
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
