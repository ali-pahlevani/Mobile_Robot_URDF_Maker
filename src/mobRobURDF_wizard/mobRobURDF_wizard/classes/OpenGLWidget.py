from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
from PyQt5.QtOpenGL import QGLWidget
from PyQt5.QtCore import Qt

# Class for 3D Preview
class OpenGLWidget(QGLWidget):
    def __init__(self, parent=None):
        super(OpenGLWidget, self).__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)
        self.zoom = -5
        self.rotation = np.identity(4, dtype=np.float32)
        self.lastPos3D = None
        self.L, self.W, self.H = 1.2, 0.8, 0.3
        self.wheel_radius, self.wheel_width = 0.22, 0.12
        self.caster_radius = 0.22  # Default to wheel_radius for 2_wheeled_caster
        self.lidar_radius, self.lidar_height = 0.1, 0.08
        self.camera_size = (0.08, 0.2, 0.08)
        self.chassis_color = (0.5, 0.5, 0.5)
        self.wheel_color = (0.0, 0.0, 0.0)
        self.lidar_color = (1.0, 0.0, 0.0)
        self.camera_color = (0.0, 0.0, 1.0)
        self.robot_type = "4_wheeled"  # Default

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
        glMultMatrixf(self.rotation.T)

        # Draw chassis
        glPushMatrix()
        glColor3f(*self.chassis_color)
        glScalef(self.L, self.W, self.H)
        glutSolidCube(1.0)
        glPopMatrix()

        # Draw wheels based on robot type
        glColor3f(*self.wheel_color)
        if self.robot_type == "4_wheeled":
            wheel_positions = [
                (self.L / 2, self.W / 2 + self.wheel_width / 2, -self.H / 2),  # Front left
                (self.L / 2, -self.W / 2 - self.wheel_width / 2, -self.H / 2),  # Front right
                (-self.L / 2, self.W / 2 + self.wheel_width / 2, -self.H / 2),  # Rear left
                (-self.L / 2, -self.W / 2 - self.wheel_width / 2, -self.H / 2),  # Rear right
            ]
            for pos in wheel_positions:
                glPushMatrix()
                glTranslatef(*pos)
                glRotatef(90, 1, 0, 0)
                glTranslatef(0, 0, -self.wheel_width / 2)
                glutSolidCylinder(self.wheel_radius, self.wheel_width, 20, 20)
                glPopMatrix()
        elif self.robot_type == "3_wheeled":
            wheel_positions = [
                (self.L / 2, 0, -self.H / 2),  # Front wheel (centered)
                (-self.L / 2, self.W / 2 + self.wheel_width / 2, -self.H / 2),  # Rear left
                (-self.L / 2, -self.W / 2 - self.wheel_width / 2, -self.H / 2),  # Rear right
            ]
            for pos in wheel_positions:
                glPushMatrix()
                glTranslatef(*pos)
                glRotatef(90, 1, 0, 0)
                glTranslatef(0, 0, -self.wheel_width / 2)
                glutSolidCylinder(self.wheel_radius, self.wheel_width, 20, 20)
                glPopMatrix()
        elif self.robot_type == "2_wheeled_caster":
            wheel_positions = [
                (0, self.W / 2 + self.wheel_width / 2, -self.H / 2),  # Left wheel
                (0, -self.W / 2 - self.wheel_width / 2, -self.H / 2),  # Right wheel
            ]
            for pos in wheel_positions:
                glPushMatrix()
                glTranslatef(*pos)
                glRotatef(90, 1, 0, 0)
                glTranslatef(0, 0, -self.wheel_width / 2)
                glutSolidCylinder(self.wheel_radius, self.wheel_width, 20, 20)
                glPopMatrix()
            # Caster wheel as a sphere
            glPushMatrix()
            glTranslatef(self.L / 2, 0, -self.H / 2)
            glutSolidSphere(self.caster_radius, 20, 20)
            glPopMatrix()

        # Draw lidar
        glColor3f(*self.lidar_color)
        glPushMatrix()
        glTranslatef(0.0, 0.0, self.H / 2)
        glutSolidCylinder(self.lidar_radius, self.lidar_height, 20, 20)
        glPopMatrix()

        # Draw camera
        glColor3f(*self.camera_color)
        glPushMatrix()
        glTranslatef(self.L / 2 + self.camera_size[0] / 2, 0.0, 0.0)
        glScalef(*self.camera_size)
        glutSolidCube(1.0)
        glPopMatrix()

    def _map_to_sphere(self, x, y):
        width, height = self.width(), self.height()
        if width <= 0 or height <= 0:
            return np.array([0.0, 0.0, 0.0], dtype=np.float32)
        nx = (2.0 * x - width) / width
        ny = (height - 2.0 * y) / height
        length2 = nx * nx + ny * ny
        if length2 > 1.0:
            norm = 1.0 / np.sqrt(length2)
            return np.array([nx * norm, ny * norm, 0.0], dtype=np.float32)
        return np.array([nx, ny, np.sqrt(1.0 - length2)], dtype=np.float32)

    def mousePressEvent(self, event):
        self.lastPos3D = self._map_to_sphere(event.x(), event.y())

    def mouseMoveEvent(self, event):
        if not (event.buttons() & Qt.LeftButton):
            return
        currentPos3D = self._map_to_sphere(event.x(), event.y())
        if self.lastPos3D is None or np.array(self.lastPos3D).size != 3:
            self.lastPos3D = currentPos3D
            return
        axis = np.cross(self.lastPos3D, currentPos3D)
        dot = np.clip(np.dot(self.lastPos3D, currentPos3D), -1.0, 1.0)
        angle = np.arccos(dot)
        if np.linalg.norm(axis) > 1e-5:
            axis /= np.linalg.norm(axis)
            c, s = np.cos(angle), np.sin(angle)
            t = 1 - c
            ax, ay, az = axis
            R = np.array([
                [t*ax*ax + c, t*ax*ay - s*az, t*ax*az + s*ay, 0],
                [t*ax*ay + s*az, t*ay*ay + c, t*ay*az - s*ax, 0],
                [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c, 0],
                [0, 0, 0, 1]
            ], dtype=np.float32)
            self.rotation = R @ self.rotation
        self.lastPos3D = currentPos3D
        self.update()

    def wheelEvent(self, event):
        self.zoom += event.angleDelta().y() / 120 * 0.5
        self.update()

    def updateRobotModel(self, L, W, H, wheel_radius, wheel_width, lidar_radius, lidar_height, camera_size,
                         chassis_color, wheel_color, lidar_color, camera_color, robot_type="4_wheeled", caster_radius=None):
        self.L, self.W, self.H = L, W, H
        self.wheel_radius = float(wheel_radius)
        self.wheel_width = float(wheel_width)
        self.lidar_radius = float(lidar_radius)
        self.lidar_height = float(lidar_height)
        self.camera_size = tuple(map(float, camera_size.split()))
        self.chassis_color = chassis_color
        self.wheel_color = wheel_color
        self.lidar_color = lidar_color
        self.camera_color = camera_color
        self.robot_type = robot_type
        self.caster_radius = float(caster_radius) if caster_radius is not None else self.wheel_radius
        self.update()