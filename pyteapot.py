"""
PyTeapot module for drawing rotating cube using OpenGL as per
quaternion or yaw, pitch, roll angles received over serial port.
"""

import pygame
import math
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

useSerial = True # set true for using serial for data transmission, false for wifi
useQuat = False   # set true for using quaternions, false for using y,p,r angles

if(True):
    import serial
    ser = serial.Serial('/dev/ttyUSB0', 115200)

def main():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("PyTeapot IMU orientation visualization")
    resizewin(640, 480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()

    # about 6 Hz
    # dt = 1.0/6.0
    # about 100 Hz
    dt = 1.0/100.0
    # State vector
    x = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    # State stepping matrix
    F = np.array([
        [1, 0, 0,  0, 0, 0,  dt,0, 0,  0, 0, 0 ], 
        [0, 1, 0,  0, 0, 0,  0, dt,0,  0, 0, 0 ], 
        [0, 0, 1,  0, 0, 0,  0, 0, dt, 0, 0, 0 ], 

        [0, 0, 0,  1, 0, 0,  0, 0, 0,  dt,0, 0 ], 
        [0, 0, 0,  0, 1, 0,  0, 0, 0,  0, dt,0 ], 
        [0, 0, 0,  0, 0, 1,  0, 0, 0,  0, 0, dt],

        [0, 0, 0,  0, 0, 0,  1, 0, 0,  0, 0, 0 ], 
        [0, 0, 0,  0, 0, 0,  0, 1, 0,  0, 0, 0 ], 
        [0, 0, 0,  0, 0, 0,  0, 0, 1,  0, 0, 0 ], 

        [0, 0, 0,  0, 0, 0,  0, 0, 0,  1, 0, 0 ], 
        [0, 0, 0,  0, 0, 0,  0, 0, 0,  0, 1, 0 ], 
        [0, 0, 0,  0, 0, 0,  0, 0, 0,  0, 0, 1 ]
        ])

    # State covariance matrix
    P = 0.1*np.identity(len(x))
    # State error matrix
    Q = 0.01*np.identity(len(x))

    # Expected state vector reading
    mu_e = np.zeros(len(x))
    # Expected state covariance matrix
    Sigma_e = 0.1*np.identity(len(x))

    # State measurement matrix
    H = np.identity(len(x))
    for i in range(0, 6):
        H[i, i] = 0

    # Actual state vector reading
    mu_a = np.zeros(len(x))
    # Expected state covariance matrix
    Sigma_a = np.zeros((len(x), len(x)))

    # Kalman gain
    K = np.zeros((len(x), len(x)))

    print("F")
    print(F)
    print("H")
    print(H)

    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        # Construct direction cosine matrix
        phi   = x[3] * math.pi/180 # converted to radians
        theta = x[4] * math.pi/180
        psi   = x[5] * math.pi/180

        dcm1 = np.array([
            [1,              0,               0],
            [0,  math.cos(phi),   math.sin(phi)],
            [0, -math.sin(phi),   math.cos(phi)]
        ])

        dcm2 = np.array([
            [math.cos(theta),  0,  -math.sin(theta)],
            [0,                1,                 0],
            [math.sin(theta),  0,   math.cos(theta)],
        ])

        dcm3 = np.array([
            [math.cos(psi),   math.sin(psi),  0],
            [-math.sin(psi),  math.cos(psi),  0],
            [0,               0,              1]
        ])

        dcm = np.matmul(dcm3, np.matmul(dcm2, dcm1));

        # read body frame vectors
        [udot, vdot, wdot, p, q, r] = read_data()
        vdot_bf  = np.array([udot, vdot, wdot])
        omega_bf = np.array([p, q, r])

        # body vectors in inertial coords
        omega_e = np.matmul(dcm, omega_bf)
        vdot_e  = np.matmul(dcm, vdot_bf)

        # step forwards
        x = np.matmul(F, x)
        P = np.matmul(F, np.matmul(P, F.transpose())) + Q

        # expected
        mu_e    = np.matmul(H, x)
        Sigma_e = np.matmul(H, np.matmul(P, H.transpose()))

        # Actual
        mu_a[ 6] = vdot_e[0]
        mu_a[ 7] = vdot_e[1]
        mu_a[ 8] = vdot_e[2]
        mu_a[ 9] = omega_e[0]
        mu_a[10] = omega_e[1]
        mu_a[11] = omega_e[2]

        Sigma_a = P

        # Kalman gain
        K = np.matmul(np.matmul(P, H.transpose()), np.linalg.inv(Sigma_e + Sigma_a))

        # Update
        x = x + np.matmul(K, mu_a - mu_e)
        P = P - np.matmul(K, np.matmul(H,P) )

        # draw
        phi   = x[3]
        theta = x[4]
        psi   = x[5]

        draw(1, phi, theta, psi)

        pygame.display.flip()
        frames += 1

    print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    ser.close()


def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1

    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def cleanSerialBegin():
    try:
        line = ser.readline().decode('UTF-8').replace('\n', '')
        u     = float(line.split('u')[1])
        v     = float(line.split('v')[1])
        w     = float(line.split('w')[1])
        roll  = float(line.split('p')[1])
        pitch = float(line.split('q')[1])
        yaw   = float(line.split('r')[1])
    except Exception:
        pass


def read_data():
    ser.reset_input_buffer()
    cleanSerialBegin()

    line = ser.readline().decode('UTF-8').replace('\n', '').split(' ')
                
    # u     = float(line.split('u')[1])
    # v     = float(line.split('v')[1])
    # w     = float(line.split('w')[1])
    # roll  = float(line.split('p')[1])
    # pitch = float(line.split('q')[1])
    # yaw   = float(line.split('r')[1])

    # print(line)
    # [u'-0.27', u'0.40', u'-0.09', u'0.01', u'-0.01', u'-0.00', u'\r']
    u     = float(line[0])
    v     = float(line[1])
    w     = float(line[2])
    roll  = float(line[3])
    pitch = float(line[4])
    yaw   = float(line[5])

    return [u, v, w, roll, pitch, yaw]


def draw(w, nx, ny, nz):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "PyTeapot", 18)
    drawText((-2.6, 1.6, 2), "Module to visualize quaternion or Euler angles data", 16)
    # drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    # yaw   = nx
    # pitch = ny
    # roll  = nz
    roll  = nx
    pitch = ny
    yaw   = nz

    drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
    glRotatef(-roll,  0.00,  0.00,  1.00)
    glRotatef(pitch,  1.00,  0.00,  0.00)
    glRotatef(yaw,    0.00,  1.00,  0.00)
    # glRotatef(roll,   1.00,  0.00,  0.00)
    # glRotatef(pitch,  0.00,  1.00,  0.00)
    # glRotatef(yaw,    0.00,  0.00,  1.00)

    glBegin(GL_QUADS)
    glColor3f(0.0,    1.0,  0.0)
    glVertex3f(1.0,   0.2,  -1.0)
    glVertex3f(-1.0,  0.2,  -1.0)
    glVertex3f(-1.0,  0.2,  1.0)
    glVertex3f(1.0,   0.2,  1.0)

    glColor3f(1.0,    0.5,   0.0)
    glVertex3f(1.0,   -0.2,  1.0)
    glVertex3f(-1.0,  -0.2,  1.0)
    glVertex3f(-1.0,  -0.2,  -1.0)
    glVertex3f(1.0,   -0.2,  -1.0)

    glColor3f(1.0,    0.0,   0.0)
    glVertex3f(1.0,   0.2,   1.0)
    glVertex3f(-1.0,  0.2,   1.0)
    glVertex3f(-1.0,  -0.2,  1.0)
    glVertex3f(1.0,   -0.2,  1.0)

    glColor3f(1.0,    1.0,   0.0)
    glVertex3f(1.0,   -0.2,  -1.0)
    glVertex3f(-1.0,  -0.2,  -1.0)
    glVertex3f(-1.0,  0.2,   -1.0)
    glVertex3f(1.0,   0.2,   -1.0)

    glColor3f(0.0,    0.0,   1.0)
    glVertex3f(-1.0,  0.2,   1.0)
    glVertex3f(-1.0,  0.2,   -1.0)
    glVertex3f(-1.0,  -0.2,  -1.0)
    glVertex3f(-1.0,  -0.2,  1.0)

    glColor3f(1.0,   0.0,   1.0)
    glVertex3f(1.0,  0.2,   -1.0)
    glVertex3f(1.0,  0.2,   1.0)
    glVertex3f(1.0,  -0.2,  1.0)
    glVertex3f(1.0,  -0.2,  -1.0)
    glEnd()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


if __name__ == '__main__':
    main()
