import math
import numpy as np
import time

# --------- Question 1 ---------- #


def rotation_matrix(axis, angle):
    """
    Compute 3D rotation matrix for a given axis and angle

    Parameters
    ----------
    axis
        the axis of rotation, as a string "x", "y" or "z"
    angle
        the angle of rotation in radians

    Returns
    -------
    R
        the SO(3) rotation matrix
    """
    if(axis == 'x'):
        R = np.array([[1,              0,          0],
                    [0, np.cos(angle), (-1)*np.sin(angle)],
                    [0, np.sin(angle), np.cos(angle)]],dtype=np.float64)
        return R
    elif(axis == 'y'):
        R = np.array([[np.cos(angle),       0,    np.sin(angle)],
                    [0,               1,           0],
                    [(-1)*np.sin(angle), 0, np.cos(angle)]],dtype=np.float64)
        return R
    elif(axis == 'z'):
        R = np.array([[np.cos(angle), (-1)*np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0,          0,          1]],dtype=np.float64)
        return R
    print("Rotation Matrix failed")
    
    return("Rotation Matrix failed")
    pass


# --------- Question 3 ---------- #
def transformation_matrix(axis, angle, t):
    """
    Create a 3D homogeneous transformation matrix

    Parameters
    ----------
    axis
        the axis of rotation, as a string "x", "y" or "z"
    angle
        the angle of rotation in radians
    t
        the translation vector (1D array)

    Returns
    -------
    T
        the SE(3) transformation matrix
    """
    R = rotation_matrix(axis,angle)
    #T = np.full((4, 4), 0)
    T = np.eye(4) #np.eye works and np.full doesn't. 
    T[:3, :3] = R #I needed to discover I could do this earlier my god
    T[:3, 3] = t 
    T[3,3] = 1
    # printing initial array

    return(T)

def vector_matrix_se3_multiplication(T,vector):
        vector_mod = np.append(vector,1)
        new_vector = T@vector_mod
        new_vector = new_vector[:3]
        return new_vector


def wordTypingRobot(robotObj, word: str):
    """
    Type out a word on the screen using the Dobot Robot.
    Note: You must not change the name of this file or this function.

    Parameters
    ----------
    robotObj
        Dobot object; see fakeRobot.py for the API
        You can pass in a FakeRobot or CoppeliaRobot object for testing
    word
        Word to type out

    """
    print(f"I was asked to type: {word}")
    letter = "a"
   #for letter in word:
    gotoKeylocation(robotObj, letter)


# Note: The remainder of the file is a template on how we would solve this task.
# You are free to use our template, or to write your own code.
def gotoKeylocation(robotObj, letter: str):
    pos = getPositionForLetter(letter)
    jumpToPos(robotObj, pos)


def getPositionForLetter(letter: str) -> np.array:
    '''
    This function should return the x, y, z coordinates of the letter on the screen.
    You need to figure out what these coordinates are for each letter.
    '''
    angle = np.arctan2(0.175,-0.150)
    O_T_K = transformation_matrix("z",angle,np.array([0.175,-0.150,0]))
    K_P = np.array([0,0,0],dtype=np.float64)
    O_P = vector_matrix_se3_multiplication(O_T_K,K_P)

    return(O_P)
    pass


def jumpToPos(robotObj, target_pos: np.array):
    '''
    This function should move the robot to the given position.
    Note: We recommend the following strategy:
    1. Move the robot to a position 20mm above the target position
    2. Move the robot to the target position
    3. Move the robot to a position 20mm above the target position
    This strategy will avoid the pen to drag across the screen
    '''
    # Move the robot to a position 20mm above the target position
    #print("Original: " + str(target_pos))

    pos = target_pos  # You will need to change this
    pos[2] = pos[2] + 0.020
    #print("After: " + str(pos))
    j1, j2, j3 = ikine(pos)
    robotObj.move_arm(j1, j2, j3)
    time.sleep(5)


    # Move the robot to the target position
    pos[2] = 0.002# You will need to change this
    j1, j2, j3 = ikine(pos)
    #print(str(j1) + " " + str(j2) + " " + str(j3))
    robotObj.move_arm(j1, j2, j3)
    time.sleep(5)


    # Move the robot to a position 20mm above the target position
    pos = target_pos  # You will need to change this
    pos[2] = pos[2] + 0.020
    j1, j2, j3 = ikine(pos)
    robotObj.move_arm(j1, j2, j3)
    print("got here")
    time.sleep(5)


def ikine(pos: np.array) -> np.array:
    '''
    This function should return the joint angles for the given position.
    '''
    """
    Inverse kinematics using geometry

    Parameters
    ----------
    pstar
        A numpy array of shape (3,) of the desired end-effector position [x, y, z] (mm)

    Returns
    -------
    theta
        A numpy array of shape (3,) of joint angles [theta1, theta2, theta3] (radians)

    """
    #------------------------------------paramaters-----------------------------------------
    L0 = 0.138  # height of shoulder raise joint above ground plane
    L1 = 0.135  # length of upper arm
    L2 = 0.147  # length of lower arm
    L3 = 0.060   # horizontal tool displacement from wrist passive joint
    L4 = -0.080  # vertical tool displacement from wrist passive joint
    #--------------------------------------------------------------

    #1
    x = pos[0]
    y = pos[1]
    z = pos[2]
    r = np.sqrt((x**2 + y**2))
    #2
    theta1 = np.arctan2(y,x)
    #3
    b = L0+L4-z
    #4
    a = r-L3
    #5
    c = np.sqrt((a**2)+(b**2))
    #6
    delta = np.arctan2(a,b)
    #7
    temp = ((-c**2 + L2**2 + L1**2))/(2*L1*L2)
    alpha = np.arccos(temp)
    #8
    temp = ((-L2**2 + c**2 + L1**2))/(2*c*L1)
    beta = np.arccos(temp)
    #9
    theta2 = np.pi - beta - delta
    #10
    theta3 = np.pi - alpha - (np.pi/2 - theta2)



    return np.array([theta1,theta2,theta3],dtype=np.float64)
    pass
