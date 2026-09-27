#!/usr/bin/python3
# coding: utf-8

# Author : vledoze
# Date   : 22/11/2017
# Author 2 : jofaure
# Date     : 25/04/2022

# ##IMPORTS =======================================================
# Global imports
import math
import numpy
import numpy.random
import unittest


# ##CONSTANTS =========================================================
R2D = 180.0/math.pi
D2R = math.pi/180.0
G = 9.81

__NTEST = 100000


# ##FUNCTIONS ==========================================================
def quaternion2euler(q):
    """
    Conversion of quaternions to euler angles
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    with q0->q3, q1->q0, q2->q1, q3->q2
    let q0.i + q1*j + q2*k + q3

    input 1:
         q = [q0, q1, q2, q3] - Quaternions
    input 2:
         q = [q2, q3] - Simplified quaternions
    --------------------------------------
    output:
         psi, tet, phi - Euler angles
    """
    # Case input 1
    if (len(q) == 4):
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]
        # Conversion
        phi = math.atan2(2.0*(q3*q0 + q1*q2), 1.0 - 2.0*(q0*q0 + q1*q1))
        tet = math.asin(2.0*(q3*q1 - q0*q2))
        psi = math.atan2(2.0*(q3*q2 + q0*q1), 1.0 - 2.0*(q1*q1 + q2*q2))
        # Outputs
        return psi, tet, phi
    # Case input 2
    if (len(q) == 2):
        # Heading
        psi = quaternion2psi(q)
        # Outputs
        return psi, 0.0, 0.0
    # Default outputs
    return None, None, None


def quaternion2tetphi(q):
    """
    Conversion of quaternions to euler angles
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    with q0->q3, q1->q0, q2->q1, q3->q2
    let q0.i + q1*j + q2*k + q3

    input:
         q = [q0, q1, q2, q3] - Quaternions
    -------------------------------------
    output:
         tet, phi - Euler angles
    """
    if (len(q) == 4):
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]
        # Conversion
        phi = math.atan2(2.0*(q3*q0 + q1*q2), 1.0 - 2.0*(q0*q0 + q1*q1))
        tet = math.asin(2.0*(q3*q1 - q0*q2))
        # Outputs
        return tet, phi
    # Default outputs
    return None, None


def quaternion2psi(q):
    """
    Conversion of quaternions to euler angles
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    with q0->q3, q1->q0, q2->q1, q3->q2
    let q0.i + q1*j + q2*k + q3

    input 1:
         q = [q2, q3] - Simplified quaternions
    input 2:
         q = [q0, q1, q2, q3] - Quaternions
    -------------------------------------------
    output:
         psi - Euler angle (heading)
    """
    # Case input 1
    if (len(q) == 2):
        # Conversion
        psi = math.atan2(2.0*q[1]*q[0], 1.0 - 2.0*q[0]*q[0])
        # Output
        return psi
    # Case input 2
    if (len(q) == 4):
        # Conversion
        psi = math.atan2(2.0*(q[0]*q[1] + q[3]*q[2]), 1.0 - 2.0*(q[1]*q[1] + q[2]*q[2]))
        # Output
        return psi
    # Default output
    return None


def euler2quaternion(psi, tet, phi):
    """
    Conversion of euler angles to quaternions
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    with q0->q3, q1->q0, q2->q1, q3->q2
    let q0.i + q1*j + q2*k + q3

    in:
         psi, tet, phi - Euler angles
    out:
         q = [q0, q1, q2, q3] - Quaternions
    """
    cospsi = math.cos(psi * 0.5)
    sinpsi = math.sin(psi * 0.5)
    costet = math.cos(tet * 0.5)
    sintet = math.sin(tet * 0.5)
    cosphi = math.cos(phi * 0.5)
    sinphi = math.sin(phi * 0.5)
    # Conversions
    q0 = sinphi*costet*cospsi - cosphi*sintet*sinpsi
    q1 = cosphi*sintet*cospsi + sinphi*costet*sinpsi
    q2 = cosphi*costet*sinpsi - sinphi*sintet*cospsi
    q3 = cosphi*costet*cospsi + sinphi*sintet*sinpsi
    # Output
    return quaternionnormalize([q0, q1, q2, q3])  # if normalize else [q0, q1, q2, q3]


def tetphi2quaternion(tet, phi):
    """
    Conversion of euler angles to quaternions
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    with q0->q3, q1->q0, q2->q1, q3->q2
    let q0.i + q1*j + q2*k + q3

    in:
         tet, phi - Euler angles
    out:
         q = [q0, q1, q2, q3] - Quaternions
    """
    costet = math.cos(tet * 0.5)
    sintet = math.sin(tet * 0.5)
    cosphi = math.cos(phi * 0.5)
    sinphi = math.sin(phi * 0.5)
    # Conversions
    q0 = sinphi*costet
    q1 = cosphi*sintet
    q2 = -sinphi*sintet
    q3 = cosphi*costet
    # Output
    return quaternionnormalize([q0, q1, q2, q3])  # if normalize else [q0, q1, q2, q3]


def psi2quaternion(psi):
    """
    Conversion of euler angles to quaternions
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    with q0->q3, q1->q0, q2->q1, q3->q2
    let q0.i + q1*j + q2*k + q3

    in:
         psi - Euler angle (heading)
    out:
         q = [q2, q3] - Quaternions
    """
    cospsi = math.cos(psi * 0.5)
    sinpsi = math.sin(psi * 0.5)
    # Conversions
    q2 = sinpsi
    q3 = cospsi
    # Ouputs
    return [q2, q3]


def euler2matrix(psi, tet, phi):
    """
    Calculation of the rotation matrix from euler angles

    input:
         psi, tet, phi - Euler angles
    output:
         rot - rotation matrix (numpy.matrix)
    """
    rot_psi = numpy.matrix([
        [math.cos(psi), -math.sin(psi), 0.0],
        [math.sin(psi), math.cos(psi), 0.0],
        [0.0, 0.0, 1.0]])
    rot_tet = numpy.matrix([
        [math.cos(tet), 0.0, math.sin(tet)],
        [0.0, 1.0, 0.0],
        [-math.sin(tet), 0.0, math.cos(tet)]])
    rot_phi = numpy.matrix([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(phi), -math.sin(phi)],
        [0.0, math.sin(phi), math.cos(phi)]])
    return rot_psi*rot_tet*rot_phi


def matrix2euler(mat):
    """
    Calculation of euler angles associated with a rotation matrix
    """
    psi = math.atan2(mat[1, 0], mat[0, 0])
    tet = math.asin(-mat[2, 0])
    phi = math.atan2(mat[2, 1], mat[2, 2])
    return psi, tet, phi


def psi2matrix(psi):
    """
    Calculation of rotation matrix from euler angle psi

    input:
         psi - Euler angles around the vertical axis
    output:
         rot - rotation matrix (numpy.matrix)
    """
    rot_psi = numpy.matrix([
        [math.cos(psi), -math.sin(psi), 0.0],
        [math.sin(psi), math.cos(psi), 0.0],
        [0.0, 0.0, 1.0]])
    return rot_psi


def quaternion2matrix(q):
    """
    Calculation of the rotation matrix from the quaternions
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    with q0 -> q3, q1 ->q0, q2->q1, q3->q2
    let q0.i + q1*j + q2*k + q3

    input:
         q = [q0, q1, q2, q3] - Quaternions
    output:
         rot - rotation matrix (numpy.matrix)
    """
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]
    # Rotation matrix
    rot = numpy.matrix([
        [q3*q3 + q0*q0 - q1*q1 - q2*q2, 2.0*(q0*q1 - q3*q2), 2.0*(q3*q1 + q0*q2)],
        [2.0*(q0*q1 + q3*q2), q3*q3 - q0*q0 + q1*q1 - q2*q2, 2.0*(q1*q2 - q3*q0)],
        [2.0*(q0*q2 - q3*q1), 2.0*(q3*q0 + q1*q2), q3*q3 - q0*q0 - q1*q1 + q2*q2]])
    # Output
    return rot


def rotatewitheuler(v, psi, tet, phi):
    """
    Rotation with euler angles

    input:
         v - input vector (numpy.array)
         psi, tet, phi - Euler angles
    output:
         u - output vector (numpy.array)
    """
    rot = euler2matrix(psi, tet, phi)
    u = rot*numpy.matrix(v).T
    return [u[0, 0], u[1, 0], u[2, 0]]


def rotatewithpsi(v, psi):
    """
    Rotation with euler angles psi

    input:
         v - input vector (numpy.array)
         psi - Euler's angle around the vertical axis
    output:
         u - output vector (numpy.array)
    """
    rot = psi2matrix(psi)
    u = rot*numpy.matrix(v).T
    return [u[0, 0], u[1, 0], u[2, 0]]


def rotatewithquaternion(v, q, method3D=True):
    """
    Rotating a vector with quaternions
    q0.i + q1*j + q2*k + q3

    input 1:
         v - input vector (numpy.array)
         q = [q0, q1, q2, q3] - Quaternions
    input 2:
         v - input vector (numpy.array)
         q = [q2, q3] - Simplified quaternions
    output:
         u - output vector (numpy.array)
    """
    # Case Input 1 or Input 2
    if (len(v) == 3) & (len(q) == 4):
        if method3D:
            w = [q[0], q[1], q[2]]
            t = 2.0*numpy.cross(w, v)
            # Computing
            u = v + q[3]*t + numpy.cross(w, t)
        else:
            u = quaternionproduct(quaternionproduct(q, v), quaternionconj(q))
            u.pop(-1)
        # Output
        return u
    # Case Input 2
    if (len(v) == 3) & (len(q) == 2):
        if method3D:
            w = [0.0, 0.0, q[0]]
            t = 2.0*numpy.cross(w, v)
            # Computing
            u = v + q[1]*t + numpy.cross(w, t)
        else:
            u = quaternionproduct(quaternionproduct(q, v), quaternionconj(q))
            u.pop(-1)
        # Output
        return u
    # Default output
    return None


def quaternionconj(q):
    """
    Calculation of the conjugate of the quaternion
    q0.i + q1*j + q2*k + q3

    input 1:
         q = [q0, q1, q2, q3] - Quaternions
    input 2:
         q = [q2, q3] - Simplified quaternions
    ------------------------------------------
    output 1:
         q* = [-q0, -q1, -q2, q3] - Quaternions
    output 2:
         q* = [-q2, q3] - Simplified quaternions
    """
    # Case input 1
    if len(q) == 2:
        return [-q[0], q[1]]
    # Case input 2
    if len(q) == 4:
        return [-q[0], -q[1], -q[2], q[3]]
    # Default output
    return None


def quaternioninv(q):
    """
    Calculating the inverse of a quaternion
    q0.i + q1*j + q2*k + q3

    input 1:
         q = [q0, q1, q2, q3] - Quaternions
    input 2:
         q = [q2, q3] - Simplified quaternions
    --------------------------------------
    output 1:
         q-1 = [-q0/nq, -q1/nq, -q2/nq, q3/nq] - Quaternions
    output 2:
         q-1 = [-q2/nq, q3/nq] - Simplified quaternions
    """
    # Case input 1
    if len(q) == 4:
        n = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]
        return [-q[0]/n, -q[1]/n, -q[2]/n, q[3]/n]
    # Case input 2
    if len(q) == 2:
        n = q[0]*q[0] + q[1]*q[1]
        return [-q[0]/n, q[1]/n]
    # Default output
    return None


def quaternionproduct(q1, q2):
    """
    Define the hamilton product for two quaternions
    input 1:
         q1 = [q01, q11, q21, q31] - Quaternion
         q2 = [q02, q12, q22, q32] - Quaternion
    input 2:
         q1 = [q01, q11, q21, q31] - Quaternion
         q2 = [q02, q12, q22] - Vector
    input 3:
         q1 = [q01, q11, q21] - Vector
         q2 = [q02, q12, q22, q32] - Quaternion
    input 4:
         q1 = [q21, q31] - Simplified quaternion
         q2 = [q22, q32] - Simplified quaternion
    input 5:
         q1 = [q21, q31] - Simplified quaternion
         q2 = [q02, q12, q22] - Vector
    input 6:
         q1 = [q01, q11, q21] - Vector
         q2 = [q22, q32] - Simplified quaternion
    ------------------------------------------
    output 1:
         q = (q01.i + q11*j + q21*k + q31)(q02.i + q12*j + q22*k + q32)
           =[q1,q2,q3,q4]-Quaternion
    output 2:
         q = (q01.i + q11*j + q21*k + q31)(q02.i + q12*j + q22*k)
           =[q1,q2,q3,q4]-Quaternion
    output 3:
         q = (q01.i + q11*j + q21*k)(q02.i + q12*j + q22*k + q32)
           =[q1,q2,q3,q4]-Quaternion
    output 4:
         q = (q21*k + q31)(q22*k + q32)
           = [q3, q4] - Simplified quaternion
    output 5:
         q = (q21*k + q31)(q02.i + q12*j + q22*k)
           =[q1,q2,q3,q4]-Quaternion
    output 6:
         q = (q01.i + q11*j + q21*k)(q22*k + q32)
           =[q1,q2,q3,q4]-Quaternion
    """
    # Case input 1
    if (len(q1) == len(q2) == 4):
        q01 = q1[0]
        q11 = q1[1]
        q21 = q1[2]
        q31 = q1[3]
        q02 = q2[0]
        q12 = q2[1]
        q22 = q2[2]
        q32 = q2[3]
        # Computing
        q0 = q31*q02 + q01*q32 + q11*q22 - q21*q12
        q1 = q31*q12 - q01*q22 + q11*q32 + q21*q02
        q2 = q31*q22 + q01*q12 - q11*q02 + q21*q32
        q3 = q31*q32 - q01*q02 - q11*q12 - q21*q22
        # Outputs
        q = [q0, q1, q2, q3]
        return quaternionnormalize(q)
    # Case input 2
    if (len(q1) == 4) & (len(q2) == 3):
        q01 = q1[0]
        q11 = q1[1]
        q21 = q1[2]
        q31 = q1[3]
        q02 = q2[0]
        q12 = q2[1]
        q22 = q2[2]
        # Computing
        q0 = q31*q02 + q11*q22 - q21*q12
        q1 = q31*q12 - q01*q22 + q21*q02
        q2 = q31*q22 + q01*q12 - q11*q02
        q3 = -q01*q02 - q11*q12 - q21*q22
        # Outputs
        q = [q0, q1, q2, q3]
        return q
    # Case input 3
    if (len(q1) == 3) & (len(q2) == 4):
        q01 = q1[0]
        q11 = q1[1]
        q21 = q1[2]
        q02 = q2[0]
        q12 = q2[1]
        q22 = q2[2]
        q32 = q2[3]
        # Computing
        q0 = q01*q32 + q11*q22 - q21*q12
        q1 = -q01*q22 + q11*q32 + q21*q02
        q2 = q01*q12 - q11*q02 + q21*q32
        q3 = -q01*q02 - q11*q12 - q21*q22
        # Outputs
        q = [q0, q1, q2, q3]
        return q
    # Case input 4
    if (len(q1) == len(q2) == 2):
        q21 = q1[0]
        q31 = q1[1]
        q22 = q2[0]
        q32 = q2[1]
        # Computing
        q2 = q31*q22 + q21*q32
        q3 = q31*q32 - q21*q22
        # Outputs
        q = [q2, q3]
        return quaternionnormalize(q)
    # Case input 5
    if (len(q1) == 2) & (len(q2) == 3):
        q21 = q1[0]
        q31 = q1[1]
        q02 = q2[0]
        q12 = q2[1]
        q22 = q2[2]
        # Computing
        q0 = q31*q02 - q21*q12
        q1 = q31*q12 + q21*q02
        q2 = q31*q22
        q3 = -q21*q22
        # Outputs
        q = [q0, q1, q2, q3]
        return q
    # Case input 6
    if (len(q1) == 3) & (len(q2) == 2):
        q01 = q1[0]
        q11 = q1[1]
        q21 = q1[2]
        q22 = q2[0]
        q32 = q2[1]
        # Computing
        q0 = q01*q32 + q11*q22
        q1 = q11*q32 - q01*q22
        q2 = q21*q32
        q3 = -q21*q22
        # Outputs
        q = [q0, q1, q2, q3]
        return q
    # Default outputs
    return None


def quaternionnormalize(q):
    """
    Returns the normed quaternion

    input 1:
         q1 = [q0, q1, q2, q3] - Quaternions
    input 2:
         q1 = [q2, q3] - Simplified quaternions
    ----------------------------------------
    output 1:
         q = [q0/n, q1/n, q2/n, q3/n] - Quaternions
    output 2:
         q = [q2/n, q3/n] - Simplified quaternions
    """
    # Case input 1
    if (len(q) == 4):
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]
        # Computing
        n = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        # Outputs
        return [q0/n, q1/n, q2/n, q3/n]
    # Case input 2
    if (len(q) == 2):
        q2 = q[0]
        q3 = q[1]
        # Computing
        n = math.sqrt(q2*q2 + q3*q3)
        # Outputs
        return [q2/n, q3/n]


def quaterniondiff(q1, q2):
    """
    Difference between two quaternions

    input 1:
         q1 = [q01, q11, q21, q31] - Quaternions
         q2 = [q02, q12, q22, q32] - Quaternions
    input 2:
         q1 = [q21, q31] - Simplified quaternions
         q2 = [q22, q32] - Simplified quaternions
    ---------------------------------------
    output:
         q = [q0, q1, q2, q3] - Quaternions
    """
    if (len(q1) == len(q2) == 4) | (len(q1) == len(q2) == 2):
        # Output
        return quaternionproduct(q1, quaternionconj(q2))
    # Default output
    return None


def minmax(v, min_v=-1.0, max_v=1.0):
    """
    Limits a value between -1 and 1
    """
    return max(min_v, min(max_v, float(v)))


def mean(vlist):
    """
    Calculate the average value of a list
    """
    m = 0.0
    n = float(len(vlist))
    if n > 0:
        for v in vlist:
            m += v
        return m/n
    else:
        return 0.0


def l1_dist(vlist1, vlist2):
    """
    Calculate the distance l1 between two lists
    """
    d = 0.0
    for v1, v2 in zip(vlist1, vlist2):
        d += (v1 - v2)
    return d


def l2_dist(vlist1, vlist2):
    """
    Calculate the distance l2 between two lists
    """
    d = 0.0
    for v1, v2 in zip(vlist1, vlist2):
        d += (v1 - v2)*(v1 - v2)
    return d


def euclidian_dist(vlist1, vlist2):
    """
    Calculate the Euclidean distance between two lists
    """
    d = 0.0
    for v1, v2 in zip(vlist1, vlist2):
        d += (v1 - v2)*(v1 - v2)
    return math.sqrt(d)


def l1_norm(vlist):
    """
    Calculate the l1 norm of a list
    """
    n = 0.0
    for v in vlist:
        n += v
    return n


def euclidian_norm(vlist):
    """
    Calculate the euclidean norm of a list
    """
    n = 0.0
    for v in vlist:
        n += v*v
    return math.sqrt(n)


def round_to_nearest(x, precision):
    """
    Round a number to the requested precision
    """
    correction = 0.5 if x >= 0 else -0.5
    return int(x/precision+correction) * precision


def update_tree_points(list_points, tree_points, drone_name):
    """
    Updating a point tree with a list of new points
    Returns the list of new points
    """
    # List of unique / added points
    list_points_uniques = []
    # New points
    for point in list_points:
        if point.x in tree_points.keys():
            if point.y in tree_points[point.x].keys():
                if not (point.z in tree_points[point.x][point.y].keys()):
                    # New point : discoverer name storage
                    tree_points[point.x][point.y][point.z] = drone_name
                    list_points_uniques.append(point)
            else:
                # New point : discoverer name storage
                tree_points[point.x][point.y] = {point.z: drone_name}
                list_points_uniques.append(point)
        else:
            # New point : discoverer name storage
            tree_points[point.x] = {point.y: {point.z: drone_name}}
            list_points_uniques.append(point)
    return list_points_uniques


def searchn_tree_points(list_points, tree_points, count=False):
    """
    We search if one of the points of the list is already discovered
    Output if count=False
         True: at least 1 known point
         False: no points
    Output if count=True
         Number of points already discovered in the list
    """
    # Counter
    if count:
        n = 0
    # New points
    for point in list_points:
        if point.x in tree_points.keys():
            if point.y in tree_points[point.x].keys():
                if point.z in tree_points[point.x][point.y].keys():
                    if count:
                        n += 1
                    else:
                        return True
    if count:
        return n
    else:
        return False


def search_tree_points(point, tree_points):
    """
    We search if a point has already been discovered
    Output if count=False
         True: The point is known
         False: The point is not known
    """
    if point.x in tree_points.keys():
        if point.y in tree_points[point.x].keys():
            if point.z in tree_points[point.x][point.y].keys():
                return True
    return False


def SigmoidLin(x,p,v_max,r0):
    vel = (r0 - x) * p
    if (p <= 0):
        return 0
    if (vel >= v_max):
        return v_max
    return vel


def SigmoidLike(x,R,d):
    if (x < R):
        return 0
    elif (x < R+d):
        return numpy.sin((math.pi * pow(d,-1.0)) * (x-R) - (math.pi / 2)) + 1.0
    else:
        return 1.0


def VelDecayLinSqrt(x,p,a,v_max,r0):
    vel = (x - r0) * p
    if (a<=0 or p<=0 or vel<=0):
        return 0
    if (vel<a/p):
        if (vel>=v_max):
            return v_max
        return vel
    vel = math.sqrt(2*a*(x - r0) - a**2/p**2)

def perpendicular( a ) :
    b = numpy.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b



# ##CLASSES ============================================================
class TestQuaternions(unittest.TestCase):

    def test_conversion(self):
        # Euler -> Quaternion
        for i in range(__NTEST):
            psi = math.pi*numpy.random.uniform()
            tet = math.pi/2.0*numpy.random.uniform()
            phi = math.pi/2.0*numpy.random.uniform()
            q = euler2quaternion(psi, tet, phi)
            psi_, tet_, phi_ = quaternion2euler(q)
            self.assertAlmostEqual(psi_, psi)
            self.assertAlmostEqual(tet_, tet)
            self.assertAlmostEqual(phi_, phi)
        # Quaternion -> Euler
        for i in range(__NTEST):
            q = quaternionnormalize(numpy.random.random_sample((4,)))
            psi, tet, phi = quaternion2euler(q)
            q_ = euler2quaternion(psi, tet, phi)
            for j in range(len(q)):
                self.assertAlmostEqual(q_[j], q[j])

    def test_rotation(self):
        # Euler -> Quaternion
        for i in range(__NTEST):
            psi = math.pi*numpy.random.uniform()
            tet = math.pi/2.0*numpy.random.uniform()
            phi = math.pi/2.0*numpy.random.uniform()
            v = numpy.random.random_sample((3,))
            v1 = rotatewitheuler(v, psi, tet, phi)
            q = euler2quaternion(psi, tet, phi)
            v2 = rotatewithquaternion(v, q)
            v3 = rotatewithquaternion(v, q, method3D=False)
            for j in range(len(v1)):
                self.assertAlmostEqual(v1[j], v2[j])
                self.assertAlmostEqual(v1[j], v3[j])
        # Quaternion->Euler
        for i in range(__NTEST):
            q = quaternionnormalize(numpy.random.random_sample((4,)))
            v = numpy.random.random_sample((3,))
            v1 = rotatewithquaternion(v, q)
            v1bis = rotatewithquaternion(v, q, method3D=False)
            psi, tet, phi = quaternion2euler(q)
            v2 = rotatewitheuler(v, psi, tet, phi)
            for j in range(len(v1)):
                self.assertAlmostEqual(v1[j], v2[j])
                self.assertAlmostEqual(v1bis[j], v2[j])

    def test_quaternionconj(self):
        for i in range(__NTEST):
            psi = math.pi*numpy.random.uniform()
            tet = math.pi/2.0*numpy.random.uniform()
            phi = math.pi/2.0*numpy.random.uniform()
            q = euler2quaternion(psi, tet, phi)
            qc = quaternionconj(q)
            psi, tet, phi = quaternion2euler(quaternionproduct(q, qc))
            self.assertAlmostEqual(psi, 0.0)
            self.assertAlmostEqual(tet, 0.0)
            self.assertAlmostEqual(phi, 0.0)

    def test_quaternioninv(self):
        for i in range(__NTEST):
            psi = math.pi*numpy.random.uniform()
            tet = math.pi/2.0*numpy.random.uniform()
            phi = math.pi/2.0*numpy.random.uniform()
            q = euler2quaternion(psi, tet, phi)
            qi = quaternioninv(q)
            psi, tet, phi = quaternion2euler(quaternionproduct(q, qi))
            self.assertAlmostEqual(psi, 0.0)
            self.assertAlmostEqual(tet, 0.0)
            self.assertAlmostEqual(phi, 0.0)

    def test_quaternioninv2(self):
        for i in range(__NTEST):
            q = [
                1000*numpy.random.uniform(),
                1000*numpy.random.uniform(),
                1000*numpy.random.uniform(),
                1000*numpy.random.uniform()]
            qi = quaternioninv(q)
            psi, tet, phi = quaternion2euler(quaternionproduct(q, qi))
            self.assertAlmostEqual(psi, 0.0)
            self.assertAlmostEqual(tet, 0.0)
            self.assertAlmostEqual(phi, 0.0)

    def test_quaternioninv3(self):
        for i in range(__NTEST):
            q = [
                1000*numpy.random.uniform(),
                1000*numpy.random.uniform(),
                1000*numpy.random.uniform(),
                1000*numpy.random.uniform()]
            qi = quaternioninv(q)
            qc = quaternionconj(q)
            qic = quaternionconj(qi)
            qci = quaternioninv(qc)
            for qic_k, qci_k in zip(qci, qic):
                self.assertAlmostEqual(qic_k, qci_k)



# ##MAIN ===============================================================
if __name__ == "__main__":
    # Tests Quaternions
    unittest.main()
