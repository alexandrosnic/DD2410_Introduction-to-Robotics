#! /usr/bin/env python3
from math import *
from numpy import linalg as LA
import numpy as np

"""
    # {student full name}
    # {student email}
"""



def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]
    l0 = 0.07
    l1 = 0.3
    l2 = 0.35

    """
    Fill in your IK solution here and return the three joint values in q
    """

    return q

def transform(q,num):
    DH=[pi/2,0,0,q[0];-pi/2,0,0,q[1];-pi/2,L,0,q[2];pi/2,0,0,q[3];pi/2,M,0,q[4];-pi/2,0,0,q[5];0,0,0,q[6]]]
    link = [1:num]
    for i in num:

        T(i,i-1)=[[         cos(DH[i,3],       -sin(DH[i,3])*cos(DH[i,0]),        sin(DH[i,4])*sin(DH[i,0]),     DH[i,2])*cos(DH[i,3])],
                  [        sin(DH[i,3]),        cos(DH[i,3])*cos(DH[i,0]),       -cos(DH[i,4])*sin(DH[i,0]),      DH(i,2)*sin(DH[i,3])],
                  [                   0,                     sin(DH[i,0]),                     cos(DH[i,0]),                   DH(i,1)],
                  [                   0,                                0,                                0,                         1]]

        Transform=Transform*T(i,i-1)
    return Transform

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements
    btos = 0.311
    stoe = 0.4
    etow = 0.39
    wtof = 0.078
    GC=[0, 0, 0, 0, 0, 0, 0]
    qv = [0, 0, 0, 0, 0, 0, 0]

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    # T = transform(q)


    for i in [1,3,5]:
        if q[i]>=0:
            GC[i]=1
        else:
            GC[i]=-1

    # To find psi
    p0_2 = [0, 0, btos]
    p2_4 = [0, stoe, 0]
    p4_6 = [0, 0, etow]
    p6_7 = [0, 0, wtof]
    p2_6 = point - p0_2 - R * p6_7
    qv[3] = GC[3] * acos((LA.norm(p2_6)**2 - stoe**2 - etow**2)/(2*stoe*etow))
    qv[2] = 0
    if LA.norm(np.cross(p2_6, [sin(q[0]), -cos(q[0]), 0]))>0:
        qv[0]=atan2(p2_6[1], p2_6[0])
    else:
        qv[0]=0
    
    phi = acos((stoe**2+LA.norm(p2_6)**2-etow**2)/(2*stoe*LA.norm(p2_6)))
    qv[1]=atan2(sqrt(p2_6[0]**2+p2_6[1]**2), p2_6[2]) + GC[3] * phi

    # Tv = transform(qv[0:3],0,0,0)
    #pv0_4 = Tv * qv[0:3]
    # p = T * q

    # p0_4 = p[3,:]
    # print(p0_4)

    T0_4 = transform(q,3)
    p0_4 = T0_4[0:2][3]
    p0_4 = T * q[0:3]
    pv0_4 = p0_4
    # Tv = transform(qv[0:1],0,0,0,0,0)
    # pv0_2 = Tv * qv[0:1]
    p0_2 = T * q[0:1]
    pv0_2 = p0_2
    # T = transform(q[0:5],0)
    p0_6 = T * q[0:5]
    pv0_6 = p0_6
    # pv0_6 = Tv * qv

    vvsew = np.cross(((pv0_4 - pv0_2)/(LA.norm(pv0_4 - pv0_2))), ((pv0_6 - pv0_2)/(LA.norm(pv0_6 - pv0_2))))

    # T = transform(q[0:3])
    
    vsew = np.cross(((p0_4 - p0_2)/(LA.norm(p0_4 - p0_2))), ((p0_6 - p0_2)/(LA.norm(p0_6 - p0_2))))
    sgpsi = np.sign(np.inner(np.cross(vvsew, vsew), p2_6))
    psi = sgpsi * acos(np.inner(vvsew, vsew))

    # Inverse Kinematics
    q[3] = qv[3]
    p2_6cross = [0, -p2_6[2], p2_6[1]; p2_6[2], 0, -p2_6[0]; -p2_6[1], p2_6[0], 0]
    R0_psi = np.ones(3) + sin(psi)* p2_6cross + (1-cos(psi)*p2_6cross**2)
    Tv = transform(qv[0:2])
    Rv0_3 = Tv[Tv[0,0:2];Tv[1,0:2];Tv[2,0:2]]
    As = p2_6cross * Rv0_3
    Bs = -p2_6cross**2 * Rv0_3
    Cs = p2_6*np.transpose(p2_6) * Rv0_3

    R0_3 = As * sin(psi) + Bs * cos(psi) + Cs

    Rq0_3 = [ 0, cos(q[0]*sin(q[1])), 0 ; 0, sin(q[0])*sin(q[1]), 0 ; -sin(q[1])*cos(q[2]), cos(q[1]), -sin(q[1])*sin(q[2]) ]

    q[0] = atan2(GC[1] * (As[1,1]*sin(psi) + Bs[1,1]*cos(psi) + Cs[1,1]), GC[1] * (As[0,1]*sin(psi) + Bs[0,1]*cos(psi) + Cs[0,1]))
    q[1] = GC[1] * acos(As[2,1]*sin(psi) + Bs[2,1]*cos(psi) + Cs[2,1])
    q[2] = atan2(GC[1] * (-As[2,2]*sin(psi) - Bs[1,1]*cos(psi) + Cs[1,1]), GC[1] * (As[0,1]*sin(psi) + Bs[0,1]*cos(psi) + Cs[0,1]))

    R3_4 = [cos(q[3]), sin(q[3]), 0; 0, 0, 1; sin(q[3]), -cos(q[3]), 0)
    Aw = np.transpose(R3_4) * np.transpose(As) * R
    Bw = np.transpose(R3_4) * np.transpose(Bs) * R
    Cw = np.transpose(R3_4) * np.transpose(Cs) * R
    R4_7 = [0, 0, cos(q[4])*sin(q[5]); 0, 0, sin(q[4])*sin(q[5]); -sin(q[5])*cos(q[6]), sin(q[5])*sin(q[6]), cos(q[5])]

    q[4] = atan2(GC[5] * (Aw[1,2]*sin(psi) + Bw[1,2]*cos(psi) + Cw[1,2]), GC[5] * (Aw[0,2]*sin(psi) + Bw[0,2]*cos(psi) + Cw[0,2]))
    q[6] = GC[5] * acos(Aw[2,2]*sin(psi) + Bw[2,2]*cos(psi) + Cw[2,2])
    q[7] = atan2(GC[5] * (Aw[2,1]*sin(psi) + Bw[2,1]*cos(psi) + Cw[2,1]), GC[5] * (Aw[2,0]*sin(psi) + Bw[2,0]*cos(psi) + Cw[2,0]))


    return q
