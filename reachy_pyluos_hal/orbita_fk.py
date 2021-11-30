#!/usr/bin/python
# -*- coding: utf-8 -*-

#
#  File Name	: orbita_fk.py
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen.000@gmail.com
#  Created	: mercredi, novembre 24 2021
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	notes
#

import numpy as np
import time

from scipy.optimize import fsolve, root_scalar, least_squares
#from mpmath import findroot
from scipy.spatial.transform import Rotation as Rotation
import logging

# really, Orbita cannot go farther than that for roll or pitch
MAX_ANGLE = np.radians(60)
MAX_TRIES = 5

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class OrbitaFK(object):
    def __init__(self, alpha1=np.radians(50), alpha2=np.radians(90), roll_offset=0, pitch_offset=0, yaw_offset=0):
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.yaw_offset = yaw_offset+np.radians(90)  # 90° for Orbita
        self.roll_offset = roll_offset
        self.pitch_offset = pitch_offset

        self.sa1 = np.sin(self.alpha1)
        self.sa2 = np.sin(self.alpha2)
        self.ca1 = np.cos(self.alpha1)
        self.ca2 = np.cos(self.alpha2)

        self.disks0 = np.array([0, np.radians(120), np.radians(-120)])
        self.curr_theta1, self.curr_theta2, self.curr_theta3 = self.disks0

        self.curr_phi1 = None
        self.curr_phi2 = None
        self.curr_phi3 = None

        self.fk((0,0,0))
    def v1(self, cp1, sp1, cp2, sp2, cp3, sp3, ct1, st1, ct2, st2, ct3, st3):
        sa1 = self.sa1
        sa2 = self.sa2
        ca1 = self.ca1
        ca2 = self.ca2
        return np.array([[-ca1*cp1*sa2*st1 + ca2*sa1*st1 + ct1*sa2*sp1], [ca1*cp1*ct1*sa2 - ca2*ct1*sa1 + sa2*sp1*st1], [-ca1*ca2 - cp1*sa1*sa2]])

    def v2(self, cp1, sp1, cp2, sp2, cp3, sp3, ct1, st1, ct2, st2, ct3, st3):
        sa1 = self.sa1
        sa2 = self.sa2
        ca1 = self.ca1
        ca2 = self.ca2
        return np.array([[-ca1*cp2*sa2*st2 + ca2*sa1*st2 + ct2*sa2*sp2], [ca1*cp2*ct2*sa2 - ca2*ct2*sa1 + sa2*sp2*st2], [-ca1*ca2 - cp2*sa1*sa2]])

    def v3(self, cp1, sp1, cp2, sp2, cp3, sp3, ct1, st1, ct2, st2, ct3, st3):
        sa1 = self.sa1
        sa2 = self.sa2
        ca1 = self.ca1
        ca2 = self.ca2
        return np.array([[-ca1*cp3*sa2*st3 + ca2*sa1*st3 + ct3*sa2*sp3], [ca1*cp3*ct3*sa2 - ca2*ct3*sa1 + sa2*sp3*st3], [-ca1*ca2 - cp3*sa1*sa2]])

    def w1(self, ct1, st1, ct2, st2, ct3, st3):
        sa1 = self.sa1
        # sa2 = self.sa2
        ca1 = self.ca1
        # ca2 = self.ca2
        return np.array([[sa1*st1], [-ct1*sa1], [-ca1]])

    def w2(self, ct1, st1, ct2, st2, ct3, st3):
        sa1 = self.sa1
        # sa2 = self.sa2
        ca1 = self.ca1
        # ca2 = self.ca2
        return np.array([[sa1*st2], [-ct2*sa1], [-ca1]])

    def w3(self, ct1, st1, ct2, st2, ct3, st3):
        sa1 = self.sa1
        # sa2 = self.sa2
        ca1 = self.ca1
        # ca2 = self.ca2
        return np.array([[sa1*st3], [-ct3*sa1], [-ca1]])

    def eqs_fk(self, phis):

        (cp1, sp1, cp2, sp2, cp3, sp3) = phis
        st1 = np.sin(self.curr_theta1)
        st2 = np.sin(self.curr_theta2)
        st3 = np.sin(self.curr_theta3)
        ct1 = np.cos(self.curr_theta1)
        ct2 = np.cos(self.curr_theta2)
        ct3 = np.cos(self.curr_theta3)

        v1n = self.v1(cp1, sp1, cp2, sp2, cp3, sp3,
                      ct1, st1, ct2, st2, ct3, st3).flatten()
        v2n = self.v2(cp1, sp1, cp2, sp2, cp3, sp3,
                      ct1, st1, ct2, st2, ct3, st3).flatten()
        v3n = self.v3(cp1, sp1, cp2, sp2, cp3, sp3,
                      ct1, st1, ct2, st2, ct3, st3).flatten()

        w1n = self.w1(ct1, st1, ct2, st2, ct3, st3).flatten()
        w2n = self.w2(ct1, st1, ct2, st2, ct3, st3).flatten()
        w3n = self.w3(ct1, st1, ct2, st2, ct3, st3).flatten()

        res1 = v1n.dot(v2n.T)+0.5
        res2 = v2n.dot(v3n.T)+0.5
        res3 = v3n.dot(v1n.T)+0.5
        res4 = w1n.dot(v1n.T)-self.ca2
        res5 = w2n.dot(v2n.T)-self.ca2
        res6 = w3n.dot(v3n.T)-self.ca2
        res7 = v1n.dot(v1n.T)-1
        res8 = v2n.dot(v2n.T)-1
        res9 = v3n.dot(v3n.T)-1
        res10 = np.array(v1n+v2n+v3n, dtype=object)

        res = [res1, res2, res3, res4, res5, res6, res7,
               res8, res9, res10[0], res10[1], res10[2]]

        return res

    def get_phi(self, theta1, theta2, theta3, initial_guess):

        # self.curr_theta1, self.curr_theta2, self.curr_theta3 = (
        #     theta1, theta2, theta3)+self.disks0

        self.curr_theta1, self.curr_theta2, self.curr_theta3 = (
            theta1, theta2, theta3)

        # self.curr_tetha1 = theta1
        # self.curr_tetha2 = theta2
        # self.curr_tetha3 = theta3

        # res = fsolve(self.fk_eqs_opt, self.initial_guess)
        # res = root_scalar(self.fk_eqs_opt, x0=self.initial_guess)

        # res = findroot(self.fk_eqs_opt, x0=self.initial_guess,
        # tol=1e-6)
        # res = least_squares(self.fk_eqs_opt, x0=self.initial_guess)
        # least squares seems the quickest method here

        if initial_guess is not None:
            init = initial_guess
        else:
            if self.curr_phi1 is not None and self.curr_phi2 is not None and self.curr_phi3 is not None:
                init = (np.cos(self.curr_phi1), np.sin(self.curr_phi1), np.cos(self.curr_phi2), np.sin(
                    self.curr_phi2), np.cos(self.curr_phi3), np.sin(self.curr_phi3))
                logger.debug(f'Init {init}')
            else:
                init = (0, 1, 0, 1, 0, 1)  # cp1, sp1, cp2,sp2,cp3,sp3

        # phi in [45°, 180°]

        # bounds for dogbox method
        # bounds = ([-1, 0, -1, 0, -1, 0], [np.cos(np.radians(25)), 1, np.cos(
        #    np.radians(25)), 1, np.cos(np.radians(25)), 1])

        # It seems that we find the correct solution from here...
        init = (0, 1, 0, 1, 0, 1)  # cp1, sp1, cp2,sp2,cp3,sp3

        # maybe check params if we can make it faster? (tol?)
        # res = least_squares(self.eqs_fk, x0=init,
        #                     bounds=bounds, method='dogbox')
        # res = least_squares(self.eqs_fk, x0=init, bounds=bounds)
        res = least_squares(self.eqs_fk, x0=init, method='lm')

        return res

    def get_sol(self, thetas, initial_guess):

        theta1, theta2, theta3 = thetas+self.disks0
        resopt = self.get_phi(theta1, theta2, theta3, initial_guess)
        # TODO check if solution is valid (it returns a OptimizeResult object)
        sols = resopt['x']

        sp1_n = float(sols[1])
        sp2_n = float(sols[3])
        sp3_n = float(sols[5])

        cp1_n = float(sols[0])
        cp2_n = float(sols[2])
        cp3_n = float(sols[4])

        phi1 = np.arctan2(sp1_n, cp1_n)
        phi2 = np.arctan2(sp2_n, cp2_n)
        phi3 = np.arctan2(sp3_n, cp3_n)

        self.curr_phi1 = phi1
        self.curr_phi2 = phi2
        self.curr_phi3 = phi3

        logger.debug(f'phi1: {phi1} ({np.cos(phi1)} {np.sin(phi1)}) phi2: {phi2} ({np.cos(phi2)} {np.sin(phi2)}) phi3: {phi3} ({np.cos(phi3)} {np.sin(phi3)})')
        logger.debug(f'theta1: {theta1} ({np.degrees(theta1)}) theta2: {theta2} ({np.degrees(theta2)}) theta3: {theta3} ({np.degrees(theta3)})')

        st1 = np.sin(theta1)
        st2 = np.sin(theta2)
        st3 = np.sin(theta3)

        ct1 = np.cos(theta1)
        ct2 = np.cos(theta2)
        ct3 = np.cos(theta3)

        v1sol = self.v1(cp1_n, sp1_n, cp2_n, sp2_n, cp3_n,
                        sp3_n, ct1, st1, ct2, st2, ct3, st3)
        v2sol = self.v2(cp1_n, sp1_n, cp2_n, sp2_n, cp3_n,
                        sp3_n, ct1, st1, ct2, st2, ct3, st3)
        v3sol = self.v3(cp1_n, sp1_n, cp2_n, sp2_n, cp3_n,
                        sp3_n, ct1, st1, ct2, st2, ct3, st3)

        # donc bizarement on a [y, -x, z] (rotation de 90°)
        v1sol = np.array([-v1sol[1][0], v1sol[0][0],
                          v1sol[2][0]], dtype=np.float64)
        v2sol = np.array([-v2sol[1][0], v2sol[0][0],
                          v2sol[2][0]], dtype=np.float64)
        v3sol = np.array([-v3sol[1][0], v3sol[0][0],
                          v3sol[2][0]], dtype=np.float64)

        V_mat = np.matrix([v1sol, v2sol, v3sol])
        # config de base
        b1 = np.array([np.cos(np.radians(0)), np.sin(np.radians(0)), 0])
        b2 = np.array([np.cos(np.radians(120)), np.sin(np.radians(120)), 0])
        b3 = np.array([np.cos(np.radians(-120)), np.sin(np.radians(-120)), 0])

        # dans le repère Orbita:
        # yaw_offset = -np.pi/2.0
        # yaw_offset=0
        roffset = Rotation.from_euler(
            'xyz', [self.roll_offset, self.pitch_offset, self.yaw_offset])
        Qoffset = np.array(roffset.as_matrix())
        b1 = Qoffset.dot(b1)
        b2 = Qoffset.dot(b2)
        b3 = Qoffset.dot(b3)
        B_mat = np.matrix([b1, b2, b3])

        ra, rms = Rotation.align_vectors(V_mat, B_mat)
        return ra

    def fk(self, thetas, initial_guess=None):

        valid = False
        nb_tries = 0
        r = None
        while not valid:

            r = self.get_sol(thetas, initial_guess)
            roll, pitch, yaw = r.as_euler('xyz')
            if np.abs(roll) > MAX_ANGLE or np.abs(pitch) > MAX_ANGLE:
                logger.warning("Solution looks suspicious")
                valid = True
            else:
                # TODO retry?
                init = (0, 1, 0, 1, 0, 1)  # cp1, sp1, cp2,sp2,cp3,sp3
                r = self.get_sol(thetas, init)
                # roll, pitch, yaw = r.as_euler('xyz')
                valid = True

        rpy=r.as_euler('xyz', degrees=True)

        logger.warning(f'{np.degrees(thetas)} ({np.degrees(self.curr_theta1)} {np.degrees(self.curr_theta2)} {np.degrees(self.curr_theta3)}) {rpy}')
        return r


if __name__ == '__main__':
    fk = OrbitaFK()

    # rpy=0,0,0
    # theta1 = 0.
    # theta2 = 2.0943951
    # theta3 = -2.0943951

    # rpy: 20,20,0
    theta1 = 0.16494017
    theta2 = 2.17988495
    theta3 = -2.57287471

    # rpy:20,0,0
    # theta1 = 0.31036613
    # theta2 = 1.921728
    # theta3 = -2.21404742

    theta1, theta2, theta3 = np.array([theta1, theta2, theta3]) - \
        np.array([0, np.radians(120), np.radians(-120)])

    t0 = time.time()
    r = fk.fk((theta1, theta2, theta3))
    t1 = time.time()
    print(f'{t1-t0}s')
    print(f'theta1: {np.degrees(theta1)} theta2: {np.degrees(theta2)} theta2: {np.degrees(theta2)}')
    print(r.as_euler('xyz', degrees=True))
