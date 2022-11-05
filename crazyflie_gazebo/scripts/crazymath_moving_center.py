#!/usr/bin/env python

import math

class CopterParams:
    def __init__(self, d, phi, angle, v, vx, vy):
        self.d = d
        self.phi = phi
        self.angle = angle
        self.v = v
        self.vx = vx
        self.vy = vy

class Crazymath3:
    def __init__(self, v_cruise, v_f, k_f, D_12, D_23, k, R):
        # self.CX = CX
        # self.CY = CY
        self.v_cruise = v_cruise
        self.v_f = v_f
        self.k_f = k_f
        self.D_12 = D_12
        self.D_23 = D_23
        self.k = k
        self.R = R

    def calculate(self, px1, py1, px2, py2, px3, py3, C_X, C_Y):
        d_1, phi_1 = self.distance_to_centre(px1, py1, C_X, C_Y)
        angle_1 = self.phase_angle(d_1, phi_1)

        d_2, phi_2 = self.distance_to_centre(px2, py2, C_X, C_Y)
        angle_2 = self.phase_angle(d_2, phi_2)

        d_3, phi_3 = self.distance_to_centre(px3, py3, C_X, C_Y)
        angle_3 = self.phase_angle(d_3, phi_3)

        p12 = self.phase_shift_ab(px1, py1, px2, py2, C_X, C_Y)
        p23 = self.phase_shift_ab(px2, py2, px3, py3, C_X, C_Y)

        v1, v2, v3 = self.velocity(p12, p23)

        vx1, vy1 = self.get_velocity(v1, angle_1)
        vx2, vy2 = self.get_velocity(v2, angle_2)
        vx3, vy3 = self.get_velocity(v3, angle_3)

        cf1 = CopterParams(d_1, phi_1, angle_1, v1, vx1, vy1)
        cf2 = CopterParams(d_2, phi_2, angle_2, v2, vx2, vy2)
        cf3 = CopterParams(d_3, phi_3, angle_3, v3, vx3, vy3)

        return cf1, cf2, cf3, p12, p23
    
    def phase_shift_ab(self, px_a, py_a, px_b, py_b, CX, CY):
        dot_product = (px_a - CX)*(px_b - CX) + (py_a - CY)*(py_b - CY)
        magnitude_i = math.sqrt((px_a - CX)**2 + (py_a - CY)**2)
        magnitude_j = math.sqrt((px_b - CX)**2 + (py_b - CY)**2)
        triple_product = (px_a - CX)*(py_b - CY) - (px_b - CX)*(py_a - CY)
        p_ab = math.acos(dot_product / (magnitude_i * magnitude_j))
        if triple_product > 0:
            p_ab = 2*math.pi - p_ab
        return p_ab


    def velocity(self, p_12, p_23):
        v1 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (p_12 - self.D_12))
        v2 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f *
                                                    (-p_12 + self.D_12 + p_23 - self.D_23))
        v3 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (-p_23 + self.D_23))
        return (v1, v2, v3)


    def distance_to_centre(self, px, py, CX, CY):
        d = math.sqrt((px - CX)**2 + (py - CY)**2)
        phi = math.atan2(px - CX, py - CY)
        return (d, phi)


    def phase_angle(self, d, phi):
        angle = phi + math.pi/2 + math.atan(self.k * (d - self.R))
        return angle


    def get_velocity(self, v, angle):
        vx = v * math.sin(angle)
        vy = v * math.cos(angle)
        return (vx.real, vy.real)    

class Crazymath2:
    def __init__(self, CX, CY, v_cruise, v_f, k_f, D, k, R):
        # self.CX = CX
        # self.CY = CY
        self.v_cruise = v_cruise
        self.v_f = v_f
        self.k_f = k_f
        self.D = D
        self.k = k
        self.R = R

    def calculate(self, px1, py1, px2, py2, C_X, C_Y):
        d_1, phi_1 = self.distance_to_centre(px1, py1, C_X, C_Y)
        angle_1 = self.phase_angle(d_1, phi_1)

        d_2, phi_2 = self.distance_to_centre(px2, py2, C_X, C_Y)
        angle_2 = self.phase_angle(d_2, phi_2)

        p_12 = self.phase_shift_ab(px1, py1, px2, py2, C_X, C_Y)

        v1, v2 = self.velocity(p_12)

        vx1, vy1 = self.get_velocity(v1, angle_1)
        vx2, vy2 = self.get_velocity(v2, angle_2)

        cf1 = CopterParams(d_1, phi_1, angle_1, v1, vx1, vy1)
        cf2 = CopterParams(d_2, phi_2, angle_2, v2, vx2, vy2)

        return cf1, cf2, p_12
    
    def phase_shift_ab(self, px_a, py_a, px_b, py_b, CX, CY):
        dot_product = (px_a - CX)*(px_b - CX) + (py_a - CY)*(py_b - CY)
        magnitude_i = math.sqrt((px_a - CX)**2 + (py_a - CY)**2)
        magnitude_j = math.sqrt((px_b - CX)**2 + (py_b - CY)**2)
        triple_product = (px_a - CX)*(py_b - CY) - (px_b - CX)*(py_a - CY)
        p_ab = math.acos(dot_product / (magnitude_i * magnitude_j))
        if triple_product > 0:
            p_ab = 2*math.pi - p_ab
        return p_ab


    def velocity(self, p_12):
        v1 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (p_12 - self.D))
        v2 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (-p_12 + self.D))
        return (v1, v2)


    def distance_to_centre(self, px, py, CX, CY):
        d = math.sqrt((px - CX)**2 + (py - CY)**2)
        phi = math.atan2(px - CX, py - CY)
        return (d, phi)


    def phase_angle(self, d, phi):
        angle = phi + math.pi/2 + math.atan(self.k * (d - self.R))
        return angle


    def get_velocity(self, v, angle):
        vx = v * math.sin(angle)
        vy = v * math.cos(angle)
        return (vx.real, vy.real)            