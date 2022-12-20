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

class Crazymath3wAPF:
    def __init__(self, CX, CY, v_cruise, v_f, k_f, D_12, D_23, k, R, safety_radius, eta_vf, eta_apf):
        self.CX = CX
        self.CY = CY
        self.v_cruise = v_cruise
        self.v_f = v_f
        self.k_f = k_f
        self.D_12 = D_12
        self.D_23 = D_23
        self.k = k
        self.R = R
        self.safety_radius = safety_radius
        self.eta_vf = eta_vf
        self.eta_apf = eta_apf

    def calculate(self, px1, py1, px2, py2, px3, py3):

        distance_12 = self.distance_between_copters(px1, py1, px2, py2)

        d_1, phi_1 = self.distance_to_centre(px1, py1)
        angle_1 = self.phase_angle(d_1, phi_1)
        
        d_2, phi_2 = self.distance_to_centre(px2, py2)
        angle_2 = self.phase_angle(d_2, phi_2)

        d_3, phi_3 = self.distance_to_centre(px3, py3)
        angle_3 = self.phase_angle(d_3, phi_3)

        p12_APF = self.phase_shift_ab_APF(distance_12, px1, py1, px2, py2)
        p23 = self.phase_shift_ab(px2, py2, px3, py3)

        v1, v2, v3 = self.velocity(distance_12, p12_APF, p23)
        # v1, v2 = 0.5, 0.2

        # vx1, vy1 = self.get_velocity_APF(self, distance_12, v1, angle_1, px1, py1, px2, py2)
        # vx2, vy2 = self.get_velocity_APF(self, distance_12, v2, angle_2, px2, py2, px1, py1)
        
        
        angle_1_APF = self.get_angle_APF(distance_12, v1, angle_1, px1, py1, px2, py2)
        angle_2_APF = self.get_angle_APF(distance_12, v2, angle_2, px2, py2, px1, py1)


        vx1, vy1 = self.get_velocity(v1, angle_1_APF)
        vx2, vy2 = self.get_velocity(v2, angle_2_APF)
        vx3, vy3 = self.get_velocity(v3, angle_3)

        
        cf1 = CopterParams(d_1, phi_1, angle_1, v1, vx1, vy1)
        cf2 = CopterParams(d_2, phi_2, angle_2, v2, vx2, vy2)
        cf3 = CopterParams(d_3, phi_3, angle_3, v3, vx3, vy3)

        

        return cf1, cf2, cf3, p12_APF, p23, distance_12, angle_1_APF, angle_2_APF
    
    def phase_shift_ab(self, px_a, py_a, px_b, py_b):
        dot_product = (px_a - self.CX)*(px_b - self.CX) + (py_a - self.CY)*(py_b - self.CY)
        magnitude_i = math.sqrt((px_a - self.CX)**2 + (py_a - self.CY)**2)
        magnitude_j = math.sqrt((px_b - self.CX)**2 + (py_b - self.CY)**2)
        triple_product = (px_a - self.CX)*(py_b - self.CY) - (px_b - self.CX)*(py_a - self.CY)
        p_ab = math.acos(dot_product / (magnitude_i * magnitude_j))
        if triple_product > 0:
            p_ab = 2*math.pi - p_ab
        return p_ab

    def phase_shift_ab_APF(self, distance_ab, px_a, py_a, px_b, py_b):
        dot_product = (px_a - self.CX)*(px_b - self.CX) + (py_a - self.CY)*(py_b - self.CY)
        magnitude_i = math.sqrt((px_a - self.CX)**2 + (py_a - self.CY)**2)
        magnitude_j = math.sqrt((px_b - self.CX)**2 + (py_b - self.CY)**2)
        triple_product = (px_a - self.CX)*(py_b - self.CY) - (px_b - self.CX)*(py_a - self.CY)
        p_ab_APF = math.acos(dot_product / (magnitude_i * magnitude_j))
        # if triple_product < 0:
        p_ab_APF = 2*math.pi - p_ab_APF
        # else if or distance_ab < self.safety_radius
        return p_ab_APF

    def distance_between_copters(self, px_a, py_a, px_b, py_b):
        distance_ab=math.sqrt((px_a - px_b)**2 + (py_a - py_b)**2)
        return distance_ab

    # Calculate course angles via Artificial Potencial Field (APF):
    def get_angle_APF(self, distance_ab, v, angle, px_a, py_a, px_b, py_b):
        triple_product = (px_a - self.CX)*(py_b - self.CY) - (px_b - self.CX)*(py_a - self.CY)
        if distance_ab < self.safety_radius:
        # if triple_product < 0:
            vx_APF = self.eta_vf * v * math.sin(angle) - self.eta_apf * (-1/self.safety_radius+1/distance_ab) * (py_a - py_b) / distance_ab**3
            vy_APF = self.eta_vf * v * math.cos(angle) + self.eta_apf * (-1/self.safety_radius+1/distance_ab) * (px_a - px_b) / distance_ab**3
            angle_APF = math.atan2(vx_APF, vy_APF)
        else:
            angle_APF = angle
        return (angle_APF)    

    def velocity(self, distance_ab, p_12, p_23):
        if distance_ab < self.safety_radius:
            v1 = self.v_cruise
            v2 = self.v_cruise - 0.2
        else:
            v1 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (p_12 - self.D_12))
            v2 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f *
                                                    (-p_12 + self.D_12 + p_23 - self.D_23))
        v3 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (-p_23 + self.D_23))
        return (v1, v2, v3)

    def distance_to_centre(self, px, py):
        d = math.sqrt((px - self.CX)**2 + (py - self.CY)**2)
        phi = math.atan2(px - self.CX, py - self.CY)
        return (d, phi)


    def phase_angle(self, d, phi):
        angle = phi + math.pi/2 + math.atan(self.k * (d - self.R))
        return angle


    def get_velocity(self, v, angle_APF):
        vx = v * math.sin(angle_APF)
        vy = v * math.cos(angle_APF)
        return (vx.real, vy.real)    

# class Crazymath3:
#     def __init__(self, CX, CY, v_cruise, v_f, k_f, D_12, D_23, k, R):
#         self.CX = CX
#         self.CY = CY
#         self.v_cruise = v_cruise
#         self.v_f = v_f
#         self.k_f = k_f
#         self.D_12 = D_12
#         self.D_23 = D_23
#         self.k = k
#         self.R = R

#     def calculate(self, px1, py1, px2, py2, px3, py3):
#         d_1, phi_1 = self.distance_to_centre(px1, py1)
#         angle_1 = self.phase_angle(d_1, phi_1)

#         d_2, phi_2 = self.distance_to_centre(px2, py2)
#         angle_2 = self.phase_angle(d_2, phi_2)

#         d_3, phi_3 = self.distance_to_centre(px3, py3)
#         angle_3 = self.phase_angle(d_3, phi_3)

#         p12 = self.phase_shift_ab(px1, py1, px2, py2)
#         p23 = self.phase_shift_ab(px2, py2, px3, py3)

#         v1, v2, v3 = self.velocity(p12, p23)

#         vx1, vy1 = self.get_velocity(v1, angle_1)
#         vx2, vy2 = self.get_velocity(v2, angle_2)
#         vx3, vy3 = self.get_velocity(v3, angle_3)

#         cf1 = CopterParams(d_1, phi_1, angle_1, v1, vx1, vy1)
#         cf2 = CopterParams(d_2, phi_2, angle_2, v2, vx2, vy2)
#         cf3 = CopterParams(d_3, phi_3, angle_3, v3, vx3, vy3)

#         return cf1, cf2, cf3, p12, p23
    
#     def phase_shift_ab(self, px_a, py_a, px_b, py_b):
#         dot_product = (px_a - self.CX)*(px_b - self.CX) + (py_a - self.CY)*(py_b - self.CY)
#         magnitude_i = math.sqrt((px_a - self.CX)**2 + (py_a - self.CY)**2)
#         magnitude_j = math.sqrt((px_b - self.CX)**2 + (py_b - self.CY)**2)
#         triple_product = (px_a - self.CX)*(py_b - self.CY) - (px_b - self.CX)*(py_a - self.CY)
#         p_ab = math.acos(dot_product / (magnitude_i * magnitude_j))
#         if triple_product > 0:
#             p_ab = 2*math.pi - p_ab
#         return p_ab


#     def velocity(self, p_12, p_23):
#         v1 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (p_12 - self.D_12))
#         v2 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f *
#                                                     (-p_12 + self.D_12 + p_23 - self.D_23))
#         v3 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (-p_23 + self.D_23))
#         return (v1, v2, v3)


#     def distance_to_centre(self, px, py):
#         d = math.sqrt((px - self.CX)**2 + (py - self.CY)**2)
#         phi = math.atan2(px - self.CX, py - self.CY)
#         return (d, phi)


#     def phase_angle(self, d, phi):
#         angle = phi + math.pi/2 + math.atan(self.k * (d - self.R))
#         return angle


#     def get_velocity(self, v, angle):
#         vx = v * math.sin(angle)
#         vy = v * math.cos(angle)
#         return (vx.real, vy.real)    

class Crazymath2:
    def __init__(self, CX, CY, v_cruise, v_f, k_f, D, k, R):
        self.CX = CX
        self.CY = CY
        self.v_cruise = v_cruise
        self.v_f = v_f
        self.k_f = k_f
        self.D = D
        self.k = k
        self.R = R

    def calculate(self, px1, py1, px2, py2):
        d_1, phi_1 = self.distance_to_centre(px1, py1)
        angle_1 = self.phase_angle(d_1, phi_1)

        d_2, phi_2 = self.distance_to_centre(px2, py2)
        angle_2 = self.phase_angle(d_2, phi_2)

        p_12 = self.phase_shift_ab(px1, py1, px2, py2)

        v1, v2 = self.velocity(p_12)

        vx1, vy1 = self.get_velocity(v1, angle_1)
        vx2, vy2 = self.get_velocity(v2, angle_2)

        cf1 = CopterParams(d_1, phi_1, angle_1, v1, vx1, vy1)
        cf2 = CopterParams(d_2, phi_2, angle_2, v2, vx2, vy2)

        return cf1, cf2, p_12
    
    def phase_shift_ab(self, px_a, py_a, px_b, py_b):
        dot_product = (px_a - self.CX)*(px_b - self.CX) + (py_a - self.CY)*(py_b - self.CY)
        magnitude_i = math.sqrt((px_a - self.CX)**2 + (py_a - self.CY)**2)
        magnitude_j = math.sqrt((px_b - self.CX)**2 + (py_b - self.CY)**2)
        triple_product = (px_a - self.CX)*(py_b - self.CY) - (px_b - self.CX)*(py_a - self.CY)
        p_ab = math.acos(dot_product / (magnitude_i * magnitude_j))
        if triple_product > 0:
            p_ab = 2*math.pi - p_ab
        return p_ab


    def velocity(self, p_12):
        v1 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (p_12 - self.D))
        v2 = self.v_cruise + self.v_f * (2 / math.pi) * math.atan(self.k_f * (-p_12 + self.D))
        return (v1, v2)


    def distance_to_centre(self, px, py):
        d = math.sqrt((px - self.CX)**2 + (py - self.CY)**2)
        phi = math.atan2(px - self.CX, py - self.CY)
        return (d, phi)


    def phase_angle(self, d, phi):
        angle = phi + math.pi/2 + math.atan(self.k * (d - self.R))
        return angle


    def get_velocity(self, v, angle):
        vx = v * math.sin(angle)
        vy = v * math.cos(angle)
        return (vx.real, vy.real)            