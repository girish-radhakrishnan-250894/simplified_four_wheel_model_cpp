//
// Created by gradhakrish on 11/1/23.
//

#ifndef SIMPLIFIED_FOUR_WHEEL_MODEL_INPUTS_H
#define SIMPLIFIED_FOUR_WHEEL_MODEL_INPUTS_H

#include <memory>
#include <vector>

struct inputs {

    std::vector<double> delta;
    std::vector<double> time;

    double m_s = 1110;             // Sprung Mass [kg]
    double J_x = 627;             // Sprung Mass Inertia : X-Axes (Roll Inertia) [kgm^2]
    double J_y = 2302;            // Sprung Mass Inertia : Y-Axes (Pitch Inertia) [kgm^2]
    double J_z = 2394;            // Sprung Mass Inertia : Z-Axes (Yaw Inertia) [kgm^2]
    double m_u_1 = 52.5;          // Unsprung Mass 1 [kg]
    double m_u_2 = 52.5;          // Unsprung Mass 2 [kg]
    double m_u_3 = 37.5;          // Unsprung Mass 3 [kg]
    double m_u_4 = 37.5;          // Unsprung Mass 4 [kg]
    double I_yp_1 = 3.705;        // Unsprung Inertia 1 : Y-Axis (Rotational Ineretia) [kgm^2]
    double I_yp_2 = 3.705;        // Unsprung Inertia 2 : Y-Axis (Rotational Ineretia) [kgm^2]
    double I_yp_3 = 4.494;        // Unsprung Inertia 3 : Y-Axis (Rotational Ineretia) [kgm^2]
    double I_yp_4 = 4.494;        // Unsprung Inertia $ : Y-Axis (Rotational Ineretia) [kgm^2]
    double h_cg__0  = 0.44181;    // Sprung Mass CG Height [m]

    double weight_distribution = 0.6;

    // Longitudinal dimensions
    double wheelbase = 2.672;
    double a = (1 - weight_distribution)*wheelbase;
    double b = weight_distribution*wheelbase;

    double a_1 = + a;
    double a_2 = + a;
    double a_3 = - b;
    double a_4 = - b;


    // Lateral dimensions
    double s_1 = + 0.795;
    double s_2 = - 0.795;
    double s_3 = + 0.805;
    double s_4 = - 0.805;

    // Component Dimension :- Wheel Unloaded Radii - Corner 'i'
    double r_01 = 0.3277;
    double r_02 = 0.3277;
    double r_03 = 0.3277;
    double r_04 = 0.3277;

    // Component Dimension :- Spring Free Length
    double l_01 = 0.35;
    double l_02 = 0.35;
    double l_03 = 0.35;
    double l_04 = 0.35;

    // Spring Stiffness
    double k_s = 60000;

    // Damping Coefficient
    double d_s = 9000;

    // Tire Vertical Stiffness
    double k_t = 250000;

    // Tire Slip Stiffness
    double C_x = 250000;

    // Tire Cornering Stiffness
    double C_y = 100000;

};


#endif //SIMPLIFIED_FOUR_WHEEL_MODEL_INPUTS_H
