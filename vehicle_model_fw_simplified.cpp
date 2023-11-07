//
// Created by gradhakrish on 11/1/23.
//

#include "vehicle_model_fw_simplified.h"

Eigen::VectorXd vehicle_model_fw_simplified::solve(Eigen::VectorXd q, const double &delta_c,
                                                    const double &m_d_c)
{
    //%vehicle_model_fw_simplified Simplified, abstract, four-wheel vehicle model
    //%   This is a four-wheel vehicle model formulated in a simplified, abstract
    //%   way. Small angle approximations are used substantially in this
    //%   formulation. Slip angle/ratio are calculated using the slip of the
    //%   chassis corner instead of slip of the contact patch.
    //% INPUTS
    //%   q           State-vector in 1st order formulation which contains
    //%               generalized positions & velocities
    //%   input       Input structu which contains vehicle parameter information
    //%               such as mass, inertia, tire model etc.
    //%   delta_c     Steering input command in radian
    //%   m_d_c       Accelerating/Braking torque input command in radian
    //
    //% NOTE: Check the "input_script.m" to understand the inputs inside the
    //%       input structure
    //
    //% OUTPUTS
    //% NOTE- The vectors and matrices that this function outputs are done so
    //% keeping in mind future operations that this function will be used for
    //% EXAMPLE - This function may need to output controller states or estimator/observer states. The O_simulator will do that
    //% EXAMPLE - The vehicle model may need to output complex variables like slip angle, slip ratio etc. The O_model will do that
    //
    //%   Qdot        State vector time derivative
    //%   f_qd_q_u    Forces & moments vector
    //%   M           Mass matrix
    //%   O           Model Outputs
    //
    //
    //% NOTE
    //% __c   indicates chassis frame of refernece
    //% __0   indicates world frame of reference
    //% __i   indicates i frame of reference (i = 1,2,3,4)

    // Initialization : System Variables
    double x = q(0);
    double y = q(1);
    double z = q(2);
    double theta = q(3);
    double phi = q(4);
    double psi = q(5);
    double z_1 = q(6);
    double z_2 = q(7);
    double z_3 = q(8);
    double z_4 = q((9));

    double x_d = q(14);
    double y_d = q(15);
    double z_d = q(16);
    double co_omega__0_1 = q(17);
    double co_omega__0_2 = q(18);
    double co_omega__0_3 = q(19);
    double z_d_1 = q(20);
    double z_d_2 = q(21);
    double z_d_3 = q(22);
    double z_d_4 = q(23);
    double omega_1 = q(24);
    double omega_2 = q(25);
    double omega_3 = q(26);
    double omega_4 = q(27);

    // Initialization : Vehicle Input Structure
    auto in = model_input;

    // Rotation Matrices - Chassis
    Eigen::MatrixXd A_ao = Eigen::Matrix3d::Zero();
    A_ao <<  cos(psi), sin(psi), 0,
            -sin(psi), cos(psi), 0,
             0,          0,           1;

    Eigen::MatrixXd A_ba = Eigen::Matrix3d::Zero();
    A_ba << cos(phi), 0, -sin(phi),
            0,          1,           0,
            sin(phi), 0, cos(phi);

    Eigen::MatrixXd A_cb = Eigen::Matrix3d::Zero();
    A_cb << 1, 0, 0,
            0, cos(theta), sin(theta),
            0, -sin(theta), cos(theta);

    Eigen::MatrixXd A_co = A_cb*A_ba*A_ao;

    // Rotation Matrices - NSM - Corner 1
    Eigen::MatrixXd A_a1o = Eigen::Matrix3d::Zero();
    A_a1o << cos(psi + delta_c), sin(psi + delta_c), 0,
             -sin(psi + delta_c), cos(psi + delta_c), 0,
             0, 0, 1;

    Eigen::MatrixXd A_1o = A_cb*A_ba*A_a1o;

    // Rotation Matrices - NSM - Corner 2
    Eigen::MatrixXd A_a20 = A_1o;

    Eigen::MatrixXd A_20 = A_cb*A_ba*A_a1o;

    // Rotation Matrices - NSM - Corner 3
    Eigen::MatrixXd A_30 = A_co;

    // Rotation Matrices - NSM - Corner 5
    Eigen::MatrixXd A_40 = A_co;


    // Angular Velocity Vector
    Eigen::Vector3d co_omega__0 = Eigen::Vector3d::Zero();
    co_omega__0 << co_omega__0_1, co_omega__0_2, co_omega__0_3;

    Eigen::Vector3d co_omega__c = Eigen::Vector3d::Zero();
    co_omega__c = (co_omega__0.transpose()*A_co.transpose()).transpose();

    double r{co_omega__c(2)};

    // Cardan Angle Velocities
    /* Since the equations of motion are calculated using angular velocity vector
     * Hence the integration of this vector will NOT yield the cardan angles
     * Integrating the calculations below is what will yeild the cardan angles
     */
    Eigen::VectorXd tait_bryant_angle_d_0 = Eigen::Vector3d::Zero();
    tait_bryant_angle_d_0 << (co_omega__0_1*cos(psi) + co_omega__0_2*sin(psi))/cos(phi),
                             co_omega__0_2*cos(psi) - co_omega__0_1*sin(psi),
                             (co_omega__0_3*cos(phi) + co_omega__0_1*cos(psi)*sin(phi) + co_omega__0_2*sin(phi)*sin(psi))/cos(phi);

    // Cardan Angle velocities
    double theta_d{tait_bryant_angle_d_0(0)};
    double phi_d{tait_bryant_angle_d_0(1)};
    double psi_d{tait_bryant_angle_d_0(2)};


    // Chassis CG Velocity
    Eigen::Vector3d r_cm_d__0{x_d, y_d, z_d};

    Eigen::Vector3d r_cm_d__c{(r_cm_d__0.transpose()*A_co.transpose()).transpose()};

    double u{r_cm_d__c(0)};
    double v{r_cm_d__c(1)};

    // Spring Defelction
    double Delta_s_1{ z_1 - z + in.a_1*phi - in.s_1*theta };
    double Delta_s_2{ z_2 - z + in.a_2*phi - in.s_2*theta };
    double Delta_s_3{ z_3 - z + in.a_3*phi - in.s_3*theta };
    double Delta_s_4{ z_4 - z + in.a_4*phi - in.s_4*theta };

    // Spring Length
    double l_1__c{ in.l_01 - Delta_s_1 };
    double l_2__c{ in.l_02 - Delta_s_2 };
    double l_3__c{ in.l_03 - Delta_s_3 };
    double l_4__c{ in.l_04 - Delta_s_4 };

    // Damper Velocities
    double Delta_d_s_1{ z_d_1 - z_d + in.a_1*phi_d - in.s_1*theta_d };
    double Delta_d_s_2{ z_d_2 - z_d + in.a_2*phi_d - in.s_2*theta_d };
    double Delta_d_s_3{ z_d_3 - z_d + in.a_3*phi_d - in.s_3*theta_d };
    double Delta_d_s_4{ z_d_4 - z_d + in.a_4*phi_d - in.s_4*theta_d };

    // Tire Deflection
    double Delta_t_1{ -z_1 };
    double Delta_t_2{ -z_2 };
    double Delta_t_3{ -z_3 };
    double Delta_t_4{ -z_4 };

    // Tire Loaded Radius
    double r_l_1{ in.r_01 - Delta_t_1 };
    double r_l_2{ in.r_02 - Delta_t_2 };
    double r_l_3{ in.r_03 - Delta_t_3 };
    double r_l_4{ in.r_04 - Delta_t_4 };

    // Slip Angles
    // Corner 1
    double Vx_1 =   (u - in.s_1*r)*cos(delta_c) + (v + in.a_1*r)*sin(delta_c);
    double Vsy_1 = -(u - in.s_1*r)*sin(delta_c) + (v + in.a_1*r)*cos(delta_c);
    double alpha_1__1 = -(Vsy_1/Vx_1);
    double kappa_1__1 = -(Vx_1 - omega_1*r_l_1)/(Vx_1);

    // Corner 2
    double Vx_2 =   (u - in.s_2*r)*cos(delta_c) + (v + in.a_2*r)*sin(delta_c);
    double Vsy_2 = -(u - in.s_2*r)*sin(delta_c) + (v + in.a_2*r)*cos(delta_c);
    double alpha_2__2 = -(Vsy_2/Vx_2);
    double kappa_2__2 = -(Vx_2 - omega_2*r_l_2)/(Vx_2);

    // Corner 3
    double Vx_3 =   (u - in.s_3*r)*cos(0) + (v + in.a_3*r)*sin(0);
    double Vsy_3 = -(u - in.s_3*r)*sin(0) + (v + in.a_3*r)*cos(0);
    double alpha_3__3 = -(Vsy_3/Vx_3);
    double kappa_3__3 = -(Vx_3 - omega_3*r_l_3)/(Vx_3);

    // Corner 4
    double Vx_4 =   (u - in.s_4*r)*cos(0) + (v + in.a_4*r)*sin(0);
    double Vsy_4 = -(u - in.s_4*r)*sin(0) + (v + in.a_4*r)*cos(0);
    double alpha_4__4 = -(Vsy_4/Vx_4);
    double kappa_4__4 = -(Vx_4 - omega_4*r_l_4)/(Vx_4);


    // Vector Formulation : Chassis CG to Contact Patch (for moment calculations)
    Eigen::Vector3d r_cp1_cm__c;
    r_cp1_cm__c << in.a_1, in.s_1, -(l_1__c + r_l_1);
    Eigen::Vector3d r_cp2_cm__c;
    r_cp2_cm__c << in.a_2, in.s_2, -(l_2__c + r_l_2);
    Eigen::Vector3d r_cp3_cm__c;
    r_cp3_cm__c << in.a_3, in.s_3, -(l_3__c + r_l_3);
    Eigen::Vector3d r_cp4_cm__c;
    r_cp4_cm__c << in.a_4, in.s_4, -(l_4__c + r_l_4);

    Eigen::Vector3d r_cp1_cm__0{ (r_cp1_cm__c.transpose()*A_co).transpose() };
    Eigen::Vector3d r_cp2_cm__0{ (r_cp2_cm__c.transpose()*A_co).transpose() };
    Eigen::Vector3d r_cp3_cm__0{ (r_cp3_cm__c.transpose()*A_co).transpose() };
    Eigen::Vector3d r_cp4_cm__0{ (r_cp4_cm__c.transpose()*A_co).transpose() };

    // Forces - Spring
    Eigen::Vector3d F_s1_0; F_s1_0 << 0, 0, in.k_s*Delta_s_1;
    Eigen::Vector3d F_s2_0; F_s2_0 << 0, 0, in.k_s*Delta_s_2;
    Eigen::Vector3d F_s3_0; F_s3_0 << 0, 0, in.k_s*Delta_s_3;
    Eigen::Vector3d F_s4_0; F_s4_0 << 0, 0, in.k_s*Delta_s_4;

    // Forces - Damper
    Eigen::Vector3d F_d1_0; F_d1_0 << 0, 0, in.d_s*Delta_d_s_1;
    Eigen::Vector3d F_d2_0; F_d2_0 << 0, 0, in.d_s*Delta_d_s_2;
    Eigen::Vector3d F_d3_0; F_d3_0 << 0, 0, in.d_s*Delta_d_s_3;
    Eigen::Vector3d F_d4_0; F_d4_0 << 0, 0, in.d_s*Delta_d_s_4;

    // Forces - Tire Vertical
    Eigen::Vector3d F_kt1_0; F_kt1_0 << 0, 0, in.k_t*Delta_t_1;
    Eigen::Vector3d F_kt2_0; F_kt2_0 << 0, 0, in.k_t*Delta_t_2;
    Eigen::Vector3d F_kt3_0; F_kt3_0 << 0, 0, in.k_t*Delta_t_3;
    Eigen::Vector3d F_kt4_0; F_kt4_0 << 0, 0, in.k_t*Delta_t_4;

    // Force - Tire Lateral/Longitudinal
    Eigen::Vector3d F_cp1__1; F_cp1__1 << in.C_x*kappa_1__1, in.C_y*alpha_1__1, 0;
    Eigen::Vector3d F_cp2__2; F_cp2__2 << in.C_x*kappa_2__2, in.C_y*alpha_2__2, 0;
    Eigen::Vector3d F_cp3__3; F_cp3__3 << in.C_x*kappa_3__3, in.C_y*alpha_3__3, 0;
    Eigen::Vector3d F_cp4__4; F_cp4__4 << in.C_x*kappa_4__4, in.C_y*alpha_4__4, 0;

    Eigen::Vector3d F_cp1__0{ (F_cp1__1.transpose()*A_a1o).transpose() };
    Eigen::Vector3d F_cp2__0{ (F_cp2__2.transpose()*A_a20).transpose() };
    Eigen::Vector3d F_cp3__0{ (F_cp3__3.transpose()*A_ao).transpose() };
    Eigen::Vector3d F_cp4__0{ (F_cp4__4.transpose()*A_ao).transpose() };


    // Matrix Formulation : Mass & Inertia Matrix
    // Mass Matrix - Suspended Mass
    Eigen::Matrix3d ms_mat;
    ms_mat << in.m_s, 0,      0,
              0,      in.m_s, 0,
              0,      0,      in.m_s;

    // Inertia Matrix - Suspended Mass
    Eigen::Matrix3d jmat__c;
    jmat__c << in.J_x, 0,      0,
               0,      in.J_y, 0,
               0,      0,      in.J_z;

    Eigen::Matrix3d jmat__0{ (A_co.transpose()*jmat__c*A_co) };

    // Mass Matrix - Non-Suspended Mass
    Eigen::Matrix4d mu_mat;
    mu_mat << in.m_u_1, 0,        0,        0,
              0,        in.m_u_2, 0,        0,
              0,        0,        in.m_u_3, 0,
              0,        0,        0,        in.m_u_4;

    // Inertia Matrix - Wheel rotational inertia
    Eigen::Matrix4d jmat_y_i;
    jmat_y_i << in.I_yp_1, 0,         0,         0,
                0,         in.I_yp_2, 0,         0,
                0,         0,         in.I_yp_3, 0,
                0,         0,         0,         in.I_yp_4;


    // Mass & Inertia Matrix - Conatenated - All 14 degress of freedom
    Eigen::MatrixXd m_mat__0 = Eigen::MatrixXd::Zero(14,14);
    Eigen::MatrixXd rep_jmat = Eigen::MatrixXd::Zero(jmat__0.rows(), jmat__0.cols());
    Eigen::MatrixXd rep_msmat = Eigen::MatrixXd::Zero(ms_mat.rows(), ms_mat.cols());
    Eigen::MatrixXd zeros_4_3 = Eigen::MatrixXd::Zero(4, 3);
    Eigen::MatrixXd zeros_4_4 = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd zeros_3_4 = Eigen::MatrixXd::Zero(3, 4);

    /* Assembling the Matrix */
    m_mat__0 << ms_mat   ,   rep_jmat,    zeros_3_4,   zeros_3_4,
                rep_msmat,   jmat__0,     zeros_3_4,   zeros_3_4,
                zeros_4_3,   zeros_4_3,   mu_mat,      zeros_4_4,
                zeros_4_3,   zeros_4_3,   zeros_4_4,   jmat_y_i;

    // Force Summation
    double g{-9.81};

    // Suspended Mass
    Eigen::Vector3d F_sm1__0{ F_s1_0 + F_d1_0 + F_cp1__0 };
    Eigen::Vector3d F_sm2__0{ F_s2_0 + F_d2_0 + F_cp2__0 };
    Eigen::Vector3d F_sm3__0{ F_s3_0 + F_d3_0 + F_cp3__0 };
    Eigen::Vector3d F_sm4__0{ F_s4_0 + F_d4_0 + F_cp4__0 };
    Eigen::Vector3d F_sm_g__0{0, 0, in.m_s*g};

    // Non Suspended Mass
    double F_nsm1__0_3 = -F_s1_0(2) - F_d1_0(2) + F_kt1_0(2) + in.m_u_1*g;
    double F_nsm2__0_3 = -F_s2_0(2) - F_d2_0(2) + F_kt2_0(2) + in.m_u_2*g;
    double F_nsm3__0_3 = -F_s3_0(2) - F_d3_0(2) + F_kt3_0(2) + in.m_u_3*g;
    double F_nsm4__0_3 = -F_s4_0(2) - F_d4_0(2) + F_kt4_0(2) + in.m_u_4*g;

    // Moment Summation

    // Chassis
    Eigen::Vector3d moment_chassis = r_cp1_cm__0.cross(F_sm1__0) + r_cp2_cm__0.cross(F_sm2__0) + r_cp3_cm__0.cross(F_sm3__0) + r_cp4_cm__0.cross(F_sm4__0);

    // Gyroscopic
    Eigen::Vector3d moment_gyro = co_omega__0.cross(jmat__0*co_omega__0);

    // NSM - Wheel - Corner 1
    double moment_wheel_1__1 = -F_cp1__1(0)*r_l_1 + m_d_c/4;

    // NSM - Wheel - Corner 2
    double moment_wheel_2__2 = -F_cp2__2(0)*r_l_2 + m_d_c/4;

    // NSM - Wheel - Corner 3
    double moment_wheel_3__3 = -F_cp3__3(0)*r_l_3 + m_d_c/4;

    // NSM - Wheel - Corner 4
    double moment_wheel_4__4 = -F_cp4__4(0)*r_l_4 + m_d_c/4;

    // Force & Moment Vector Formulation
    Eigen::VectorXd f_qd_q_u = Eigen::VectorXd::Zero(14);
    f_qd_q_u << F_sm1__0 + F_sm2__0 + F_sm3__0 + F_sm4__0 + F_sm_g__0,
                moment_chassis - moment_gyro,
                F_nsm1__0_3,
                F_nsm2__0_3,
                F_nsm3__0_3,
                F_nsm4__0_3,
                moment_wheel_1__1,
                moment_wheel_2__2,
                moment_wheel_3__3,
                moment_wheel_4__4;

    Eigen::VectorXd q_d = Eigen::VectorXd::Zero(14);
    q_d << x_d,
           y_d,
           z_d,
           tait_bryant_angle_d_0,
           z_d_1,
           z_d_2,
           z_d_3,
           z_d_4,
           omega_1,
           omega_2,
           omega_3,
           omega_4;

    // First order state-vector formulation
    Eigen::VectorXd Q_dot = Eigen::VectorXd::Zero(28);
    Q_dot << q_d,
             m_mat__0.inverse()*f_qd_q_u;

    // Assembling the Output Vector
    outputs_model.push_back(u);
    outputs_model.push_back(v);

    return Q_dot;


}