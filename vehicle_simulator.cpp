//
// Created by gradhakrish on 11/1/23.
//

#include "vehicle_simulator.h"
#include "vehicle_model_fw_simplified.h"

vehicle_simulator::vehicle_simulator(vehicle_model_fw_simplified &_vehicle_model) : vehicle_model(_vehicle_model) {}

// The ::simulate function cannot be used while using boost::odeint libraries. Therefore, this code is commented out.
// The code that exists inside this function is replicated inside the overloaded operator () as needed by boost
/*Eigen::VectorXd vehicle_simulator::simulate(const double &t, Eigen::VectorXd &q)
{

    *//*
     * %vehicle_simulator Simulator function that runs vehicle simulations
        %   This is a wrapping function that is called by the numerical integrator.
        %   It knows the current time-step and using it, it interpolates all the
        %   inputs to be tracked. It also calculates the steering and throttle
        %   control action needed and it passes these as scalar values to the
        %   vehicle model.

        % INPUTS
        %   t       Time instant being simulator
        %   Q       State vector ([gen. coordinates gen. velocities]')
        %   input   Input struct

        % OUTPUTS
        % NOTE- The vectors and matrices that this function outputs are done so
        % keeping in mind future operations that this function will be used for
        % EXAMPLE - This function may need to output controller states or estimator/observer states. The O_simulator will do that
        % EXAMPLE - The vehicle model may need to output complex variables like slip angle, slip ratio etc. The O_model will do that
     *//*


    // Interpolating the steering input
    double delta_c{0};

    if (t > 4)
    {
        delta_c = 0.05;
    }

    // Initializing a driving torque input
    double m_d_c{0};

    // Initializing a vehicle model object that can solve the equations of motion
    //vehicle_model_fw_simplified vehicle_model;

    //
    Eigen::VectorXd Qdot = vehicle_model.solve(q,delta_c, m_d_c);

    return Qdot;
}*/

void vehicle_simulator::operator() ( const state_type &x , state_type &dxdt , const double  t  )
{
    // Interpolating the steering input
    double delta_c{0};

    if (t > 4)
    {
        delta_c = 0.05;
    }

    // Initializing a driving torque input
    double m_d_c{0};

    Eigen::VectorXd q = Eigen::VectorXd::Zero(28);

    for (int i = 0; i < x.size(); ++i) {
        q(i) = x[i];
    }

    Eigen::VectorXd Qdot = vehicle_model.solve(q,delta_c, m_d_c);

    for (int i=0; i < Qdot.size(); ++i)
    {
        dxdt[i] = (Qdot(i));
    }





}