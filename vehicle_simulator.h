//
// Created by gradhakrish on 11/1/23.
//

#ifndef SIMPLIFIED_FOUR_WHEEL_MODEL_VEHICLE_SIMULATOR_H
#define SIMPLIFIED_FOUR_WHEEL_MODEL_VEHICLE_SIMULATOR_H

#include <eigen3/Eigen/Dense>
#include "inputs.h"

#include "vehicle_model_fw_simplified.h"

typedef std::vector<double> state_type;

class vehicle_simulator {

    vehicle_model_fw_simplified vehicle_model;
    Eigen::VectorXd outputs_simulator;

public:

    explicit vehicle_simulator(vehicle_model_fw_simplified &_vehicle_model);

/*    virtual Eigen::VectorXd simulate(const double &t, Eigen::VectorXd &q);*/

    void operator() ( const state_type &x , state_type &dxdt , const double t  );

    virtual Eigen::VectorXd get_outputs_simulator() {return outputs_simulator;}
};


#endif //SIMPLIFIED_FOUR_WHEEL_MODEL_VEHICLE_SIMULATOR_H
