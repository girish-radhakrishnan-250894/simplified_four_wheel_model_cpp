//
// Created by gradhakrish on 11/1/23.
//

#ifndef SIMPLIFIED_FOUR_WHEEL_MODEL_VEHICLE_MODEL_FW_SIMPLIFIED_H
#define SIMPLIFIED_FOUR_WHEEL_MODEL_VEHICLE_MODEL_FW_SIMPLIFIED_H

#include <eigen3/Eigen/Dense>
#include "inputs.h"
#include <memory>

class vehicle_model_fw_simplified {

    std::vector<double> outputs_model;
    inputs model_input;

public:

    explicit vehicle_model_fw_simplified(const inputs &_model_input) : model_input(_model_input) {}

    virtual Eigen::VectorXd solve(Eigen::VectorXd q, const double &delta_c, const double &m_d_c);

    virtual inputs get_model_input() {return model_input;}

    virtual std::vector<double> get_outputs_model() { return outputs_model;}

};


#endif //SIMPLIFIED_FOUR_WHEEL_MODEL_VEHICLE_MODEL_FW_SIMPLIFIED_H
