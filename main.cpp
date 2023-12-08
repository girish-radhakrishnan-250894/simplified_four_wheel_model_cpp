#include <iostream>

#include "inputs.h"
#include "vehicle_simulator.h"
#include <memory>
#include <iostream>
#include <fstream>
#include <boost/numeric/odeint.hpp>

#include <sciplot/sciplot.hpp>
using namespace sciplot;



//[ integrate_observer
struct push_back_state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
            : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
    }
};
//]

int main() {
    using namespace std;
    using namespace boost::numeric::odeint;

    // Initializing input struc
    shared_ptr<inputs> input_struct = make_shared<inputs>();

    // Initializing steering input
    input_struct->delta = {0 , 0 , 0 , 0 , 0.05 , 0.05 , 0 , 0 , -0.05 , -0.05 , 0, 0} ;
    // Initializing time input
    input_struct->delta = {0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 };


    // Initializing initial conditions
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(28);
    q0(14) = 50/3.6;
    q0(24) = q0(14)/(input_struct)->r_01;
    q0(25) = q0(14)/(input_struct)->r_02;
    q0(26) = q0(14)/(input_struct)->r_03;
    q0(27) = q0(14)/(input_struct)->r_04;


    // Initializing vehicle simulator
    shared_ptr<vehicle_model_fw_simplified> vehicle_model = make_shared<vehicle_model_fw_simplified>((*input_struct));
    shared_ptr<vehicle_simulator> simulate_vehicle = make_shared<vehicle_simulator>(*vehicle_model);


    double u_start{50/3.5};

    state_type x(28);
    x[14] = u_start;
    x[24] = u_start/(input_struct)->r_01;
    x[25] = u_start/(input_struct)->r_02;
    x[27] = u_start/(input_struct)->r_04;
    x[26] = u_start/(input_struct)->r_03;

    //[ integrate_observ
    vector<state_type> x_vec;
    vector<double> times;


    ofstream fout("sim.csv");
    if(!fout)
    {
        cout<<"\n error"<<endl;
    }

    size_t steps = integrate((*simulate_vehicle) ,
                             x , 0.0 , 5.0 , 0.01,
                             push_back_state_and_time(x_vec, times));

    for (int i = 0; i < x_vec.size(); ++i) {
        for (int j = 0; j < x.size(); ++j) {
            fout << x_vec[i][j] << " , ";
        }
        fout <<"\n";
    }

    std::cout << x[14] << "\n" << endl;

    //////////////////////////////////////////////////////////////
    // SCI PLOT SAMPLE
    //////////////////////////////////////////////////////////////
    // Create a vector with values from 0 to pi divived into 200 uniform intervals for the x-axis
    Vec xs = linspace(0.0, PI, 200);

    // Create a Plot object
    Plot2D plot;

    // Set the x and y labels
    plot.xlabel("x");
    plot.ylabel("y");

    // Set the x and y ranges
    plot.xrange(0.0, PI);
    plot.yrange(0.0, 1.0);

    // Set the legend to be on the bottom along the horizontal
    plot.legend()
            .atOutsideBottom()
            .displayHorizontal()
            .displayExpandWidthBy(2);

    // Plot sin(i*x) from i = 1 to i = 6
    plot.drawCurve(xs, std::sin(1.0 * xs)).label("sin(x)");
    plot.drawCurve(xs, std::sin(2.0 * xs)).label("sin(2x)");
    plot.drawCurve(xs, std::sin(3.0 * xs)).label("sin(3x)");
    plot.drawCurve(xs, std::sin(4.0 * xs)).label("sin(4x)");
    plot.drawCurve(xs, std::sin(5.0 * xs)).label("sin(5x)");
    plot.drawCurve(xs, std::sin(6.0 * xs)).label("sin(6x)");

    // Create figure to hold plot
    Figure fig = {{plot}};
    // Create canvas to hold figure
    Canvas canvas = {{fig}};

    // Show the plot in a pop-up window
    canvas.show();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
