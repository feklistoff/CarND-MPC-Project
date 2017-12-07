#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
int N = 10;
double dt = 0.15; // in sec

// This value assumes the model presented in the classroom is used
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius
//
// This is the length from front to CoG that has a similar radius
const double Lf = 2.67;

// Reference values
const double v_ref = 45; // 45 m/s == 100.66 mph

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier
const int x_start     = 0;
const int y_start     = x_start + N;
const int psi_start   = y_start + N;
const int v_start     = psi_start + N;
const int cte_start   = v_start + N;
const int epsi_start  = cte_start + N;
const int delta_start = epsi_start + N;
const int a_start     = delta_start + N - 1;

// Weights for each part of cost function
const double Wcte          = 20;
const double Wepsi         = 6000;
const double Wv            = 1;
const double Wdelta        = 1500;
const double Wa            = 20;
const double Wdelta_change = 2000;
const double Wa_change     = 20;

class FG_eval
{
    public:
        // Fitted polynomial coefficients
        Eigen::VectorXd coeffs;
        FG_eval(Eigen::VectorXd coeffs) {this->coeffs = coeffs;}

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        void operator()(ADvector &fg, const ADvector &vars)
        {
            // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
            
            // The cost is stored is the first element of `fg`
            // Any additions to the cost should be added to `fg[0]`
            fg[0] = 0;

            // Cost: `cte`, `epsi` and `v`
            for (int t = 0; t < N; t++)
            {
                fg[0] += Wcte * CppAD::pow(vars[cte_start + t], 2);
                fg[0] += Wepsi * CppAD::pow(vars[epsi_start + t], 2);
                fg[0] += Wv * CppAD::pow(vars[v_start + t] - v_ref, 2);
            }
            
            // Cost: `delta` and `a`
            for (int t = 0; t < N - 1; t++)
            {
                fg[0] += Wdelta * CppAD::pow(vars[delta_start + t], 2);
                fg[0] += Wa * CppAD::pow(vars[a_start + t], 2);
            }

            // Cost: the change in `delta` and `a` (to make control decisions more smoother)
            for (int t = 0; t < N - 2; t++)
            {
                fg[0] += Wdelta_change * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
                fg[0] += Wa_change * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
            }

            // Setup Constraints
            
            // Initial constraints
            fg[1 + x_start]    = vars[x_start];
            fg[1 + y_start]    = vars[y_start];
            fg[1 + psi_start]  = vars[psi_start];
            fg[1 + v_start]    = vars[v_start];
            fg[1 + cte_start]  = vars[cte_start];
            fg[1 + epsi_start] = vars[epsi_start];

            // The rest of the constraints
            for (int t = 1; t < N; t++) 
            {
                // The state at time t+1
                AD<double> x1    = vars[x_start + t];
                AD<double> y1    = vars[y_start + t];
                AD<double> psi1  = vars[psi_start + t];
                AD<double> v1    = vars[v_start + t];
                AD<double> cte1  = vars[cte_start + t];
                AD<double> epsi1 = vars[epsi_start + t];

                // The state at time t
                AD<double> x0    = vars[x_start + t - 1];
                AD<double> y0    = vars[y_start + t - 1];
                AD<double> psi0  = vars[psi_start + t - 1];
                AD<double> v0    = vars[v_start + t - 1];
                AD<double> cte0  = vars[cte_start + t - 1];
                AD<double> epsi0 = vars[epsi_start + t - 1];

                // Only consider the actuation at time t
                AD<double> delta0 = vars[delta_start + t - 1];
                AD<double> a0     = vars[a_start + t - 1];

                AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
                AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

                // The idea here is to constraint this value to be 0.
                //
                // Recall the equations for the model:
                // x_[t+1]   = x[t] + v[t] * cos(psi[t]) * dt
                // y_[t+1]   = y[t] + v[t] * sin(psi[t]) * dt
                // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
                // v_[t+1]   = v[t] + a[t] * dt
                // cte[t+1]  = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
                // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
                fg[1 + x_start + t]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
                fg[1 + y_start + t]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
                fg[1 + psi_start + t]  = psi1 - (psi0 + v0 / Lf * delta0 * dt);
                fg[1 + v_start + t]    = v1 - (v0 + a0 * dt);
                fg[1 + cte_start + t]  = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
                fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
            }
        }
};

//
// MPC class definition implementation
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // Number of model variables (includes both states and actuators)
    int n_vars = N * 6 + (N - 1) * 2; // 6 - size of state vector; 2 - number of actuators
    // Number of constraints
    int n_constraints = N * 6;

    // Initial state
    double x    = state[0];
    double y    = state[1];
    double psi  = state[2];
    double v    = state[3];
    double cte  = state[4];
    double epsi = state[5];

    // Initial value of the independent variable
    // SHOULD BE 0 besides initial state
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start]    = x;
    vars[y_start]    = y;
    vars[psi_start]  = psi;
    vars[v_start]    = v;
    vars[cte_start]  = cte;
    vars[epsi_start] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set lower and upper limits for variables
    for (int i = 0; i < delta_start; i++)
    {
        // some big numbers as we don't want to constrarin non-actuator variables
        vars_lowerbound[i] = -100000000;
        vars_upperbound[i] =  100000000;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians) 25 deg == 0.392699 rad
    for (int i = delta_start; i < a_start; i++)
    {
        vars_lowerbound[i] = -0.392699;
        vars_upperbound[i] =  0.392699;
    }

    // Acceleration/decceleration upper and lower limits
    for (int i = a_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] =  1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // Set the initial state constraints
    constraints_lowerbound[x_start]    = x;
    constraints_lowerbound[y_start]    = y;
    constraints_lowerbound[psi_start]  = psi;
    constraints_lowerbound[v_start]    = v;
    constraints_lowerbound[cte_start]  = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start]    = x;
    constraints_upperbound[y_start]    = y;
    constraints_upperbound[psi_start]  = psi;
    constraints_upperbound[v_start]    = v;
    constraints_upperbound[cte_start]  = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // Place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // Solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Return the first actuator values
    // the variables can be accessed with `solution.x[i]`
    std::vector<double> result;

    // Set first two elements of output vector as `delta` and `a`
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);

    // Set other elements of vector as x,y pairs for the visualization
    for (int i = 0; i < N - 1; i++)
    {
        result.push_back(solution.x[x_start + 1 + i]);
        result.push_back(solution.x[y_start + 1 + i]);        
    }

    return result;
}
