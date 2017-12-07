#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// For convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned
string hasData(string s) 
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main() 
{
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
        {
            string s = hasData(sdata);
            if (s != "") 
            {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px    = j[1]["x"];               // given in meters
                    double py    = j[1]["y"];               // given in meters
                    double psi   = j[1]["psi"];             // given in radians
                    double v     = j[1]["speed"];           // given in mph
                    double delta = j[1]["steering_angle"];  // given in radians
                    double a     = j[1]["throttle"];

                    v *= 0.44704; // convert speed from mph to m/s
                    delta *= -1;  // we need to flip steereing angle
                    
                    // Convert waypoints x,y from map coordinates to vehicle coordinates
                    // we need Eigen::VectorXd for later use in polyfit()
                    Eigen::VectorXd wpts_x(ptsx.size());
                    Eigen::VectorXd wpts_y(ptsy.size());
                    for (size_t i = 0; i < ptsx.size(); i++)
                    {
                        // find offset for waypoints (shift)
                        double x = ptsx[i] - px;
                        double y = ptsy[i] - py;
                        // rotate, mind the psi
                        wpts_x[i] = x * cos(-psi) - y * sin(-psi);
                        wpts_y[i] = x * sin(-psi) + y * cos(-psi);
                    }

                    // Fit a 3rd-order polynomial to waypoints' x,y that represents the desired trajectory
                    auto coeffs = polyfit(wpts_x, wpts_y, 3);

                    // Account for the 100ms delay
                    double dt = 0.1; // in sec

                    // Initial state 
                    // NOTE: in vehicle coordinates x, y and psi always zero
                    double x0    = 0;
                    double y0    = 0;
                    double psi0  = 0;
                    double v0    = v;
                    double cte0  = polyeval(coeffs, 0); // find offset at (0,0)
                    // NOTE: epsi0 = psi - arctan(coeffs[1] + 2*x*coeffs[2] + 3*x^2*coeffs[3]) when x=0, psi0 -> -arctan(coeffs[1])
                    double epsi0 = -atan(coeffs[1]); 
                    
                    // Future state in 100ms
                    double x1 = x0 + v0 * cos(psi0) * dt;
                    double y1 = y0 + v0 * sin(psi0) * dt;
                    const double Lf = 2.67; // this is the length from front to CoG that has a similar radius
                    double psi1  = psi0 + v0 * delta / Lf * dt;
                    double v1    = v0 + a * dt;
                    double cte1  = cte0 + v0 * sin(epsi0) * dt;
                    double epsi1 = epsi0 + v0 * delta / Lf * dt;

                    Eigen::VectorXd state(6);
                    state << x1, y1, psi1, v1, cte1, epsi1;

                    /*
                    * Calculate steering angle and throttle using MPC
                    * Both are in between [-1, 1]
                    */
                    auto result = mpc.Solve(state, coeffs);

                    double steer_value = -1 * result[0]; // flip angle back
                    double throttle_value = result[1];

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before sending the steering value back
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1]
                    msgJson["steering_angle"] = steer_value / deg2rad(25);
                    msgJson["throttle"] = throttle_value;

                    //Display the MPC predicted trajectory 
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    for (size_t i = 2; i < result.size(); i++)
                    {
                        if (i % 2 == 0)
                            mpc_x_vals.push_back(result[i]);
                        else
                            mpc_y_vals.push_back(result[i]);
                    }

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    for (int i = 0; i < wpts_x.size(); i++)
                    {
                        next_x_vals.push_back(wpts_x[i]);
                        next_y_vals.push_back(wpts_y[i]);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";

                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) 
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        } 
        else
        {
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    } 
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
