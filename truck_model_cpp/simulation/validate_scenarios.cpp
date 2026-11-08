#include "truck_model/truck_model.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct Scenario {
    std::string name;
    truck_model::State initial;
    truck_model::Control control;
    double duration{};
};

void write_header(std::ofstream& output) {
    output
        << "t,x,y,psi_tractor,psi_trailer,vx,vy,r1,r2,delta,eta,"
        << "steering_rate,longitudinal_acceleration,dvx,dvy,dr1,dr2,"
        << "lateral_acceleration,alpha_front,alpha_rear,alpha_trailer,"
        << "front_fx,front_fy,rear_fx,rear_fy,trailer_fx,trailer_fy\n";
}

void write_row(std::ofstream& output, double time, const truck_model::State& state,
               const truck_model::Control& control,
               const truck_model::Diagnostics& diagnostics) {
    output << std::fixed << std::setprecision(10)
           << time << ',' << state.x << ',' << state.y << ','
           << state.psi_tractor << ',' << state.psi_trailer << ','
           << state.vx << ',' << state.vy << ','
           << state.yaw_rate_tractor << ',' << state.yaw_rate_trailer << ','
           << state.steering_angle << ',' << diagnostics.articulation_angle << ','
           << control.steering_rate << ',' << control.longitudinal_acceleration << ','
           << diagnostics.velocity_derivative[0] << ','
           << diagnostics.velocity_derivative[1] << ','
           << diagnostics.velocity_derivative[2] << ','
           << diagnostics.velocity_derivative[3] << ','
           << diagnostics.velocity_derivative[1]
                + state.vx * state.yaw_rate_tractor << ','
           << diagnostics.front_slip_angle << ','
           << diagnostics.rear_slip_angle << ','
           << diagnostics.trailer_slip_angle << ','
           << diagnostics.front_force.longitudinal << ','
           << diagnostics.front_force.lateral << ','
           << diagnostics.rear_force.longitudinal << ','
           << diagnostics.rear_force.lateral << ','
           << diagnostics.trailer_force.longitudinal << ','
           << diagnostics.trailer_force.lateral << '\n';
}

void run_scenario(const truck_model::TruckModel& model, const Scenario& scenario,
                  const std::filesystem::path& output_directory, double dt) {
    const std::size_t steps =
        static_cast<std::size_t>(std::llround(scenario.duration / dt));
    const auto path = output_directory / (scenario.name + ".csv");
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("cannot open output file: " + path.string());
    }

    write_header(output);
    truck_model::State state = scenario.initial;
    for (std::size_t step = 0; step <= steps; ++step) {
        const auto diagnostics = model.evaluate(state, scenario.control);
        write_row(output, static_cast<double>(step) * dt, state,
                  scenario.control, diagnostics);
        if (step < steps) {
            state = model.step(state, scenario.control, dt,
                               truck_model::Integrator::rk4);
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    const std::filesystem::path output_directory =
        argc > 1 ? argv[1] : "experements/csv";
    std::filesystem::create_directories(output_directory);

    truck_model::TruckModel model;
    constexpr double dt = 0.01;

    truck_model::State straight{};
    straight.vx = 15.0;

    truck_model::State steering = straight;
    // Low lateral acceleration keeps the comparison with the kinematic
    // bicycle approximation meaningful.
    steering.vx = 5.0;
    steering.steering_angle = 0.03;

    truck_model::State braking = straight;

    truck_model::State sideslip = straight;
    sideslip.vy = 1.0;
    sideslip.psi_tractor = 0.08;
    sideslip.psi_trailer = 0.0;
    sideslip.yaw_rate_tractor = 0.10;
    sideslip.yaw_rate_trailer = -0.10;

    const Scenario scenarios[] = {
        {"test13_straight", straight, {0.0, 0.0}, 10.0},
        {"test14_constant_steering", steering, {0.0, 0.0}, 20.0},
        {"test15_braking", braking, {0.0, -3.0}, 4.0},
        {"test16_initial_sideslip", sideslip, {0.0, 0.0}, 10.0},
    };

    try {
        for (const auto& scenario : scenarios) {
            run_scenario(model, scenario, output_directory, dt);
            std::cout << "wrote " << (output_directory / (scenario.name + ".csv"))
                      << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << "simulation failed: " << error.what() << '\n';
        return 1;
    }
}
