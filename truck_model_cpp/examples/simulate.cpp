#include "truck_model/truck_model.hpp"
#include "truck_model/mppi_controller.hpp"

#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    truck_model::TruckModel model;

    // Controller settings belong to the application, not to the vehicle model.
    truck_model::MppiConfig mppi_config;
    mppi_config.dt = 0.02;
    mppi_config.horizon = 50;  // 1 second = horizon * dt
    mppi_config.samples = 1000;
    mppi_config.temperature = 10.0;
    mppi_config.steering_rate_noise_std = 0.10;
    mppi_config.acceleration_noise_std = 0.50;
    mppi_config.random_seed = 42;  // repeatable simulation

    truck_model::MppiCostWeights cost_weights;
    cost_weights.trailer_slip_angle = 100.0;
    cost_weights.tractor_yaw_rate = 10.0;
    cost_weights.yaw_rate_difference = 20.0;
    cost_weights.articulation_angle = 200.0;
    cost_weights.control_effort = 0.01;
    cost_weights.terminal_multiplier = 5.0;

    truck_model::MppiController controller(model, mppi_config, cost_weights);

    truck_model::State initial{};
    initial.vx = 15.0;
    initial.vy = 1.0;
    initial.psi_trailer = -0.08;
    initial.yaw_rate_tractor = 0.10;
    initial.yaw_rate_trailer = -0.10;

    constexpr std::size_t steps = 250;
    const double dt = mppi_config.dt;
    std::vector<truck_model::Control> controls(steps);
    std::vector<truck_model::State> trajectory(steps + 1);
    trajectory[0] = initial;
    for (std::size_t i = 0; i < steps; ++i) {
        controls[i] = controller.compute(trajectory[i]);
        trajectory[i + 1] = model.step(trajectory[i], controls[i], dt);
    }

    std::cout
    << "t,x,y,psi_tractor,psi_trailer,vx,vy,r1,r2,delta,"
    << "steering_rate,longitudinal_acceleration\n";
    for (std::size_t i = 0; i < trajectory.size(); i += 10) {
        const auto values = trajectory[i].as_array();
        std::cout << std::fixed << std::setprecision(6) << i * dt;
        for (const double value : values) std::cout << ',' << value;
        if (i < controls.size()) {
            std::cout
                << ',' << controls[i].steering_rate
                << ',' << controls[i].longitudinal_acceleration;
        } else {
            std::cout << ",0,0";
        }
        std::cout << '\n';
    }
}

