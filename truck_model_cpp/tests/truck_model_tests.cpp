#include "truck_model/truck_model.hpp"
#include "controller/mppi_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void require_near(double actual, double expected, double tolerance, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::cerr << message << ": actual=" << actual << " expected=" << expected << '\n';
        throw std::runtime_error(message);
    }
}

void test_straight_coast() {
    truck_model::TruckModel model;
    truck_model::State state{};
    state.vx = 12.0;
    const auto next = model.step(state, {}, 0.1);
    require_near(next.x, 1.2, 1e-10, "straight-line x");
    require_near(next.y, 0.0, 1e-10, "straight-line y");
    require_near(next.vx, 12.0, 1e-10, "coasting vx");
    require_near(next.vy, 0.0, 1e-10, "coasting vy");
}

void test_steering_and_control_limits() {
    truck_model::TruckModel model;
    truck_model::State state{};
    state.vx = 10.0;
    truck_model::Control control{100.0, 100.0};
    const auto derivative = model.derivative(state, control);
    require_near(derivative.steering_angle, model.parameters().max_steering_rate, 1e-12,
                 "steering-rate clamp");
    for (int i = 0; i < 20; ++i) state = model.step(state, control, 0.05);
    require_near(state.steering_angle, model.parameters().max_steering_angle, 1e-12,
                 "steering-angle clamp");
}

void test_friction_circles() {
    truck_model::TruckModel model;
    truck_model::State state{};
    state.vx = 20.0;
    state.vy = 15.0;
    state.steering_angle = 0.5;
    const auto d = model.evaluate(state, {0.0, -8.0});
    const auto& p = model.parameters();
    require(std::hypot(d.front_force.longitudinal, d.front_force.lateral) <=
                p.friction_coefficient * d.front_normal_load + 1e-8,
            "front friction circle");
    require(std::hypot(d.rear_force.longitudinal, d.rear_force.lateral) <=
                p.friction_coefficient * d.rear_normal_load + 1e-8,
            "rear friction circle");
    require(std::hypot(d.trailer_force.longitudinal, d.trailer_force.lateral) <=
                p.friction_coefficient * d.trailer_normal_load + 1e-8,
            "trailer friction circle");
}

void test_mppi_returns_limited_finite_control() {
    truck_model::MppiConfig config;
    config.horizon = 8;
    config.samples = 32;
    config.dt = 0.02;
    config.random_seed = 123;
    truck_model::MppiController controller(truck_model::TruckModel{}, config);

    truck_model::State state{};
    state.vx = 15.0;
    state.vy = 1.0;
    state.psi_trailer = -0.08;
    const auto control = controller.compute(state);
    const auto& p = truck_model::Parameters{};
    require(std::isfinite(control.steering_rate) &&
                std::isfinite(control.longitudinal_acceleration),
            "MPPI control must be finite");
    require(std::abs(control.steering_rate) <= p.max_steering_rate + 1e-12,
            "MPPI steering rate must be limited");
    require(control.longitudinal_acceleration >= -p.max_braking_deceleration - 1e-12 &&
                control.longitudinal_acceleration <= p.max_acceleration + 1e-12,
            "MPPI acceleration must be limited");
    require(controller.nominal_controls().size() == config.horizon,
            "MPPI warm-start plan must keep its horizon");
}

void test_rollout_matches_repeated_step() {
    truck_model::TruckModel model;
    truck_model::State initial{};
    initial.vx = 8.0;
    std::vector<truck_model::Control> controls(20, {0.03, 0.1});
    std::vector<truck_model::State> states(controls.size() + 1);
    model.rollout(initial, controls.data(), controls.size(), 0.02, states.data());
    auto manual = initial;
    for (const auto& control : controls) manual = model.step(manual, control, 0.02);
    const auto a = manual.as_array();
    const auto b = states.back().as_array();
    for (std::size_t i = 0; i < a.size(); ++i) {
        require_near(a[i], b[i], 1e-12, "rollout equivalence");
    }
}

void test_invalid_dt() {
    truck_model::TruckModel model;
    bool threw = false;
    try { static_cast<void>(model.step({}, {}, 0.0)); }
    catch (const std::invalid_argument&) { threw = true; }
    require(threw, "zero dt must fail");
}

void test_nominal_rollout_stays_finite() {
    truck_model::TruckModel model;
    truck_model::State state{};
    state.vx = 15.0;
    for (int i = 0; i < 500; ++i) {
        const double steering_rate = i < 100 ? 0.15 : (i < 200 ? -0.15 : 0.0);
        state = model.step(state, {steering_rate, 0.0}, 0.01);
        for (const double value : state.as_array()) {
            require(std::isfinite(value), "nominal rollout must stay finite");
        }
    }
}

} // namespace

int main() {
    try {
        test_straight_coast();
        test_steering_and_control_limits();
        test_friction_circles();
        test_mppi_returns_limited_finite_control();
        test_rollout_matches_repeated_step();
        test_invalid_dt();
        test_nominal_rollout_stays_finite();
        std::cout << "All truck model tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
