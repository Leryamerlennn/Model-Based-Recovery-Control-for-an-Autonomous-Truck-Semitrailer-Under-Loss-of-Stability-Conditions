#pragma once

#include <array>
#include <cstddef>

namespace truck_model {

constexpr std::size_t kStateDimension = 9;
constexpr std::size_t kControlDimension = 2;

// SI units: m, s, kg, rad, N.
struct State {
    double x{};
    double y{};
    double psi_tractor{};
    double psi_trailer{};
    double vx{};
    double vy{};
    double yaw_rate_tractor{};
    double yaw_rate_trailer{};
    double steering_angle{};

    [[nodiscard]] std::array<double, kStateDimension> as_array() const noexcept;
    [[nodiscard]] static State from_array(
        const std::array<double, kStateDimension>& values) noexcept;
};

struct Control {
    double steering_rate{};               // rad/s
    double longitudinal_acceleration{};   // m/s^2; positive = drive, negative = brake
};

enum class Integrator { euler, rk4 };

struct Parameters {
    // Tractor/trailer rigid-body properties.
    double tractor_mass{8000.0};
    double trailer_mass{20000.0};
    double tractor_yaw_inertia{40000.0};
    double trailer_yaw_inertia{250000.0};

    // Geometry from the report.
    double l1{1.5};   // tractor CoM -> front axle
    double l2{1.9};   // tractor CoM -> hitch
    double l3{2.2};   // tractor CoM -> rear axle
    double b1{5.0};   // hitch -> trailer CoM
    double b3{3.0};   // trailer CoM -> trailer axle

    // Сa per axle
    double front_cornering_stiffness{230000.0};
    double rear_cornering_stiffness{500000.0};
    double trailer_cornering_stiffness{1650000.0};

    double friction_coefficient{0.4};
    double gravity{9.81};

    // Actuator/numerical limits.
    double max_steering_angle{0.6};
    double max_steering_rate{0.8};
    double max_acceleration{3.0};
    double max_braking_deceleration{8.0};
    double minimum_slip_speed{0.5};
};

struct AxleForce {
    double longitudinal{};
    double lateral{};
};

struct Diagnostics {
    double articulation_angle{};
    double front_slip_angle{};
    double rear_slip_angle{};
    double trailer_slip_angle{};

    double front_normal_load{};
    double rear_normal_load{};
    double trailer_normal_load{};

    AxleForce front_force{};
    AxleForce rear_force{};
    AxleForce trailer_force{};
    std::array<double, 4> generalized_force{};
    std::array<double, 4> velocity_derivative{}; // dvx, dvy, dr1, dr2
};

class TruckModel {
public:
    explicit TruckModel(Parameters parameters = {});

    [[nodiscard]] const Parameters& parameters() const noexcept { return parameters_; }
    [[nodiscard]] State derivative(const State& state, const Control& control) const;
    [[nodiscard]] State step(const State& state, const Control& control, double dt,
                             Integrator integrator = Integrator::rk4) const;
    [[nodiscard]] Diagnostics evaluate(const State& state, const Control& control) const;

    // Allocation-free horizon rollout for MPPI. out_states must hold count + 1
    // elements; element zero receives initial_state.
    void rollout(const State& initial_state, const Control* controls,
                 std::size_t count, double dt, State* out_states,
                 Integrator integrator = Integrator::rk4) const;

private:
    Parameters parameters_;

    [[nodiscard]] Control clamp_control(const Control& control) const noexcept;
    [[nodiscard]] State project_state(State state) const noexcept;
};

} // namespace truck_model

