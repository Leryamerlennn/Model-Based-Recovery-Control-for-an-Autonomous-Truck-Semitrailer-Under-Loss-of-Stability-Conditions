#include "truck_model/truck_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace truck_model {
namespace {

using Vector4 = std::array<double, 4>;
using Matrix4 = std::array<std::array<double, 4>, 4>;

double clamp(double value, double low, double high) noexcept {
    return std::max(low, std::min(value, high));
}

double safe_longitudinal_speed(double speed, double minimum_magnitude) noexcept {
    if (std::abs(speed) >= minimum_magnitude) {
        return speed;
    }
    return std::copysign(minimum_magnitude, speed == 0.0 ? 1.0 : speed);
}

AxleForce saturate_friction_circle(double fx, double fy, double normal_load,
                                   double friction_coefficient) noexcept {
    const double limit = friction_coefficient * normal_load;
    const double magnitude = std::hypot(fx, fy);
    if (magnitude <= limit || magnitude == 0.0) {
        return {fx, fy};
    }
    const double scale = limit / magnitude;
    return {scale * fx, scale * fy};
}

Vector4 solve_linear_system(Matrix4 matrix, Vector4 rhs) {
    // Gaussian elimination with partial pivoting. A fixed-size solver avoids
    // heap allocation in the inner MPPI rollout loop.
    for (std::size_t column = 0; column < 4; ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < 4; ++row) {
            if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
                pivot = row;
            }
        }
        if (std::abs(matrix[pivot][column]) < 1e-12) {
            throw std::runtime_error("truck inertia matrix is singular");
        }
        if (pivot != column) {
            std::swap(matrix[pivot], matrix[column]);
            std::swap(rhs[pivot], rhs[column]);
        }
        for (std::size_t row = column + 1; row < 4; ++row) {
            const double factor = matrix[row][column] / matrix[column][column];
            for (std::size_t j = column; j < 4; ++j) {
                matrix[row][j] -= factor * matrix[column][j];
            }
            rhs[row] -= factor * rhs[column];
        }
    }

    Vector4 solution{};
    for (int row = 3; row >= 0; --row) {
        double value = rhs[static_cast<std::size_t>(row)];
        for (std::size_t column = static_cast<std::size_t>(row) + 1; column < 4;
             ++column) {
            value -= matrix[static_cast<std::size_t>(row)][column] * solution[column];
        }
        solution[static_cast<std::size_t>(row)] =
            value / matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)];
    }
    return solution;
}

State add_scaled(const State& state, const State& derivative, double scale) noexcept {
    const auto a = state.as_array();
    const auto b = derivative.as_array();
    std::array<double, kStateDimension> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        result[i] = a[i] + scale * b[i];
    }
    return State::from_array(result);
}

State rk4_sum(const State& state, const State& k1, const State& k2,
              const State& k3, const State& k4, double dt) noexcept {
    const auto x = state.as_array();
    const auto a = k1.as_array();
    const auto b = k2.as_array();
    const auto c = k3.as_array();
    const auto d = k4.as_array();
    std::array<double, kStateDimension> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        result[i] = x[i] + (dt / 6.0) * (a[i] + 2.0 * b[i] + 2.0 * c[i] + d[i]);
    }
    return State::from_array(result);
}

void validate_parameters(const Parameters& p) {
    const bool positive = p.tractor_mass > 0.0 && p.trailer_mass > 0.0 &&
        p.tractor_yaw_inertia > 0.0 && p.trailer_yaw_inertia > 0.0 &&
        p.l1 > 0.0 && p.l2 > 0.0 && p.l3 > 0.0 && p.b1 > 0.0 && p.b3 > 0.0 &&
        p.front_cornering_stiffness > 0.0 && p.rear_cornering_stiffness > 0.0 &&
        p.trailer_cornering_stiffness > 0.0 && p.gravity > 0.0 && 
        p.friction_coefficient > 0.0 && p.max_steering_angle > 0.0 &&
        p.max_steering_rate > 0.0 && p.max_acceleration > 0.0 &&
        p.max_braking_deceleration > 0.0 && p.minimum_slip_speed > 0.0;
    if (!positive) {
        throw std::invalid_argument("truck model parameters must be positive");
    }
}

} // namespace

std::array<double, kStateDimension> State::as_array() const noexcept {
    return {x, y, psi_tractor, psi_trailer, vx, vy, yaw_rate_tractor,
            yaw_rate_trailer, steering_angle};
}

State State::from_array(const std::array<double, kStateDimension>& v) noexcept {
    return {v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]};
}

TruckModel::TruckModel(Parameters parameters) : parameters_(std::move(parameters)) {
    validate_parameters(parameters_);
}

Control TruckModel::clamp_control(const Control& control) const noexcept {
    return {
        clamp(control.steering_rate, -parameters_.max_steering_rate,
              parameters_.max_steering_rate),
        clamp(control.longitudinal_acceleration,
              -parameters_.max_braking_deceleration, parameters_.max_acceleration)
    };
}

State TruckModel::project_state(State state) const noexcept {
    state.steering_angle = clamp(state.steering_angle, -parameters_.max_steering_angle,
                                 parameters_.max_steering_angle);
    return state;
}

Diagnostics TruckModel::evaluate(const State& raw_state, const Control& raw_control) const {
    const State state = project_state(raw_state);
    const Control control = clamp_control(raw_control);
    const auto& p = parameters_;
    // Static normal loads.
    const double trailer_normal_load =
        p.trailer_mass * p.gravity *
        p.b1 / (p.b1 + p.b3);

    const double hitch_normal_load =
        p.trailer_mass * p.gravity *
        p.b3 / (p.b1 + p.b3);

    const double front_normal_load =
        (p.tractor_mass * p.gravity * p.l3 +
        hitch_normal_load * (p.l3 - p.l2)) /
        (p.l1 + p.l3);

    const double rear_normal_load =
        (p.tractor_mass * p.gravity * p.l1 +
        hitch_normal_load * (p.l1 + p.l2)) /
        (p.l1 + p.l3);

    const double eta = state.psi_tractor - state.psi_trailer;
    const double sin_eta = std::sin(eta);
    const double cos_eta = std::cos(eta);
    const double trailer_length = p.b1 + p.b3;

    const double front_vy = state.vy + p.l1 * state.yaw_rate_tractor;
    const double rear_vy = state.vy - p.l3 * state.yaw_rate_tractor;
    const double alpha_front = std::atan2(
        front_vy, safe_longitudinal_speed(state.vx, p.minimum_slip_speed)) -
        state.steering_angle;
    const double alpha_rear = std::atan2(
        rear_vy, safe_longitudinal_speed(state.vx, p.minimum_slip_speed));

    const double trailer_vx_tractor =
        state.vx - trailer_length * state.yaw_rate_trailer * sin_eta;
    const double trailer_vy_tractor = state.vy - p.l2 * state.yaw_rate_tractor -
        trailer_length * state.yaw_rate_trailer * cos_eta;
    const double trailer_vx = cos_eta * trailer_vx_tractor - sin_eta * trailer_vy_tractor;
    const double trailer_vy = sin_eta * trailer_vx_tractor + cos_eta * trailer_vy_tractor;
    const double alpha_trailer = std::atan2(
        trailer_vy, safe_longitudinal_speed(trailer_vx, p.minimum_slip_speed));

    const double total_command =
        (p.tractor_mass + p.trailer_mass) * control.longitudinal_acceleration;
    double front_fx_command = 0.0;
    double rear_fx_command = total_command;
    double trailer_fx_command = 0.0;
    if (total_command < 0.0) {
        const double total_normal =
            front_normal_load +
            rear_normal_load +
            trailer_normal_load;

        front_fx_command =
            total_command * front_normal_load / total_normal;

        rear_fx_command =
            total_command * rear_normal_load / total_normal;

        trailer_fx_command =
            total_command * trailer_normal_load / total_normal;
    }

    const double front_fy_nominal =
        -p.front_cornering_stiffness * alpha_front;

    const double rear_fy_nominal =
        -p.rear_cornering_stiffness * alpha_rear;

    const double trailer_fy_nominal =
        -p.trailer_cornering_stiffness * alpha_trailer;

    const AxleForce front = saturate_friction_circle(
        front_fx_command, front_fy_nominal, front_normal_load, p.friction_coefficient);
    const AxleForce rear = saturate_friction_circle(
        rear_fx_command, rear_fy_nominal, rear_normal_load, p.friction_coefficient);
    const AxleForce trailer = saturate_friction_circle(
        trailer_fx_command, trailer_fy_nominal, trailer_normal_load,
        p.friction_coefficient);

    const double cos_delta = std::cos(state.steering_angle);
    const double sin_delta = std::sin(state.steering_angle);
    const double front_x = front.longitudinal * cos_delta - front.lateral * sin_delta;
    const double front_y = front.longitudinal * sin_delta + front.lateral * cos_delta;
    const double trailer_x = trailer.longitudinal * cos_eta + trailer.lateral * sin_eta;
    const double trailer_y = -trailer.longitudinal * sin_eta + trailer.lateral * cos_eta;

    Vector4 generalized_force{
        front_x + rear.longitudinal + trailer_x,
        front_y + rear.lateral + trailer_y,
        p.l1 * front_y - p.l3 * rear.lateral - p.l2 * trailer_y,
        -trailer_length * trailer.lateral
    };

    const double m2 = p.trailer_mass;
    const double r1 = state.yaw_rate_tractor;
    const double r2 = state.yaw_rate_trailer;
    Matrix4 inertia{{
        {{p.tractor_mass + m2, 0.0, 0.0, -m2 * p.b1 * sin_eta}},
        {{0.0, p.tractor_mass + m2, -m2 * p.l2, -m2 * p.b1 * cos_eta}},
        {{0.0, -m2 * p.l2, p.tractor_yaw_inertia + m2 * p.l2 * p.l2,
          m2 * p.l2 * p.b1 * cos_eta}},
        {{-m2 * p.b1 * sin_eta, -m2 * p.b1 * cos_eta,
          m2 * p.l2 * p.b1 * cos_eta, p.trailer_yaw_inertia + m2 * p.b1 * p.b1}}
    }};
    const Vector4 velocity_terms{
        m2 * p.l2 * r1 * r1 + m2 * p.b1 * r2 * r2 * cos_eta -
            (p.tractor_mass + m2) * r1 * state.vy,
        (p.tractor_mass + m2) * r1 * state.vx - m2 * p.b1 * r2 * r2 * sin_eta,
        -m2 * p.l2 * (r1 * state.vx - p.b1 * r2 * r2 * sin_eta),
        -m2 * p.b1 * r1 *
            (state.vx * cos_eta + (p.l2 * r1 - state.vy) * sin_eta)
    };
    Vector4 rhs{};
    for (std::size_t i = 0; i < 4; ++i) {
        rhs[i] = generalized_force[i] - velocity_terms[i];
    }
    const Vector4 velocity_derivative = solve_linear_system(inertia, rhs);

    return {eta, alpha_front, alpha_rear, alpha_trailer, front_normal_load, rear_normal_load, trailer_normal_load, front, rear, trailer,
            generalized_force, velocity_derivative};
}

State TruckModel::derivative(const State& state, const Control& raw_control) const {
    const Control control = clamp_control(raw_control);
    const Diagnostics diagnostics = evaluate(state, control);
    double steering_rate = control.steering_rate;
    if ((state.steering_angle >= parameters_.max_steering_angle && steering_rate > 0.0) ||
        (state.steering_angle <= -parameters_.max_steering_angle && steering_rate < 0.0)) {
        steering_rate = 0.0;
    }
    const double cos_psi = std::cos(state.psi_tractor);
    const double sin_psi = std::sin(state.psi_tractor);
    return {
        state.vx * cos_psi - state.vy * sin_psi,
        state.vx * sin_psi + state.vy * cos_psi,
        state.yaw_rate_tractor,
        state.yaw_rate_trailer,
        diagnostics.velocity_derivative[0],
        diagnostics.velocity_derivative[1],
        diagnostics.velocity_derivative[2],
        diagnostics.velocity_derivative[3],
        steering_rate
    };
}

State TruckModel::step(const State& state, const Control& control, double dt,
                       Integrator integrator) const {
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        throw std::invalid_argument("dt must be finite and positive");
    }
    if (integrator == Integrator::euler) {
        return project_state(add_scaled(state, derivative(state, control), dt));
    }
    const State k1 = derivative(state, control);
    const State k2 = derivative(project_state(add_scaled(state, k1, 0.5 * dt)), control);
    const State k3 = derivative(project_state(add_scaled(state, k2, 0.5 * dt)), control);
    const State k4 = derivative(project_state(add_scaled(state, k3, dt)), control);
    return project_state(rk4_sum(state, k1, k2, k3, k4, dt));
}

void TruckModel::rollout(const State& initial_state, const Control* controls,
                         std::size_t count, double dt, State* out_states,
                         Integrator integrator) const {
    if (out_states == nullptr || (count > 0 && controls == nullptr)) {
        throw std::invalid_argument("rollout buffers must not be null");
    }
    out_states[0] = project_state(initial_state);
    for (std::size_t i = 0; i < count; ++i) {
        out_states[i + 1] = step(out_states[i], controls[i], dt, integrator);
    }
}

} // namespace truck_model

