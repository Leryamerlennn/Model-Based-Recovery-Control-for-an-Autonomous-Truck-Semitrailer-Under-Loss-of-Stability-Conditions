#include "truck_model/mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace truck_model {

namespace {

bool finite_state(const State& state) {
    for (double value : state.as_array()) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

void validate(const MppiConfig& config, const MppiCostWeights& weights) {
    if (config.horizon == 0 || config.samples == 0 || !(config.dt > 0.0) ||
        !(config.temperature > 0.0) || config.steering_rate_noise_std < 0.0 ||
        config.acceleration_noise_std < 0.0) {
        throw std::invalid_argument("MPPI configuration has invalid dimensions or scales");
    }
    if (weights.trailer_slip_angle < 0.0 || weights.tractor_yaw_rate < 0.0 ||
        weights.yaw_rate_difference < 0.0 || weights.articulation_angle < 0.0 ||
        weights.control_effort < 0.0 || weights.terminal_multiplier < 0.0) {
        throw std::invalid_argument("MPPI cost weights must be non-negative");
    }
}

} // namespace

MppiController::MppiController(TruckModel model, MppiConfig config,
                               MppiCostWeights weights)
    : model_(std::move(model)), config_(config), weights_(weights),
      nominal_controls_(config.horizon), sampled_controls_(config.horizon),
      sampled_states_(config.horizon + 1),
      sampled_deltas_(config.samples, std::vector<Control>(config.horizon)),
      costs_(config.samples), engine_(config.random_seed) {
    validate(config_, weights_);
}

Control MppiController::clamp_control(Control control) const noexcept {
    const auto& p = model_.parameters();
    control.steering_rate = std::clamp(control.steering_rate,
                                       -p.max_steering_rate, p.max_steering_rate);
    control.longitudinal_acceleration = std::clamp(
        control.longitudinal_acceleration, -p.max_braking_deceleration,
        p.max_acceleration);
    return control;
}

double MppiController::state_cost(const State& state, const Control& control) const {
    const Diagnostics d = model_.evaluate(state, control);
    const double yaw_difference = state.yaw_rate_tractor - state.yaw_rate_trailer;
    return weights_.trailer_slip_angle * d.trailer_slip_angle * d.trailer_slip_angle +
        weights_.tractor_yaw_rate * state.yaw_rate_tractor * state.yaw_rate_tractor +
        weights_.yaw_rate_difference * yaw_difference * yaw_difference +
        weights_.articulation_angle * d.articulation_angle * d.articulation_angle +
        weights_.control_effort *
            (control.steering_rate * control.steering_rate +
             control.longitudinal_acceleration * control.longitudinal_acceleration);
}

double MppiController::trajectory_cost(const std::vector<State>& states,
                                       const std::vector<Control>& controls) const {
    double cost = 0.0;
    for (std::size_t k = 0; k < controls.size(); ++k) {
        if (!finite_state(states[k + 1])) return std::numeric_limits<double>::infinity();
        cost += state_cost(states[k + 1], controls[k]);
    }
    if (!states.empty() && !controls.empty()) {
        cost += weights_.terminal_multiplier *
            state_cost(states.back(), controls.back());
    }
    return cost;
}

Control MppiController::compute(const State& current_state) {
    std::normal_distribution<double> steering_noise(
        0.0, config_.steering_rate_noise_std);
    std::normal_distribution<double> acceleration_noise(
        0.0, config_.acceleration_noise_std);

    for (std::size_t sample = 0; sample < config_.samples; ++sample) {
        for (std::size_t k = 0; k < config_.horizon; ++k) {
            const Control raw_delta{steering_noise(engine_), acceleration_noise(engine_)};
            sampled_controls_[k] = clamp_control({
                nominal_controls_[k].steering_rate + raw_delta.steering_rate,
                nominal_controls_[k].longitudinal_acceleration +
                    raw_delta.longitudinal_acceleration});
            // Keep the effective, bounded perturbation.  This prevents a clipped
            // candidate from pulling the nominal plan beyond its actuator limits.
            sampled_deltas_[sample][k] = {
                sampled_controls_[k].steering_rate - nominal_controls_[k].steering_rate,
                sampled_controls_[k].longitudinal_acceleration -
                    nominal_controls_[k].longitudinal_acceleration};
        }
        model_.rollout(current_state, sampled_controls_.data(), config_.horizon,
                       config_.dt, sampled_states_.data());
        costs_[sample] = trajectory_cost(sampled_states_, sampled_controls_);
    }

    const double minimum_cost = *std::min_element(costs_.begin(), costs_.end());
    last_best_cost_ = minimum_cost;
    if (!std::isfinite(minimum_cost)) {
        throw std::runtime_error("all MPPI sampled trajectories are invalid");
    }

    std::vector<double> normalized_weights(config_.samples);
    double weight_sum = 0.0;
    for (std::size_t sample = 0; sample < config_.samples; ++sample) {
        normalized_weights[sample] = std::isfinite(costs_[sample])
            ? std::exp(-(costs_[sample] - minimum_cost) / config_.temperature)
            : 0.0;
        weight_sum += normalized_weights[sample];
    }
    if (!(weight_sum > 0.0) || !std::isfinite(weight_sum)) {
        throw std::runtime_error("MPPI could not normalize trajectory weights");
    }

    for (std::size_t sample = 0; sample < config_.samples; ++sample) {
        normalized_weights[sample] /= weight_sum;
    }
    for (std::size_t k = 0; k < config_.horizon; ++k) {
        Control update{};
        for (std::size_t sample = 0; sample < config_.samples; ++sample) {
            update.steering_rate +=
                normalized_weights[sample] * sampled_deltas_[sample][k].steering_rate;
            update.longitudinal_acceleration += normalized_weights[sample] *
                sampled_deltas_[sample][k].longitudinal_acceleration;
        }
        nominal_controls_[k] = clamp_control({
            nominal_controls_[k].steering_rate + update.steering_rate,
            nominal_controls_[k].longitudinal_acceleration +
                update.longitudinal_acceleration});
    }

    const Control command = nominal_controls_.front();
    shift_warm_start();
    return command;
}

void MppiController::shift_warm_start() {
    std::move(nominal_controls_.begin() + 1, nominal_controls_.end(),
              nominal_controls_.begin());
    nominal_controls_.back() = {};
}

void MppiController::reset(Control fill) {
    std::fill(nominal_controls_.begin(), nominal_controls_.end(), clamp_control(fill));
    last_best_cost_ = 0.0;
}

} // namespace truck_model
