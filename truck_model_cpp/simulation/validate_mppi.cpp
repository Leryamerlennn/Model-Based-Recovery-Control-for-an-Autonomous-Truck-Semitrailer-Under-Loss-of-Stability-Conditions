#include "controller/mppi_controller.hpp"
#include "truck_model/truck_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kDt = 0.02;
constexpr double kDuration = 5.0;
constexpr std::size_t kSteps = static_cast<std::size_t>(kDuration / kDt);

enum class Algorithm { zero, stabilizer, mppi_steering, mppi_full };

struct Disturbance {
    std::string name;
    std::string severity;
    double vy{};
    double r1{};
    double r2{};
    double eta{};
};

struct Uncertainty {
    std::string name;
    bool model_error{};
    bool delay{};
    bool noise{};
};

struct Metrics {
    bool success{};
    bool finite{true};
    double recovery_time{std::numeric_limits<double>::quiet_NaN()};
    double peak_abs_eta{};
    double peak_abs_r1{};
    double peak_abs_r2{};
    double peak_abs_trailer_slip{};
    double integrated_error{};
    double control_energy{};
    double steering_effort{};
    double braking_effort{};
    double minimum_vx{std::numeric_limits<double>::infinity()};
    double final_vx{};
    double final_vy{};
    double final_r1{};
    double final_r2{};
    double final_eta{};
};

const char* algorithm_name(Algorithm algorithm) {
    switch (algorithm) {
    case Algorithm::zero: return "zero";
    case Algorithm::stabilizer: return "stabilizer";
    case Algorithm::mppi_steering: return "mppi_steering";
    case Algorithm::mppi_full: return "mppi_full";
    }
    return "unknown";
}

bool finite_state(const truck_model::State& state) {
    for (double value : state.as_array()) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}

truck_model::Control stabilizing_control(const truck_model::State& state,
                                         const truck_model::Parameters& p) {
    const double eta = state.psi_tractor - state.psi_trailer;
    const double steering_rate =
        -1.8 * state.steering_angle - 0.75 * state.yaw_rate_tractor
        - 0.45 * (state.yaw_rate_tractor - state.yaw_rate_trailer)
        - 0.65 * eta - 0.08 * state.vy;
    return {clamp(steering_rate, -p.max_steering_rate, p.max_steering_rate), 0.0};
}

truck_model::Parameters prediction_parameters(const truck_model::Parameters& plant,
                                               bool model_error) {
    auto prediction = plant;
    if (!model_error) return prediction;
    prediction.tractor_mass *= 0.85;
    prediction.trailer_mass *= 1.15;
    prediction.tractor_yaw_inertia *= 0.80;
    prediction.trailer_yaw_inertia *= 0.80;
    prediction.front_cornering_stiffness *= 1.25;
    prediction.rear_cornering_stiffness *= 1.25;
    prediction.trailer_cornering_stiffness *= 1.25;
    prediction.friction_coefficient = std::min(1.0, plant.friction_coefficient * 1.25);
    prediction.l2 *= 1.10;
    prediction.b1 *= 0.90;
    return prediction;
}

truck_model::State noisy_measurement(const truck_model::State& state, bool enabled,
                                     std::mt19937_64& engine) {
    if (!enabled) return state;
    std::normal_distribution<double> position_noise(0.0, 0.02);
    std::normal_distribution<double> angle_noise(0.0, 0.005);
    std::normal_distribution<double> vx_noise(0.0, 0.20);
    std::normal_distribution<double> vy_noise(0.0, 0.10);
    std::normal_distribution<double> yaw_rate_noise(0.0, 0.010);
    std::normal_distribution<double> steering_noise(0.0, 0.003);
    auto measured = state;
    measured.x += position_noise(engine);
    measured.y += position_noise(engine);
    measured.psi_tractor += angle_noise(engine);
    measured.psi_trailer += angle_noise(engine);
    measured.vx += vx_noise(engine);
    measured.vy += vy_noise(engine);
    measured.yaw_rate_tractor += yaw_rate_noise(engine);
    measured.yaw_rate_trailer += yaw_rate_noise(engine);
    measured.steering_angle += steering_noise(engine);
    return measured;
}

bool inside_recovery_corridor(const truck_model::State& state,
                              const truck_model::Diagnostics& diagnostics) {
    return std::abs(state.vy) < 0.15
        && std::abs(state.yaw_rate_tractor) < 0.025
        && std::abs(state.yaw_rate_trailer) < 0.025
        && std::abs(diagnostics.articulation_angle) < 0.04
        && std::abs(diagnostics.trailer_slip_angle) < 0.03
        && state.vx > 2.0;
}

std::uint64_t scenario_seed(double mu, double speed, const Disturbance& disturbance,
                            const Uncertainty& uncertainty, Algorithm algorithm) {
    std::uint64_t seed = 1469598103934665603ULL;
    const auto mix = [&seed](std::uint64_t value) {
        seed ^= value;
        seed *= 1099511628211ULL;
    };
    mix(static_cast<std::uint64_t>(std::llround(mu * 1000.0)));
    mix(static_cast<std::uint64_t>(std::llround(speed * 100.0)));
    for (char value : disturbance.name + disturbance.severity + uncertainty.name) {
        mix(static_cast<unsigned char>(value));
    }
    mix(static_cast<std::uint64_t>(algorithm));
    return seed;
}

Metrics run_episode(double mu, double speed, const Disturbance& disturbance,
                    const Uncertainty& uncertainty, Algorithm algorithm,
                    std::ofstream* trajectory_output) {
    truck_model::Parameters plant_parameters;
    plant_parameters.friction_coefficient = mu;
    truck_model::TruckModel plant(plant_parameters);

    truck_model::State state{};
    state.vx = speed;
    state.vy = disturbance.vy;
    state.psi_tractor = 0.5 * disturbance.eta;
    state.psi_trailer = -0.5 * disturbance.eta;
    state.yaw_rate_tractor = disturbance.r1;
    state.yaw_rate_trailer = disturbance.r2;

    std::unique_ptr<truck_model::MppiController> mppi;
    if (algorithm == Algorithm::mppi_steering || algorithm == Algorithm::mppi_full) {
        truck_model::MppiConfig config;
        config.dt = kDt;
        config.horizon = 50;
        config.samples = 72;
        config.temperature = 6.0;
        config.steering_rate_noise_std = 0.22;
        config.acceleration_noise_std =
            algorithm == Algorithm::mppi_full ? 1.20 : 0.0;
        config.random_seed = scenario_seed(mu, speed, disturbance, uncertainty, algorithm);

        truck_model::MppiCostWeights weights;
        weights.trailer_slip_angle = 120.0;
        weights.tractor_yaw_rate = 25.0;
        weights.yaw_rate_difference = 35.0;
        weights.articulation_angle = 240.0;
        weights.control_effort = 0.025;
        weights.terminal_multiplier = 8.0;
        mppi = std::make_unique<truck_model::MppiController>(
            truck_model::TruckModel(prediction_parameters(
                plant_parameters, uncertainty.model_error)), config, weights);
    }

    constexpr double delay_seconds = 0.15;
    const double delay_in_steps = delay_seconds / kDt;
    const std::size_t delay_whole_steps = uncertainty.delay
        ? static_cast<std::size_t>(std::floor(delay_in_steps)) : 0;
    const double delay_fraction = uncertainty.delay
        ? delay_in_steps - static_cast<double>(delay_whole_steps) : 0.0;
    std::deque<truck_model::Control> delay_line(
        uncertainty.delay ? delay_whole_steps + 1 : 0,
        truck_model::Control{});
    std::mt19937_64 measurement_engine(
        scenario_seed(mu, speed, disturbance, uncertainty, Algorithm::zero));

    Metrics metrics;
    std::vector<bool> stable(kSteps + 1, false);
    for (std::size_t step = 0; step <= kSteps; ++step) {
        const auto diagnostics = plant.evaluate(state, {});
        stable[step] = inside_recovery_corridor(state, diagnostics);
        metrics.peak_abs_eta = std::max(metrics.peak_abs_eta,
                                        std::abs(diagnostics.articulation_angle));
        metrics.peak_abs_r1 = std::max(metrics.peak_abs_r1,
                                       std::abs(state.yaw_rate_tractor));
        metrics.peak_abs_r2 = std::max(metrics.peak_abs_r2,
                                       std::abs(state.yaw_rate_trailer));
        metrics.peak_abs_trailer_slip = std::max(
            metrics.peak_abs_trailer_slip, std::abs(diagnostics.trailer_slip_angle));
        metrics.minimum_vx = std::min(metrics.minimum_vx, state.vx);

        if (trajectory_output != nullptr) {
            *trajectory_output << std::fixed << std::setprecision(8)
                << step * kDt << ',' << algorithm_name(algorithm) << ','
                << state.x << ',' << state.y << ',' << state.vx << ',' << state.vy
                << ',' << state.yaw_rate_tractor << ',' << state.yaw_rate_trailer
                << ',' << diagnostics.articulation_angle << ','
                << diagnostics.trailer_slip_angle << '\n';
        }

        if (step == kSteps) break;
        const truck_model::State measured = noisy_measurement(
            state, uncertainty.noise, measurement_engine);
        truck_model::Control requested{};
        if (algorithm == Algorithm::stabilizer) {
            requested = stabilizing_control(measured, plant_parameters);
        } else if (mppi != nullptr) {
            requested = mppi->compute(measured);
            if (algorithm == Algorithm::mppi_steering) {
                requested.longitudinal_acceleration = 0.0;
            }
        }

        truck_model::Control applied = requested;
        if (uncertainty.delay) {
            delay_line.push_back(requested);
            const auto& older = delay_line[0];
            const auto& newer = delay_line[1];
            applied.steering_rate =
                delay_fraction * older.steering_rate
                + (1.0 - delay_fraction) * newer.steering_rate;
            applied.longitudinal_acceleration =
                delay_fraction * older.longitudinal_acceleration
                + (1.0 - delay_fraction) * newer.longitudinal_acceleration;
            delay_line.pop_front();
        }

        const double normalized_error =
            std::pow(state.vy / 1.0, 2)
            + std::pow(state.yaw_rate_tractor / 0.20, 2)
            + std::pow(state.yaw_rate_trailer / 0.20, 2)
            + std::pow(diagnostics.articulation_angle / 0.20, 2)
            + std::pow(diagnostics.trailer_slip_angle / 0.20, 2);
        metrics.integrated_error += normalized_error * kDt;
        metrics.steering_effort += applied.steering_rate * applied.steering_rate * kDt;
        metrics.braking_effort +=
            std::pow(std::min(0.0, applied.longitudinal_acceleration), 2) * kDt;
        metrics.control_energy +=
            (applied.steering_rate * applied.steering_rate
             + applied.longitudinal_acceleration * applied.longitudinal_acceleration)
            * kDt;

        state = plant.step(state, applied, kDt, truck_model::Integrator::rk4);
        if (!finite_state(state)) {
            metrics.finite = false;
            break;
        }
    }

    metrics.final_vx = state.vx;
    metrics.final_vy = state.vy;
    metrics.final_r1 = state.yaw_rate_tractor;
    metrics.final_r2 = state.yaw_rate_trailer;
    metrics.final_eta = state.psi_tractor - state.psi_trailer;

    if (metrics.finite) {
        for (std::size_t step = 0; step < stable.size(); ++step) {
            if (std::all_of(stable.begin() + static_cast<std::ptrdiff_t>(step),
                            stable.end(), [](bool value) { return value; })) {
                metrics.recovery_time = step * kDt;
                break;
            }
        }
    }
    metrics.success = metrics.finite
        && std::isfinite(metrics.recovery_time)
        && metrics.peak_abs_eta < 0.80
        && metrics.peak_abs_r1 < 1.0
        && metrics.peak_abs_r2 < 1.0
        && metrics.minimum_vx > 2.0;
    return metrics;
}

void write_result(std::ofstream& output, double mu, double speed,
                  const Disturbance& disturbance, const Uncertainty& uncertainty,
                  Algorithm algorithm, const Metrics& metrics) {
    output << std::fixed << std::setprecision(8)
        << mu << ',' << speed << ',' << disturbance.name << ','
        << disturbance.severity << ',' << disturbance.vy << ','
        << disturbance.r1 << ',' << disturbance.r2 << ',' << disturbance.eta << ','
        << uncertainty.name << ',' << algorithm_name(algorithm) << ','
        << (metrics.success ? 1 : 0) << ',' << (metrics.finite ? 1 : 0) << ','
        << metrics.recovery_time << ',' << metrics.peak_abs_eta << ','
        << metrics.peak_abs_r1 << ',' << metrics.peak_abs_r2 << ','
        << metrics.peak_abs_trailer_slip << ',' << metrics.integrated_error << ','
        << metrics.control_energy << ',' << metrics.steering_effort << ','
        << metrics.braking_effort << ',' << metrics.minimum_vx << ','
        << metrics.final_vx << ',' << metrics.final_vy << ',' << metrics.final_r1
        << ',' << metrics.final_r2 << ',' << metrics.final_eta << '\n';
}

} // namespace

int main(int argc, char** argv) {
    const std::filesystem::path output_directory =
        argc > 1 ? argv[1] : "experements/csv";
    const bool quick = argc > 2 && std::string(argv[2]) == "--quick";
    std::filesystem::create_directories(output_directory);

    const std::vector<double> mus = quick
        ? std::vector<double>{0.4}
        : std::vector<double>{0.7, 0.4, 0.2, 0.1};
    const std::vector<double> speeds = quick
        ? std::vector<double>{15.0}
        : std::vector<double>{10.0, 15.0, 20.0, 25.0};
    const std::vector<Disturbance> disturbances{
        {"tractor_skid", "moderate", 0.60, 0.08, 0.00, 0.03},
        {"tractor_skid", "severe", 1.50, 0.22, 0.00, 0.10},
        {"trailer_skid", "moderate", 0.30, 0.00, -0.10, 0.06},
        {"trailer_skid", "severe", 0.80, 0.00, -0.25, 0.16},
        {"jackknife", "moderate", 0.50, 0.10, -0.10, 0.18},
        {"jackknife", "severe", 1.10, 0.25, -0.25, 0.38},
    };
    const std::vector<Uncertainty> all_uncertainties{
        {"nominal", false, false, false},
        {"model_error", true, false, false},
        {"delay_150ms", false, true, false},
        {"measurement_noise", false, false, true},
        {"combined", true, true, true},
    };
    const std::vector<Uncertainty> uncertainties = quick
        ? std::vector<Uncertainty>{all_uncertainties.front()}
        : all_uncertainties;
    const std::array<Algorithm, 4> algorithms{
        Algorithm::zero, Algorithm::stabilizer,
        Algorithm::mppi_steering, Algorithm::mppi_full};

    std::ofstream results(output_directory / "mppi_experiment_results.csv");
    std::ofstream trajectories(output_directory / "mppi_representative_trajectories.csv");
    if (!results || !trajectories) {
        std::cerr << "cannot open MPPI experiment output files\n";
        return 1;
    }
    results
        << "mu,speed,disturbance,severity,initial_vy,initial_r1,initial_r2,"
        << "initial_eta,uncertainty,algorithm,success,finite,recovery_time,"
        << "peak_abs_eta,peak_abs_r1,peak_abs_r2,peak_abs_trailer_slip,"
        << "integrated_error,control_energy,steering_effort,braking_effort,"
        << "minimum_vx,final_vx,final_vy,final_r1,final_r2,final_eta\n";
    trajectories << "t,algorithm,x,y,vx,vy,r1,r2,eta,trailer_slip\n";

    std::size_t completed = 0;
    const std::size_t total = mus.size() * speeds.size() * disturbances.size()
        * uncertainties.size() * algorithms.size();
    try {
        for (double mu : mus) {
            for (double speed : speeds) {
                for (const auto& disturbance : disturbances) {
                    for (const auto& uncertainty : uncertainties) {
                        for (Algorithm algorithm : algorithms) {
                            const bool representative = mu == 0.4 && speed == 15.0
                                && disturbance.name == "jackknife"
                                && disturbance.severity == "severe"
                                && uncertainty.name == "nominal";
                            Metrics metrics = run_episode(
                                mu, speed, disturbance, uncertainty, algorithm,
                                representative ? &trajectories : nullptr);
                            write_result(results, mu, speed, disturbance, uncertainty,
                                         algorithm, metrics);
                            ++completed;
                            if (completed % 100 == 0 || completed == total) {
                                std::cout << "completed " << completed << '/' << total
                                          << '\n';
                            }
                        }
                    }
                }
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "MPPI experiment failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "wrote " << (output_directory / "mppi_experiment_results.csv")
              << '\n';
    std::cout << "wrote "
              << (output_directory / "mppi_representative_trajectories.csv") << '\n';
}
