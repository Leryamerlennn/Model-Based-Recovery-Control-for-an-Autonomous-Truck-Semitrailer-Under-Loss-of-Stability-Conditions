#pragma once

#include "truck_model/truck_model.hpp"

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace truck_model {

// Parameters of the finite-horizon sampling problem.  They deliberately live
// outside Parameters: they tune the controller, not the vehicle model.
struct MppiConfig {
    std::size_t horizon{50};
    std::size_t samples{1000};
    double dt{0.02};
    double temperature{10.0};
    double steering_rate_noise_std{0.10};
    double acceleration_noise_std{0.50};
    std::uint64_t random_seed{1};
};

// Stage cost for skid recovery.  The slip term uses the trailer axle slip
// angle returned by Diagnostics::trailer_slip_angle.
struct MppiCostWeights {
    double trailer_slip_angle{1.0};
    double tractor_yaw_rate{1.0};
    double yaw_rate_difference{1.0};
    double articulation_angle{1.0};
    double control_effort{0.1};
    double terminal_multiplier{5.0};
};

// A minimal sampled MPPI controller.  It owns a warm-start control sequence;
// compute() updates this sequence and returns only its first control.
class MppiController {
public:
    MppiController(TruckModel model, MppiConfig config = {},
                   MppiCostWeights weights = {});

    [[nodiscard]] Control compute(const State& current_state);
    void reset(Control fill = {});

    [[nodiscard]] const std::vector<Control>& nominal_controls() const noexcept {
        return nominal_controls_;
    }
    [[nodiscard]] double last_best_cost() const noexcept { return last_best_cost_; }

private:
    [[nodiscard]] Control clamp_control(Control control) const noexcept;
    [[nodiscard]] double state_cost(const State& state, const Control& control) const;
    [[nodiscard]] double trajectory_cost(const std::vector<State>& states,
                                         const std::vector<Control>& controls) const;
    void shift_warm_start();

    TruckModel model_;
    MppiConfig config_;
    MppiCostWeights weights_;
    std::vector<Control> nominal_controls_;
    std::vector<Control> sampled_controls_;
    std::vector<State> sampled_states_;
    std::vector<std::vector<Control>> sampled_deltas_;
    std::vector<double> costs_;
    double last_best_cost_{};
    std::mt19937_64 engine_;
};

} // namespace truck_model
