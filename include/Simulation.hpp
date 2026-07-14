#pragma once
#include "Graph.hpp"
#include "Scheduler.hpp"
#include <unordered_map>
#include <cmath>

namespace simmini {

// ---------------------------------------------------------------------------
// Simulation evaluates the entire block network for a *given* state vector
// (the values held by all stateful blocks) and computes:
//   (a) every block's instantaneous output, in the valid execution order
//   (b) the derivative of each stateful block's state (needed by the ODE
//       solver) — for an Integrator, that derivative is simply its input.
//
// The state vector is then advanced using classic 4th-order Runge-Kutta,
// which requires evaluating the *entire* network 4 times per timestep
// (at t, t+dt/2 twice, and t+dt) — exactly how a real block-diagram solver
// like Simulink's works: RK4 is not "integrate each state independently,"
// it's "re-evaluate the whole coupled system at each stage."
//
// PID blocks need TWO state slots, not one:
//   - an integral accumulator: dI/dt = e
//   - a filtered-derivative state: dDf/dt = (e - Df) / Tf
// Real PID controllers never differentiate the error directly (pure
// differentiation amplifies sensor noise without bound). Instead they run
// the error through a first-order low-pass filter and use the FILTER
// STATE's own derivative as the "D" term estimate. That's what Tf is:
// the filter time constant. This is a standard, deliberate implementation
// choice, not a shortcut — worth explaining if asked in an interview.
// ---------------------------------------------------------------------------
class Simulation {
public:
    Simulation(const Graph& g, std::vector<std::string> order)
        : g_(g), order_(std::move(order)) {
        // Stable, deterministic ordering of stateful blocks -> BASE state
        // vector index. Each block may occupy more than one contiguous slot
        // (Integrator: 1, PID: 2).
        int running = 0;
        for (auto& id : g_.blockIds()) {
            if (hasMemory(g_.block(id).type)) {
                stateIndex_[id] = running;
                running += numStateSlots(g_.block(id).type);
            }
        }
        state_.assign(running, 0.0);
        for (auto& [id, base] : stateIndex_) {
            const Block& b = g_.block(id);
            if (b.type == BlockType::Integrator) {
                state_[base] = getParam(b, "initial", 0.0);
            } else if (b.type == BlockType::PID) {
                state_[base]     = getParam(b, "initial_integral", 0.0); // integral accumulator
                state_[base + 1] = 0.0;                                   // filter state starts at 0
            }
        }
    }

    int stateSize() const { return static_cast<int>(state_.size()); }
    const std::vector<double>& state() const { return state_; }
    void setState(const std::vector<double>& s) { state_ = s; }
    const std::unordered_map<std::string, int>& stateIndexMap() const { return stateIndex_; }

    static int numStateSlots(BlockType t) {
        switch (t) {
            case BlockType::Integrator: return 1;
            case BlockType::PID: return 2;
            default: return 0; // UnitDelay/TransferFunctionFirstOrder: not yet implemented
        }
    }

    // Evaluate the whole network for a given state vector, without mutating
    // internal state. Returns each block's instantaneous output.
    std::unordered_map<std::string, double> evaluateNetwork(const std::vector<double>& stateIn) const {
        std::unordered_map<std::string, double> outputs;
        outputs.reserve(g_.numBlocks());

        for (auto& id : order_) {
            const Block& b = g_.block(id);

            // Gather this block's inputs, slotted by dstInputIndex.
            const auto& inEdges = g_.inEdges(id);
            int numInputs = expectedNumInputs(b);
            std::vector<double> ins(numInputs, 0.0);
            for (auto& [srcId, slot] : inEdges) {
                if (slot < 0 || slot >= numInputs) continue;
                // A producer may not yet be in `outputs` if it's a block with
                // no direct feedthrough (Integrator/UnitDelay/TF) scheduled
                // later in `order_` (its own output doesn't depend on this
                // edge, so it can be placed anywhere). That's only safe
                // because such producers are never consumed by a feedthrough
                // block before they're computed — the scheduler guarantees
                // that via hasDirectFeedthrough(). Blocks that don't need
                // `ins` for their own output may still reach this loop, so we
                // must not hard-fail on a not-yet-computed producer here.
                auto it = outputs.find(srcId);
                ins[slot] = (it != outputs.end()) ? it->second : 0.0;
            }

            double out = 0.0;
            switch (b.type) {
                case BlockType::Constant:
                    out = getParam(b, "value", 0.0);
                    break;
                case BlockType::Gain:
                    out = getParam(b, "k", 1.0) * ins[0];
                    break;
                case BlockType::Sum: {
                    double s = 0.0;
                    for (int i = 0; i < numInputs; ++i) {
                        char sign = (i < (int)b.inputSigns.size()) ? b.inputSigns[i][0] : '+';
                        s += (sign == '-') ? -ins[i] : ins[i];
                    }
                    out = s;
                    break;
                }
                case BlockType::Product: {
                    double p = 1.0;
                    for (int i = 0; i < numInputs; ++i) p *= ins[i];
                    out = p;
                    break;
                }
                case BlockType::Integrator:
                    // Output is the current state, NOT a function of the input.
                    // (The input only determines the *derivative* — see
                    // computeDerivatives below.)
                    out = stateIn[stateIndex_.at(id)];
                    break;
                case BlockType::PID: {
                    double e = ins[0];
                    int base = stateIndex_.at(id);
                    double I  = stateIn[base];
                    double Df = stateIn[base + 1];
                    double Kp = getParam(b, "Kp", 1.0);
                    double Ki = getParam(b, "Ki", 0.0);
                    double Kd = getParam(b, "Kd", 0.0);
                    double Tf = getParam(b, "Tf", 0.01);
                    out = Kp * e + Ki * I + Kd * (e - Df) / Tf;
                    break;
                }
                case BlockType::Scope:
                    out = numInputs > 0 ? ins[0] : 0.0;
                    break;
                case BlockType::UnitDelay:
                case BlockType::TransferFunctionFirstOrder:
                    throw std::runtime_error(
                        std::string("Block type not yet implemented in Simulation: ") +
                        blockTypeName(b.type));
            }
            outputs[id] = out;
        }
        return outputs;
    }

    // dx/dt for every stateful block, evaluated at the given state.
    std::vector<double> computeDerivatives(const std::vector<double>& stateIn) const {
        auto outputs = evaluateNetwork(stateIn);
        std::vector<double> deriv(stateIn.size(), 0.0);
        for (auto& [id, base] : stateIndex_) {
            const Block& b = g_.block(id);
            if (b.type == BlockType::Integrator) {
                // Derivative = the block's single input (already computed).
                double u = 0.0;
                for (auto& [srcId, slot] : g_.inEdges(id)) {
                    if (slot == 0) u = outputs.at(srcId);
                }
                deriv[base] = u;
            } else if (b.type == BlockType::PID) {
                double e = 0.0;
                for (auto& [srcId, slot] : g_.inEdges(id)) {
                    if (slot == 0) e = outputs.at(srcId);
                }
                double Df = stateIn[base + 1];
                double Tf = getParam(b, "Tf", 0.01);
                deriv[base]     = e;                 // d(integral)/dt = e
                deriv[base + 1] = (e - Df) / Tf;      // d(filter state)/dt
            }
        }
        return deriv;
    }

    // Advance internal state by dt using classic 4th-order Runge-Kutta.
    void stepRK4(double dt) {
        auto add = [&](const std::vector<double>& a, const std::vector<double>& b, double scale) {
            std::vector<double> r(a.size());
            for (size_t i = 0; i < a.size(); ++i) r[i] = a[i] + scale * b[i];
            return r;
        };

        std::vector<double> k1 = computeDerivatives(state_);
        std::vector<double> k2 = computeDerivatives(add(state_, k1, dt / 2.0));
        std::vector<double> k3 = computeDerivatives(add(state_, k2, dt / 2.0));
        std::vector<double> k4 = computeDerivatives(add(state_, k3, dt));

        for (size_t i = 0; i < state_.size(); ++i) {
            state_[i] += (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        }
    }

    static int expectedNumInputs(const Block& b) {
        switch (b.type) {
            case BlockType::Constant: return 0;
            case BlockType::Sum:
            case BlockType::Product:
                return static_cast<int>(b.inputSigns.empty() ? 2 : b.inputSigns.size());
            default: return 1;
        }
    }

    static double getParam(const Block& b, const std::string& key, double def) {
        auto it = b.params.find(key);
        return it != b.params.end() ? it->second : def;
    }

private:
    const Graph& g_;
    std::vector<std::string> order_;
    std::unordered_map<std::string, int> stateIndex_; // block id -> BASE slot index
    std::vector<double> state_;
};

} // namespace simmini
