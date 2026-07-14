#pragma once
#include <vector>
#include <random>
#include <functional>
#include <limits>
#include <algorithm>

namespace simmini {

// ---------------------------------------------------------------------------
// Standard Particle Swarm Optimization: a swarm of candidate solutions
// ("particles") move through parameter space, each pulled toward (a) its own
// best-ever position (cognitive term) and (b) the swarm's best-ever position
// (social term), with inertia carrying forward its existing velocity. No
// gradient of the cost function is needed, which is exactly why PSO is a
// good fit here: the cost function is "simulate the closed-loop system and
// measure tracking error," which has no clean analytical gradient with
// respect to (Kp, Ki, Kd).
// ---------------------------------------------------------------------------
struct PSOResult {
    std::vector<double> bestParams;
    double bestCost;
    std::vector<double> costHistory; // best-so-far cost at each iteration
};

class PSO {
public:
    using CostFn = std::function<double(const std::vector<double>&)>;

    PSO(int numParticles, std::vector<double> lowerBounds, std::vector<double> upperBounds,
        unsigned seed = 42)
        : numParticles_(numParticles),
          lb_(std::move(lowerBounds)),
          ub_(std::move(upperBounds)),
          rng_(seed) {}

    PSOResult optimize(const CostFn& cost, int maxIters,
                        double w = 0.7, double c1 = 1.5, double c2 = 1.5) {
        int dim = static_cast<int>(lb_.size());
        std::uniform_real_distribution<double> u01(0.0, 1.0);

        std::vector<std::vector<double>> pos(numParticles_, std::vector<double>(dim));
        std::vector<std::vector<double>> vel(numParticles_, std::vector<double>(dim, 0.0));
        std::vector<std::vector<double>> pbest(numParticles_, std::vector<double>(dim));
        std::vector<double> pbestCost(numParticles_, std::numeric_limits<double>::infinity());
        std::vector<double> gbest(dim, 0.0);
        double gbestCost = std::numeric_limits<double>::infinity();

        for (int i = 0; i < numParticles_; ++i) {
            for (int d = 0; d < dim; ++d) {
                double range = ub_[d] - lb_[d];
                pos[i][d] = lb_[d] + u01(rng_) * range;
                vel[i][d] = (u01(rng_) * 2.0 - 1.0) * range * 0.1;
            }
            double c = cost(pos[i]);
            pbest[i] = pos[i];
            pbestCost[i] = c;
            if (c < gbestCost) { gbestCost = c; gbest = pos[i]; }
        }

        PSOResult result;
        result.costHistory.reserve(maxIters);

        for (int iter = 0; iter < maxIters; ++iter) {
            for (int i = 0; i < numParticles_; ++i) {
                for (int d = 0; d < dim; ++d) {
                    double r1 = u01(rng_), r2 = u01(rng_);
                    vel[i][d] = w * vel[i][d]
                              + c1 * r1 * (pbest[i][d] - pos[i][d])
                              + c2 * r2 * (gbest[d] - pos[i][d]);
                    pos[i][d] += vel[i][d];
                    // Clamp to bounds and kill velocity on the clamped axis
                    // to avoid particles pinning uselessly against a wall.
                    if (pos[i][d] < lb_[d]) { pos[i][d] = lb_[d]; vel[i][d] = 0.0; }
                    if (pos[i][d] > ub_[d]) { pos[i][d] = ub_[d]; vel[i][d] = 0.0; }
                }
                double c = cost(pos[i]);
                if (c < pbestCost[i]) { pbestCost[i] = c; pbest[i] = pos[i]; }
                if (c < gbestCost) { gbestCost = c; gbest = pos[i]; }
            }
            result.costHistory.push_back(gbestCost);
        }

        result.bestParams = gbest;
        result.bestCost = gbestCost;
        return result;
    }

private:
    int numParticles_;
    std::vector<double> lb_, ub_;
    std::mt19937 rng_;
};

} // namespace simmini
