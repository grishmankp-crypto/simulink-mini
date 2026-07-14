#include "Graph.hpp"
#include "Scheduler.hpp"
#include "Simulation.hpp"
#include "PSO.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace simmini;

// ---------------------------------------------------------------------------
// Plant: a bare Integrator (dy/dt = u). This has NO natural damping of its
// own -- any settling behavior in the closed loop comes entirely from the
// controller. That makes it a good, honest test of whether PSO actually
// finds gains that produce a well-damped response rather than an oscillatory
// or unstable one; a badly-tuned PID on this plant will visibly ring or blow
// up, not just be "a bit slow."
//
// Wiring: ref -> sum_e(+,-) -> pid1 -> plant -> (feeds back into sum_e)
//                                            \-> scope
// ---------------------------------------------------------------------------
static Graph buildClosedLoopGraph(double Kp, double Ki, double Kd, double Tf) {
    Graph g;
    g.addBlock({"ref",   BlockType::Constant,   {{"value", 1.0}},                             {}});
    g.addBlock({"sum_e", BlockType::Sum,        {},                                           {"+", "-"}});
    g.addBlock({"pid1",  BlockType::PID,        {{"Kp", Kp}, {"Ki", Ki}, {"Kd", Kd}, {"Tf", Tf}}, {}});
    g.addBlock({"plant", BlockType::Integrator, {{"initial", 0.0}},                            {}});
    g.addBlock({"scope", BlockType::Scope,      {},                                            {}});
    g.addEdge("ref",   "sum_e", 0);
    g.addEdge("plant", "sum_e", 1);
    g.addEdge("sum_e", "pid1",  0);
    g.addEdge("pid1",  "plant", 0);
    g.addEdge("plant", "scope", 0);
    return g;
}

struct SimResult {
    double ise;          // integral of squared error: sum(e^2 * dt)
    double effort;       // integral of squared control signal: sum(u^2 * dt)
    double finalError;   // e at t = tEnd, i.e. how well it's tracking by the end
    bool diverged;
};

// Cost = ISE + lambda * control-effort. Without the effort term, ISE alone
// has NO reason to stop increasing gains: a more aggressive controller is
// always at least as fast, right up until the point of instability, so an
// ISE-only objective just pushes every gain to whatever upper bound you
// hand it -- which is exactly what happened on the first pass of this test
// (Kp, Ki, Kd all pinned to their maximum). Real PID tuning objectives
// (this is standard practice, not a workaround) always trade off tracking
// error against actuator effort, since an "optimal" controller that
// demands unbounded control signal isn't realizable on real hardware.
static double effortPenaltyLambda() { return 0.02; }

static SimResult simulateClosedLoop(double Kp, double Ki, double Kd, double Tf,
                                     double dt, double tEnd) {
    Graph g = buildClosedLoopGraph(Kp, Ki, Kd, Tf);
    Scheduler sched(g);
    auto order = sched.computeExecutionOrder();
    Simulation sim(g, order);

    const int steps = static_cast<int>(tEnd / dt);
    double ise = 0.0, effort = 0.0;
    double lastError = 0.0;

    for (int i = 0; i <= steps; ++i) {
        auto outputs = sim.evaluateNetwork(sim.state());
        double y = outputs.at("plant");
        double u = outputs.at("pid1");
        double error = 1.0 - y; // reference is the constant 1.0
        if (!std::isfinite(error) || std::fabs(error) > 1e6) {
            return {1e12, 0.0, error, true}; // diverged -- huge cost, PSO learns to avoid this region
        }
        ise += error * error * dt;
        effort += u * u * dt;
        lastError = error;
        if (i < steps) sim.stepRK4(dt);
    }
    return {ise, effort, lastError, false};
}

int main() {
    const double Tf = 0.05;       // filter time constant, fixed (not tuned by PSO)
    const double dt = 0.005;
    const double tEnd = 5.0;

    // --- Baseline: a deliberately mediocre hand-picked guess ---
    double baseKp = 0.5, baseKi = 0.1, baseKd = 0.0;
    SimResult baseline = simulateClosedLoop(baseKp, baseKi, baseKd, Tf, dt, tEnd);
    double baselineCost = baseline.ise + effortPenaltyLambda() * baseline.effort;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Baseline gains: Kp=" << baseKp << " Ki=" << baseKi << " Kd=" << baseKd << "\n";
    std::cout << "  ISE = " << std::scientific << baseline.ise
               << ", control effort = " << baseline.effort
               << ", combined cost = " << baselineCost
               << ", final tracking error = " << baseline.finalError << "\n\n";

    // --- PSO: search Kp in [0,20], Ki in [0,50], Kd in [0,5] to minimize
    // ISE + lambda*effort, not ISE alone ---
    PSO pso(/*numParticles=*/25,
            /*lowerBounds=*/{0.0, 0.0, 0.0},
            /*upperBounds=*/{20.0, 50.0, 5.0});

    auto costFn = [&](const std::vector<double>& p) {
        SimResult r = simulateClosedLoop(p[0], p[1], p[2], Tf, dt, tEnd);
        return r.ise + effortPenaltyLambda() * r.effort;
    };

    std::cout << std::fixed;
    std::cout << "Running PSO (25 particles, 40 iterations)...\n";
    PSOResult result = pso.optimize(costFn, /*maxIters=*/40);

    double tunedKp = result.bestParams[0];
    double tunedKi = result.bestParams[1];
    double tunedKd = result.bestParams[2];
    SimResult tuned = simulateClosedLoop(tunedKp, tunedKi, tunedKd, Tf, dt, tEnd);
    double tunedCost = tuned.ise + effortPenaltyLambda() * tuned.effort;

    std::cout << "\nPSO-tuned gains: Kp=" << tunedKp << " Ki=" << tunedKi << " Kd=" << tunedKd << "\n";
    std::cout << "  ISE = " << std::scientific << tuned.ise
               << ", control effort = " << tuned.effort
               << ", combined cost = " << tunedCost
               << ", final tracking error = " << tuned.finalError << "\n\n";

    double improvementFactor = baselineCost / tunedCost;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Improvement factor (baseline cost / tuned cost): " << improvementFactor << "x\n";

    // Flag (not fail) if PSO pinned any gain to a search-space bound -- that
    // would mean the true optimum lies outside the box I gave it, which is
    // worth knowing rather than silently accepting.
    const double eps = 1e-6;
    std::vector<double> lo = {0.0, 0.0, 0.0}, hi = {20.0, 50.0, 5.0};
    bool pinned = false;
    for (int d = 0; d < 3; ++d) {
        if (result.bestParams[d] <= lo[d] + eps || result.bestParams[d] >= hi[d] - eps) pinned = true;
    }
    if (pinned) {
        std::cout << "NOTE: at least one tuned gain landed on a search-space bound -- "
                     "the true unconstrained optimum may lie outside [Kp,Ki,Kd] in "
                     "[0,20]x[0,50]x[0,5].\n";
    }

    // Convergence sanity check: PSO's best-so-far cost should be
    // monotonically non-increasing (that's a structural guarantee of PSO's
    // "keep the best ever seen" bookkeeping, not just a hope).
    bool monotonic = true;
    for (size_t i = 1; i < result.costHistory.size(); ++i) {
        if (result.costHistory[i] > result.costHistory[i - 1] + 1e-12) { monotonic = false; break; }
    }

    bool pass = true;
    if (tuned.diverged) {
        std::cout << "FAIL: PSO-tuned gains produced a diverging simulation.\n";
        pass = false;
    }
    if (!monotonic) {
        std::cout << "FAIL: PSO's best-cost-so-far history was not monotonically non-increasing.\n";
        pass = false;
    }
    if (improvementFactor < 3.0) {
        std::cout << "FAIL: expected at least a 3x combined-cost improvement over the mediocre baseline, got "
                   << improvementFactor << "x\n";
        pass = false;
    }
    if (std::fabs(tuned.finalError) > 0.05) {
        std::cout << "FAIL: tuned system did not converge close to the setpoint (final error = "
                   << tuned.finalError << ")\n";
        pass = false;
    }

    if (pass) {
        std::cout << "PASS: PSO found gains that are stable, converge to the setpoint, "
                     "and substantially beat the baseline.\n";
        return 0;
    }
    return 1;
}
