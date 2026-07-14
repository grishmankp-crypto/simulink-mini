#include "Graph.hpp"
#include "Scheduler.hpp"
#include "JsonLoader.hpp"
#include "Simulation.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>

using namespace simmini;

int main() {
    Graph g = loadGraphFromFile("tests/shm_oscillator.json");
    Scheduler sched(g);
    auto order = sched.computeExecutionOrder();

    std::cout << "Execution order: ";
    for (size_t i = 0; i < order.size(); ++i) {
        std::cout << order[i] << (i + 1 < order.size() ? " -> " : "\n");
    }

    Simulation sim(g, order);

    const double omega = 2.0;
    const double dt = 0.001;
    const double tEnd = 10.0;
    const int steps = static_cast<int>(tEnd / dt);

    double maxAbsError = 0.0;
    double tMaxErr = 0.0;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n   t        x_sim        x_exact      abs_error\n";

    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        double xExact = std::cos(omega * t);
        double xSim = sim.state()[1]; // int_x is the 2nd registered stateful block -> index 1

        // NOTE: index into state vector depends on insertion order of blocks
        // in the JSON (int_v first => index 0, int_x second => index 1).
        double err = std::fabs(xSim - xExact);
        if (err > maxAbsError) { maxAbsError = err; tMaxErr = t; }

        if (i % 2000 == 0) {
            std::cout << "  " << t << "   " << xSim << "   " << xExact << "   " << err << "\n";
        }

        if (i < steps) sim.stepRK4(dt);
    }

    std::cout << "\nMax absolute error over t in [0, " << tEnd << "]: "
              << std::scientific << maxAbsError << " at t=" << tMaxErr << "\n";

    // RK4 at dt=0.001 on a smooth oscillator should track the analytical
    // solution extremely tightly. This threshold is generous but would
    // clearly fail if e.g. Euler integration were used by mistake, or if
    // the derivative wiring were wrong (which would blow up, not just add
    // small error).
    const double threshold = 1e-6;
    if (maxAbsError < threshold) {
        std::cout << "PASS: RK4 simulation matches analytical solution (error < " << threshold << ")\n";
        return 0;
    } else {
        std::cout << "FAIL: error exceeds threshold " << threshold << "\n";
        return 1;
    }
}
