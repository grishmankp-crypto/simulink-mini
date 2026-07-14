#include "Graph.hpp"
#include "Scheduler.hpp"
#include "JsonLoader.hpp"
#include "Simulation.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>

using namespace simmini;

// Closed-form solution for a PID block fed a CONSTANT input e0, starting
// from zero state, derived from the PID's own ODE system:
//   dI/dt = e0                        => I(t)  = e0 * t
//   dDf/dt = (e0 - Df) / Tf            => Df(t) = e0 * (1 - exp(-t/Tf))
//   y(t)  = Kp*e0 + Ki*I(t) + Kd*(e0 - Df(t))/Tf
//         = Kp*e0 + Ki*e0*t + (Kd*e0/Tf) * exp(-t/Tf)
// This lets us check the PID implementation exactly, independent of any
// closed-loop control scenario (which would require solving a 3rd-order
// transfer function to get an analytical reference).
static double pidAnalytical(double t, double e0, double Kp, double Ki, double Kd, double Tf) {
    return Kp * e0 + Ki * e0 * t + (Kd * e0 / Tf) * std::exp(-t / Tf);
}

int main() {
    Graph g = loadGraphFromFile("tests/pid_step_response.json");
    Scheduler sched(g);
    auto order = sched.computeExecutionOrder();
    std::cout << "Execution order: ";
    for (size_t i = 0; i < order.size(); ++i)
        std::cout << order[i] << (i + 1 < order.size() ? " -> " : "\n");

    Simulation sim(g, order);

    const double e0 = 2.0, Kp = 3.0, Ki = 5.0, Kd = 0.5, Tf = 0.02;
    const double dt = 0.0005;   // fine step: Tf=0.02 means the filter pole is fast, needs small dt
    const double tEnd = 2.0;
    const int steps = static_cast<int>(tEnd / dt);

    double maxAbsError = 0.0;
    double tMaxErr = 0.0;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n   t        y_sim         y_exact       abs_error\n";

    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        auto outputs = sim.evaluateNetwork(sim.state());
        double ySim = outputs.at("pid1");
        double yExact = pidAnalytical(t, e0, Kp, Ki, Kd, Tf);
        double err = std::fabs(ySim - yExact);
        if (err > maxAbsError) { maxAbsError = err; tMaxErr = t; }

        if (i % 400 == 0) {
            std::cout << "  " << t << "   " << ySim << "   " << yExact << "   " << err << "\n";
        }
        if (i < steps) sim.stepRK4(dt);
    }

    std::cout << "\nMax absolute error over t in [0, " << tEnd << "]: "
              << std::scientific << maxAbsError << " at t=" << tMaxErr << "\n";

    const double threshold = 1e-4;
    if (maxAbsError < threshold) {
        std::cout << "PASS: PID block matches closed-form solution (error < " << threshold << ")\n";
        return 0;
    } else {
        std::cout << "FAIL: error exceeds threshold " << threshold << "\n";
        return 1;
    }
}
