#include "Graph.hpp"
#include "Scheduler.hpp"
#include "JsonLoader.hpp"
#include <iostream>

using namespace simmini;

static void printOrder(const std::vector<std::string>& order) {
    for (size_t i = 0; i < order.size(); ++i) {
        std::cout << order[i];
        if (i + 1 < order.size()) std::cout << " -> ";
    }
    std::cout << "\n";
}

int main() {
    int failures = 0;

    // --- Test 1: valid feedback loop (Integrator legally breaks the cycle) ---
    std::cout << "[Test 1] Valid feedback loop (Sum -> Gain -> Integrator -> Sum)\n";
    try {
        Graph g = loadGraphFromFile("tests/valid_feedback_loop.json");
        Scheduler sched(g);
        auto order = sched.computeExecutionOrder();
        std::cout << "  PASS: execution order computed successfully: ";
        printOrder(order);
        if (order.size() != g.numBlocks()) {
            std::cout << "  FAIL: order size mismatch\n";
            failures++;
        }
    } catch (const std::exception& e) {
        std::cout << "  FAIL (unexpected exception): " << e.what() << "\n";
        failures++;
    }

    std::cout << "\n";

    // --- Test 2: broken algebraic loop (no memory block anywhere) ---
    std::cout << "[Test 2] Broken algebraic loop (Sum -> Gain -> Sum, no memory block)\n";
    try {
        Graph g = loadGraphFromFile("tests/broken_algebraic_loop.json");
        Scheduler sched(g);
        auto order = sched.computeExecutionOrder();
        std::cout << "  FAIL: expected AlgebraicLoopError but got a valid order: ";
        printOrder(order);
        failures++;
    } catch (const AlgebraicLoopError& e) {
        std::cout << "  PASS: correctly rejected with message:\n    " << e.what() << "\n";
    } catch (const std::exception& e) {
        std::cout << "  FAIL (wrong exception type): " << e.what() << "\n";
        failures++;
    }

    std::cout << "\n";

    // --- Test 3: algebraic loop through a PID block's proportional path ---
    // This is the case the Week 1/2 scheduler would have MISSED: it treated
    // PID as purely stateful (hasMemory==true meant "safe to break any
    // loop"), but PID's Kp term is direct feedthrough. A loop through
    // Sum -> PID -> Sum with no other memory-breaking block is a genuine
    // algebraic loop and must be rejected.
    std::cout << "[Test 3] Algebraic loop through PID's proportional (Kp) path\n";
    try {
        Graph g = loadGraphFromFile("tests/broken_loop_through_pid.json");
        Scheduler sched(g);
        auto order = sched.computeExecutionOrder();
        std::cout << "  FAIL: expected AlgebraicLoopError but got a valid order: ";
        printOrder(order);
        failures++;
    } catch (const AlgebraicLoopError& e) {
        std::cout << "  PASS: correctly rejected with message:\n    " << e.what() << "\n";
    } catch (const std::exception& e) {
        std::cout << "  FAIL (wrong exception type): " << e.what() << "\n";
        failures++;
    }

    std::cout << "\n";
    if (failures == 0) {
        std::cout << "ALL TESTS PASSED\n";
        return 0;
    } else {
        std::cout << failures << " TEST(S) FAILED\n";
        return 1;
    }
}
