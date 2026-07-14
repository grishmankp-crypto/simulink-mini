#include "Graph.hpp"
#include "Scheduler.hpp"
#include "JsonLoader.hpp"
#include "Simulation.hpp"
#include "CodeGen.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <array>
#include <memory>
#include <vector>
#include <algorithm>

using namespace simmini;

struct PipeCloser {
    FILE* fp;
    ~PipeCloser() { if (fp) pclose(fp); }
};

static std::string runCommand(const std::string& cmd) {
    std::array<char, 4096> buf;
    std::string result;
    PipeCloser pipe{popen(cmd.c_str(), "r")};
    if (!pipe.fp) throw std::runtime_error("popen failed for: " + cmd);
    while (fgets(buf.data(), buf.size(), pipe.fp) != nullptr) result += buf.data();
    return result;
}

int main() {
    // Use the isolated-PID diagram -- this exercises the 2-state-slot
    // (integral + filtered derivative) codegen path, not just a plain
    // Integrator.
    Graph g = loadGraphFromFile("tests/pid_step_response.json");
    Scheduler sched(g);
    auto order = sched.computeExecutionOrder();
    Simulation sim(g, order);

    std::string src = CodeGen::generate(g, order, sim, "PidModel");
    const std::string genPath = "generated_pid_model.cpp";
    const std::string binPath = "generated_pid_model_bin";
    { std::ofstream f(genPath); f << src; }
    std::cout << "Generated " << genPath << " (" << src.size() << " bytes)\n";

    std::string compileOutput = runCommand("g++ -O2 -std=c++17 " + genPath + " -o " + binPath + " 2>&1");
    if (!compileOutput.empty()) std::cout << "Compiler output:\n" << compileOutput << "\n";

    std::string genOutput = runCommand("./" + binPath);
    std::istringstream genStream(genOutput);
    std::vector<std::pair<double,double>> genTrace;
    std::string line;
    while (std::getline(genStream, line)) {
        if (line.empty()) continue;
        size_t comma = line.find(',');
        genTrace.push_back({std::stod(line.substr(0, comma)), std::stod(line.substr(comma + 1))});
    }
    std::cout << "Generated binary produced " << genTrace.size() << " samples.\n";

    const double dt = 0.001, tEnd = 10.0;
    const int steps = static_cast<int>(tEnd / dt);
    std::vector<double> interpTrace;
    interpTrace.reserve(steps + 1);
    // Note: generated main() prints the state slot feeding the Scope, which
    // for this diagram is pid1's INTEGRAL state slot (index 0), not pid1's
    // full output -- matching exactly what CodeGen's stateIdxOf() picks.
    // We replicate the same quantity from the interpreter for a fair diff.
    int scopeStateIdx = -1;
    for (auto& [id, base] : sim.stateIndexMap()) {
        for (auto& [dstId, slot] : g.outEdges(id)) {
            (void)slot;
            if (g.block(dstId).type == BlockType::Scope) scopeStateIdx = base;
        }
    }
    for (int i = 0; i <= steps; ++i) {
        interpTrace.push_back(sim.state()[scopeStateIdx]);
        if (i < steps) sim.stepRK4(dt);
    }

    if ((int)interpTrace.size() != (int)genTrace.size()) {
        std::cout << "FAIL: sample count mismatch (interp=" << interpTrace.size()
                  << ", generated=" << genTrace.size() << ")\n";
        return 1;
    }

    double maxDiff = 0.0;
    for (size_t i = 0; i < interpTrace.size(); ++i) {
        maxDiff = std::max(maxDiff, std::fabs(interpTrace[i] - genTrace[i].second));
    }
    std::cout << "Max diff between interpreter and generated-code trajectories: "
              << std::scientific << maxDiff << "\n";

    const double threshold = 1e-9;
    if (maxDiff < threshold) {
        std::cout << "PASS: PID generated code and interpreter agree (diff < " << threshold << ")\n";
        return 0;
    } else {
        std::cout << "FAIL: PID generated code diverges from interpreter\n";
        return 1;
    }
}
