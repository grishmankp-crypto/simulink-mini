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
    Graph g = loadGraphFromFile("tests/shm_oscillator.json");
    Scheduler sched(g);
    auto order = sched.computeExecutionOrder();
    Simulation sim(g, order);

    // --- 1. Generate the C++ source ---
    std::string src = CodeGen::generate(g, order, sim);
    const std::string genPath = "generated_model.cpp";
    const std::string binPath = "generated_model_bin";
    {
        std::ofstream f(genPath);
        f << src;
    }
    std::cout << "Generated " << genPath << " (" << src.size() << " bytes)\n";

    // --- 2. Compile it as a fully standalone binary (no dependency on our headers) ---
    std::string compileCmd = "g++ -O2 -std=c++17 " + genPath + " -o " + binPath + " 2>&1";
    std::string compileOutput = runCommand(compileCmd);
    if (!compileOutput.empty()) {
        std::cout << "Compiler output:\n" << compileOutput << "\n";
    }

    // --- 3. Run the generated binary, capture its CSV trajectory ---
    std::string genOutput = runCommand("./" + binPath);
    std::istringstream genStream(genOutput);
    std::vector<std::pair<double,double>> genTrace;
    std::string line;
    while (std::getline(genStream, line)) {
        if (line.empty()) continue;
        size_t comma = line.find(',');
        double t = std::stod(line.substr(0, comma));
        double x = std::stod(line.substr(comma + 1));
        genTrace.push_back({t, x});
    }
    std::cout << "Generated binary produced " << genTrace.size() << " samples.\n";

    // --- 4. Run the interpreter over the same timeline ---
    const double dt = 0.001, tEnd = 10.0;
    const int steps = static_cast<int>(tEnd / dt);
    std::vector<double> interpTrace;
    interpTrace.reserve(steps + 1);
    for (int i = 0; i <= steps; ++i) {
        interpTrace.push_back(sim.state()[1]); // int_x is index 1 (see JSON insertion order)
        if (i < steps) sim.stepRK4(dt);
    }

    if (interpTrace.size() != genTrace.size()) {
        std::cout << "FAIL: sample count mismatch (interp=" << interpTrace.size()
                  << ", generated=" << genTrace.size() << ")\n";
        return 1;
    }

    // --- 5. Diff the two trajectories ---
    double maxDiff = 0.0;
    for (size_t i = 0; i < interpTrace.size(); ++i) {
        double diff = std::fabs(interpTrace[i] - genTrace[i].second);
        if (diff > maxDiff) maxDiff = diff;
    }

    std::cout << "Max diff between interpreter and generated-code trajectories: "
              << std::scientific << maxDiff << "\n";

    const double threshold = 1e-9;
    if (maxDiff < threshold) {
        std::cout << "PASS: generated code and interpreter agree (diff < " << threshold << ")\n";
        return 0;
    } else {
        std::cout << "FAIL: generated code diverges from interpreter\n";
        return 1;
    }
}
