#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "Graph.hpp"
#include "Scheduler.hpp"
#include "Simulation.hpp"
#include "CodeGen.hpp"
#include "JsonLoader.hpp"
#include "PSO.hpp"

namespace py = pybind11;
using namespace simmini;

PYBIND11_MODULE(simmini_py, m) {
    m.doc() = "Python bindings for simulink-mini: block-diagram compiler, "
              "RK4 solver, code generator, and PSO auto-tuner.";

    py::enum_<BlockType>(m, "BlockType")
        .value("Constant", BlockType::Constant)
        .value("Gain", BlockType::Gain)
        .value("Sum", BlockType::Sum)
        .value("Product", BlockType::Product)
        .value("TransferFunctionFirstOrder", BlockType::TransferFunctionFirstOrder)
        .value("Integrator", BlockType::Integrator)
        .value("UnitDelay", BlockType::UnitDelay)
        .value("PID", BlockType::PID)
        .value("Scope", BlockType::Scope);

    py::class_<Block>(m, "Block")
        .def(py::init<>())
        .def_readwrite("id", &Block::id)
        .def_readwrite("type", &Block::type)
        .def_readwrite("params", &Block::params)
        .def_readwrite("inputSigns", &Block::inputSigns);

    py::class_<Graph>(m, "Graph")
        .def(py::init<>())
        .def("add_block", &Graph::addBlock, py::arg("block"))
        .def("add_edge", &Graph::addEdge, py::arg("src"), py::arg("dst"), py::arg("dst_input_index"))
        .def("block_ids", &Graph::blockIds)
        .def("num_blocks", &Graph::numBlocks);

    // AlgebraicLoopError becomes a genuine Python exception. Its message
    // (via str(e)) already contains the full offending cycle, e.g.
    // "Algebraic loop detected: sum1 -> pid1 -> sum1 (...)" -- the FastAPI
    // layer catches this by type and turns it into a structured 422 response
    // rather than a generic 500.
    py::register_exception<AlgebraicLoopError>(m, "AlgebraicLoopError");

    py::class_<Scheduler>(m, "Scheduler")
        .def(py::init<const Graph&>(), py::keep_alive<1, 2>()) // keep Graph alive as long as Scheduler exists
        .def("compute_execution_order", &Scheduler::computeExecutionOrder);

    py::class_<Simulation>(m, "Simulation")
        .def(py::init<const Graph&, std::vector<std::string>>(),
             py::arg("graph"), py::arg("order"), py::keep_alive<1, 2>())
        .def("state_size", &Simulation::stateSize)
        .def("state", &Simulation::state)
        .def("set_state", &Simulation::setState, py::arg("state"))
        .def("state_index_map", &Simulation::stateIndexMap)
        .def("evaluate_network", &Simulation::evaluateNetwork, py::arg("state"))
        .def("compute_derivatives", &Simulation::computeDerivatives, py::arg("state"))
        .def("step_rk4", &Simulation::stepRK4, py::arg("dt"));

    m.def("generate_code",
          [](const Graph& g, const std::vector<std::string>& order,
             const Simulation& sim, const std::string& modelName) {
              return CodeGen::generate(g, order, sim, modelName);
          },
          py::arg("graph"), py::arg("order"), py::arg("sim"), py::arg("model_name") = "GeneratedModel",
          "Generate standalone C++ source for the compiled diagram.");

    m.def("load_graph_from_json",
          [](const std::string& jsonStr) {
              auto j = nlohmann::json::parse(jsonStr);
              return loadGraphFromJson(j);
          },
          py::arg("json_str"),
          "Parse a diagram JSON string (blocks + edges) into a Graph.");

    // Generic, fast closed-loop cost evaluation for PID auto-tuning: mutates
    // the given PID block's gains in place, runs a full RK4 simulation
    // entirely in C++ (no Python call overhead per timestep), and returns
    // (ise, effort, final_error, diverged). This is what PSO's cost_fn
    // wraps -- only ONE Python<->C++ call boundary per PSO fitness
    // evaluation (not one per simulation timestep), which is what keeps a
    // 25-particle x 40-iteration search fast even though each evaluation
    // simulates hundreds of timesteps.
    m.def("simulate_closed_loop_cost",
          [](Graph& g, const std::vector<std::string>& order,
             const std::string& pidBlockId, const std::string& errorBlockId,
             double Kp, double Ki, double Kd, double Tf,
             double dt, double tEnd) -> py::tuple {
              Block& pid = g.mutableBlock(pidBlockId);
              pid.params["Kp"] = Kp;
              pid.params["Ki"] = Ki;
              pid.params["Kd"] = Kd;
              pid.params["Tf"] = Tf;

              Simulation sim(g, order);
              int steps = static_cast<int>(tEnd / dt);
              double ise = 0.0, effort = 0.0, lastError = 0.0;
              bool diverged = false;

              for (int i = 0; i <= steps; ++i) {
                  auto outputs = sim.evaluateNetwork(sim.state());
                  double e = outputs.at(errorBlockId);
                  double u = outputs.at(pidBlockId);
                  if (!std::isfinite(e) || std::fabs(e) > 1e6) {
                      diverged = true;
                      lastError = e;
                      break;
                  }
                  ise += e * e * dt;
                  effort += u * u * dt;
                  lastError = e;
                  if (i < steps) sim.stepRK4(dt);
              }
              if (diverged) return py::make_tuple(1e12, 0.0, lastError, true);
              return py::make_tuple(ise, effort, lastError, false);
          },
          py::arg("graph"), py::arg("order"), py::arg("pid_block_id"), py::arg("error_block_id"),
          py::arg("Kp"), py::arg("Ki"), py::arg("Kd"), py::arg("Tf"),
          py::arg("dt"), py::arg("t_end"),
          "Mutate a PID block's gains in place and run a full RK4 simulation "
          "entirely in C++, returning (ise, effort, final_error, diverged).");

    py::class_<PSOResult>(m, "PSOResult")
        .def_readonly("best_params", &PSOResult::bestParams)
        .def_readonly("best_cost", &PSOResult::bestCost)
        .def_readonly("cost_history", &PSOResult::costHistory);

    py::class_<PSO>(m, "PSO")
        .def(py::init<int, std::vector<double>, std::vector<double>, unsigned>(),
             py::arg("num_particles"), py::arg("lower_bounds"), py::arg("upper_bounds"),
             py::arg("seed") = 42)
        .def("optimize", &PSO::optimize,
             py::arg("cost_fn"), py::arg("max_iters"),
             py::arg("w") = 0.7, py::arg("c1") = 1.5, py::arg("c2") = 1.5);
             // NOTE: deliberately NOT releasing the GIL here. cost_fn is a
             // Python callable passed in from the FastAPI layer, so the C++
             // PSO loop calls back INTO Python on every fitness evaluation --
             // releasing the GIL around that would be incorrect (and crash).
}
