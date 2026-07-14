#pragma once
#include "Graph.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

namespace simmini {

using json = nlohmann::json;

inline BlockType parseBlockType(const std::string& s) {
    static const std::unordered_map<std::string, BlockType> map = {
        {"Constant", BlockType::Constant},
        {"Gain", BlockType::Gain},
        {"Sum", BlockType::Sum},
        {"Product", BlockType::Product},
        {"TransferFunctionFirstOrder", BlockType::TransferFunctionFirstOrder},
        {"Integrator", BlockType::Integrator},
        {"UnitDelay", BlockType::UnitDelay},
        {"PID", BlockType::PID},
        {"Scope", BlockType::Scope},
    };
    auto it = map.find(s);
    if (it == map.end()) throw std::runtime_error("Unknown block type in JSON: " + s);
    return it->second;
}

// Expected JSON schema (this is exactly what the Next.js block editor emits):
// {
//   "blocks": [
//     {"id": "c1", "type": "Constant", "params": {"value": 1.0}},
//     {"id": "g1", "type": "Gain", "params": {"k": 2.0}},
//     {"id": "sum1", "type": "Sum", "signs": ["+","-"]},
//     {"id": "int1", "type": "Integrator", "params": {"initial": 0.0}}
//   ],
//   "edges": [
//     {"src": "c1", "dst": "g1", "dstInput": 0},
//     {"src": "g1", "dst": "sum1", "dstInput": 0}
//   ]
// }
inline Graph loadGraphFromJson(const json& j) {
    Graph g;
    for (auto& jb : j.at("blocks")) {
        Block b;
        b.id = jb.at("id").get<std::string>();
        b.type = parseBlockType(jb.at("type").get<std::string>());
        if (jb.contains("params")) {
            for (auto& [k, v] : jb["params"].items()) {
                b.params[k] = v.get<double>();
            }
        }
        if (jb.contains("signs")) {
            for (auto& s : jb["signs"]) b.inputSigns.push_back(s.get<std::string>());
        }
        g.addBlock(b);
    }
    for (auto& je : j.at("edges")) {
        g.addEdge(je.at("src").get<std::string>(),
                  je.at("dst").get<std::string>(),
                  je.value("dstInput", 0));
    }
    return g;
}

inline Graph loadGraphFromFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open file: " + path);
    json j;
    f >> j;
    return loadGraphFromJson(j);
}

} // namespace simmini
