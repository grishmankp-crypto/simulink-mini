#pragma once
#include "Graph.hpp"
#include <vector>
#include <queue>
#include <algorithm>
#include <functional>

namespace simmini {

struct AlgebraicLoopError : public std::runtime_error {
    std::vector<std::string> cycle; // the offending cycle, in order
    explicit AlgebraicLoopError(std::vector<std::string> c)
        : std::runtime_error(buildMsg(c)), cycle(std::move(c)) {}

    static std::string buildMsg(const std::vector<std::string>& c) {
        std::ostringstream oss;
        oss << "Algebraic loop detected: ";
        for (size_t i = 0; i < c.size(); ++i) {
            oss << c[i];
            if (i + 1 < c.size()) oss << " -> ";
        }
        oss << " -> " << c.front() << " (no block with memory breaks this cycle)";
        return oss.str();
    }
};

// ---------------------------------------------------------------------------
// Scheduler builds the "instantaneous dependency graph": an edge u -> v
// exists iff block v's OUTPUT at time t depends directly on block u's output
// at time t (i.e. v has no internal memory / is a feedthrough block).
// If v has memory (Integrator, UnitDelay, PID, TF), its output at time t
// depends only on its internal state, NOT on u's current output, so the
// edge u -> v is dropped for scheduling purposes even though it exists in
// the wiring diagram. This is exactly why memory blocks "break" algebraic
// loops in real block-diagram tools.
// ---------------------------------------------------------------------------
class Scheduler {
public:
    explicit Scheduler(const Graph& g) : g_(g) { buildDependencyGraph(); }

    // Throws AlgebraicLoopError if a cycle exists in the feedthrough graph.
    // Returns a valid topological execution order otherwise (Kahn's algorithm,
    // O(V + E)).
    std::vector<std::string> computeExecutionOrder() {
        std::unordered_map<std::string, int> indeg;
        for (auto& id : g_.blockIds()) indeg[id] = 0;
        for (auto& [u, outs] : dep_) {
            for (auto& v : outs) indeg[v]++;
        }

        std::queue<std::string> q;
        // Deterministic order: push in the graph's original insertion order.
        for (auto& id : g_.blockIds()) {
            if (indeg[id] == 0) q.push(id);
        }

        std::vector<std::string> order;
        order.reserve(g_.numBlocks());

        while (!q.empty()) {
            std::string u = q.front(); q.pop();
            order.push_back(u);
            for (auto& v : dep_[u]) {
                if (--indeg[v] == 0) q.push(v);
            }
        }

        if (order.size() != g_.numBlocks()) {
            // Cycle exists among the blocks not placed in `order`.
            auto cyc = findCycle(indeg);
            throw AlgebraicLoopError(cyc);
        }
        return order;
    }

    // Exposed for testing / diagnostics: which blocks have memory (state).
    const std::unordered_map<std::string, std::vector<std::string>>& dependencyGraph() const {
        return dep_;
    }

private:
    const Graph& g_;
    // dep_[u] = list of v such that v depends directly (feedthrough) on u
    std::unordered_map<std::string, std::vector<std::string>> dep_;

    void buildDependencyGraph() {
        for (auto& id : g_.blockIds()) dep_[id] = {};
        for (auto& u : g_.blockIds()) {
            for (auto& [v, inputIdx] : g_.outEdges(u)) {
                const Block& dstBlock = g_.block(v);
                if (hasDirectFeedthrough(dstBlock.type)) {
                    dep_[u].push_back(v);
                }
                // If dstBlock has NO direct feedthrough (Integrator, UnitDelay,
                // TransferFunctionFirstOrder), we deliberately do NOT add this
                // edge: v's output this step does not require u's output this
                // step, since v's output = f(internal state only). Note this is
                // now DIFFERENT from "not hasMemory" — PID has memory AND direct
                // feedthrough, so edges into a PID block DO count here.
            }
        }
    }

    // DFS-based cycle extraction, used only on failure to give a readable
    // error message pointing at the exact offending loop.
    std::vector<std::string> findCycle(const std::unordered_map<std::string,int>& indegAfterKahn) {
        // Any node with indeg > 0 after Kahn's algorithm terminates is part of
        // (or downstream of) a cycle in the feedthrough graph.
        std::unordered_set<std::string> remaining;
        for (auto& [id, d] : indegAfterKahn) if (d > 0) remaining.insert(id);

        std::unordered_map<std::string, int> color; // 0=white,1=gray,2=black
        std::vector<std::string> path;
        std::vector<std::string> cyclePath;

        std::function<bool(const std::string&)> dfs = [&](const std::string& u) -> bool {
            color[u] = 1;
            path.push_back(u);
            for (auto& v : dep_[u]) {
                if (!remaining.count(v)) continue;
                if (color[v] == 1) {
                    // Found the back-edge closing the cycle; extract it.
                    auto it = std::find(path.begin(), path.end(), v);
                    cyclePath.assign(it, path.end());
                    return true;
                }
                if (color[v] == 0) {
                    if (dfs(v)) return true;
                }
            }
            path.pop_back();
            color[u] = 2;
            return false;
        };

        for (auto& id : remaining) {
            if (color[id] == 0) {
                if (dfs(id)) return cyclePath;
            }
        }
        return {"<unknown cycle>"};
    }
};

} // namespace simmini
