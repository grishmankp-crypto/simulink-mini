#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>
#include <optional>
#include <sstream>

namespace simmini {

// ---------------------------------------------------------------------------
// Block types supported by the diagram compiler.
// Each type has a well-defined "memory" property: does it have internal state
// that breaks an algebraic loop (Integrator, UnitDelay), or is it purely
// combinational / feedthrough (Gain, Sum, Product) meaning its output at
// time t depends instantaneously on its input at time t?
// This distinction is the entire basis of algebraic loop detection.
// ---------------------------------------------------------------------------
enum class BlockType {
    Constant,        // no inputs, 1 output. Feedthrough: N/A (no inputs)
    Gain,            // y = k * u.                  FEEDTHROUGH (direct)
    Sum,             // y = sum(inputs, signs).      FEEDTHROUGH (direct)
    Product,         // y = prod(inputs).            FEEDTHROUGH (direct)
    TransferFunctionFirstOrder, // 1/(s*tau+1) style, implemented via internal integrator
    Integrator,      // dy/dt = u, y = state.        HAS MEMORY (breaks loops)
    UnitDelay,       // y[k] = u[k-1].               HAS MEMORY (breaks loops)
    PID,             // PID controller.               HAS MEMORY (integral term)
    Scope            // sink, records signal.         FEEDTHROUGH (pass-through, no output used downstream)
};

inline bool hasMemory(BlockType t) {
    switch (t) {
        case BlockType::Integrator:
        case BlockType::UnitDelay:
        case BlockType::PID:
        case BlockType::TransferFunctionFirstOrder:
            return true;
        default:
            return false;
    }
}

// A SEPARATE property from hasMemory(). hasMemory asks "does this block
// need an integrated state variable?" hasDirectFeedthrough asks "does this
// block's OUTPUT at time t depend on its INPUT at time t?" These are not
// opposites for every block: a plain Integrator has memory and NO direct
// feedthrough (clean split), but a PID controller has BOTH — its integral
// term is stateful, but its proportional term (Kp*e) passes through
// instantaneously. This is exactly why a cycle through a PID block is NOT
// automatically safe the way a cycle through a bare Integrator is: the
// Kp path alone can still form a genuine algebraic loop. The scheduler
// uses THIS property (not hasMemory) to decide which edges are real
// same-instant dependencies.
inline bool hasDirectFeedthrough(BlockType t) {
    switch (t) {
        case BlockType::Integrator:
        case BlockType::UnitDelay:
        case BlockType::TransferFunctionFirstOrder:
            return false; // output depends only on internal state, not current input
        default:
            return true;  // Constant, Gain, Sum, Product, PID, Scope
    }
}

inline const char* blockTypeName(BlockType t) {
    switch (t) {
        case BlockType::Constant: return "Constant";
        case BlockType::Gain: return "Gain";
        case BlockType::Sum: return "Sum";
        case BlockType::Product: return "Product";
        case BlockType::TransferFunctionFirstOrder: return "TransferFunctionFirstOrder";
        case BlockType::Integrator: return "Integrator";
        case BlockType::UnitDelay: return "UnitDelay";
        case BlockType::PID: return "PID";
        case BlockType::Scope: return "Scope";
    }
    return "Unknown";
}

struct Block {
    std::string id;                 // unique node id, e.g. "gain1"
    BlockType type;
    std::unordered_map<std::string, double> params; // e.g. {"k": 2.0} for Gain
    std::vector<std::string> inputSigns;             // for Sum: "+","-" per input, aligned with edge arrival order
};

// A directed edge: src block's output feeds dst block's input.
// inputIndex records which input slot of dst this edge fills (order matters
// for Sum/Product/PID, which have multiple, non-commutative-in-meaning inputs).
struct Edge {
    std::string src;
    std::string dst;
    int dstInputIndex;
};

class Graph {
public:
    void addBlock(const Block& b) {
        if (blocks_.count(b.id)) {
            throw std::runtime_error("Duplicate block id: " + b.id);
        }
        blocks_[b.id] = b;
        order_.push_back(b.id);
        adjOut_[b.id] = {};
        adjIn_[b.id] = {};
    }

    void addEdge(const std::string& src, const std::string& dst, int dstInputIndex) {
        if (!blocks_.count(src)) throw std::runtime_error("Unknown src block: " + src);
        if (!blocks_.count(dst)) throw std::runtime_error("Unknown dst block: " + dst);
        edges_.push_back({src, dst, dstInputIndex});
        adjOut_[src].push_back({dst, dstInputIndex});
        adjIn_[dst].push_back({src, dstInputIndex});
    }

    const Block& block(const std::string& id) const {
        auto it = blocks_.find(id);
        if (it == blocks_.end()) throw std::runtime_error("Unknown block: " + id);
        return it->second;
    }

    // Non-const accessor: lets the tuning endpoint update a block's params
    // (e.g. PID gains) in place between PSO fitness evaluations, without
    // rebuilding the whole graph -- topology (edges) never changes during
    // tuning, only parameter VALUES, so this is both correct and much
    // cheaper than reparsing JSON on every candidate.
    Block& mutableBlock(const std::string& id) {
        auto it = blocks_.find(id);
        if (it == blocks_.end()) throw std::runtime_error("Unknown block: " + id);
        return it->second;
    }

    const std::vector<std::string>& blockIds() const { return order_; }

    const std::vector<std::pair<std::string,int>>& outEdges(const std::string& id) const {
        return adjOut_.at(id);
    }
    const std::vector<std::pair<std::string,int>>& inEdges(const std::string& id) const {
        return adjIn_.at(id);
    }

    size_t numBlocks() const { return blocks_.size(); }

private:
    std::unordered_map<std::string, Block> blocks_;
    std::vector<std::string> order_; // insertion order, for deterministic iteration
    std::vector<Edge> edges_;
    std::unordered_map<std::string, std::vector<std::pair<std::string,int>>> adjOut_;
    std::unordered_map<std::string, std::vector<std::pair<std::string,int>>> adjIn_;
};

} // namespace simmini
