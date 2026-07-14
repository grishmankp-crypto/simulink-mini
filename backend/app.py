"""
FastAPI backend for simulink-mini.

Wraps the pybind11 module (simmini_py, built from the C++ engine in
../include and ../python/bindings.cpp) with a REST + WebSocket API for the
Next.js block editor.

Endpoints:
  GET  /health           - liveness check
  GET  /blocks            - block-type metadata for the editor's palette
  POST /simulate          - run a full simulation, return the trajectory
  POST /tune               - PSO auto-tune a PID block's gains
  WS   /ws/simulate        - stream the simulation step-by-step, live

Run with (from the backend/ directory, after building the C++ project):
  uvicorn app:app --reload --port 8000
"""
import os
import sys
import json
import asyncio
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

# --- Locate the compiled pybind11 extension module ---
# The C++ project is built with CMake into ../build (see README/CMakeLists.txt
# at the project root). We add that directory to sys.path so `import
# simmini_py` finds the compiled .so without needing it installed as a
# package. Override with the SIMMINI_BUILD_DIR env var if your build lives
# somewhere else.
_BUILD_DIR = os.environ.get(
    "SIMMINI_BUILD_DIR",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "build")),
)
sys.path.insert(0, _BUILD_DIR)

try:
    import simmini_py as sm
except ImportError as e:
    raise ImportError(
        f"Could not import simmini_py from '{_BUILD_DIR}'. "
        "Build the C++ project first: mkdir build && cd build && cmake .. && make. "
        f"Original error: {e}"
    ) from e


app = FastAPI(title="simulink-mini API", version="0.4.0")

# Dev-friendly CORS: the Next.js dev server runs on a different port
# (localhost:3000) than this API (localhost:8000), so the browser needs
# CORS to allow it. Tighten allow_origins before deploying anywhere public.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---------------------------------------------------------------------------
# Request/response schemas
# ---------------------------------------------------------------------------
class DiagramBlock(BaseModel):
    id: str
    type: str
    params: Optional[Dict[str, float]] = None
    signs: Optional[List[str]] = None


class DiagramEdge(BaseModel):
    src: str
    dst: str
    dstInput: int = 0


class Diagram(BaseModel):
    blocks: List[DiagramBlock]
    edges: List[DiagramEdge]


class SimulateRequest(BaseModel):
    diagram: Diagram
    dt: float = 0.005
    tEnd: float = 5.0
    max_samples: int = Field(default=1000, ge=2, le=20000)


class TuneRequest(BaseModel):
    diagram: Diagram
    pid_block_id: str
    error_block_id: str
    lower_bounds: List[float] = [0.0, 0.0, 0.0]
    upper_bounds: List[float] = [20.0, 50.0, 5.0]
    Tf: float = 0.05
    effort_lambda: float = 0.02
    dt: float = 0.005
    tEnd: float = 5.0
    num_particles: int = 25
    max_iters: int = 40


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _build_graph_and_order(diagram: Diagram):
    """Parses the diagram into a Graph and computes execution order.
    Raises HTTPException(422) with the exact algebraic-loop cycle on failure,
    so the frontend can show a real, actionable error -- not a generic 500.
    """
    diagram_json = json.dumps(diagram.model_dump(exclude_none=True))
    try:
        g = sm.load_graph_from_json(diagram_json)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid diagram: {e}")
    try:
        order = sm.Scheduler(g).compute_execution_order()
    except sm.AlgebraicLoopError as e:
        raise HTTPException(status_code=422, detail=f"Algebraic loop detected: {e}")
    return g, order


def _run_trajectory(g, order, dt: float, tEnd: float, max_samples: int):
    sim = sm.Simulation(g, order)
    steps = int(tEnd / dt)
    stride = max(1, (steps + 1) // max_samples)

    block_ids = g.block_ids()
    times: List[float] = []
    traces: Dict[str, List[float]] = {bid: [] for bid in block_ids}

    for i in range(steps + 1):
        if i % stride == 0:
            outputs = sim.evaluate_network(sim.state())
            times.append(i * dt)
            for bid in block_ids:
                traces[bid].append(outputs.get(bid, 0.0))
        if i < steps:
            sim.step_rk4(dt)

    return times, traces


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------
@app.get("/health")
def health():
    return {"status": "ok", "module": "simmini_py loaded"}


@app.get("/blocks")
def list_blocks():
    """Block-type metadata the Next.js editor uses to build its palette."""
    return {
        "blocks": [
            {"type": "Constant", "label": "Constant", "params": ["value"], "numInputs": 0},
            {"type": "Gain", "label": "Gain", "params": ["k"], "numInputs": 1},
            {"type": "Sum", "label": "Sum", "params": [], "hasSigns": True, "numInputs": 2},
            {"type": "Product", "label": "Product", "params": [], "numInputs": 2},
            {"type": "Integrator", "label": "Integrator", "params": ["initial"], "numInputs": 1},
            {"type": "PID", "label": "PID", "params": ["Kp", "Ki", "Kd", "Tf"], "numInputs": 1},
            {"type": "Scope", "label": "Scope", "params": [], "numInputs": 1},
        ]
    }


@app.post("/simulate")
def simulate(req: SimulateRequest):
    g, order = _build_graph_and_order(req.diagram)
    times, traces = _run_trajectory(g, order, req.dt, req.tEnd, req.max_samples)
    return {"order": order, "times": times, "traces": traces}


@app.post("/tune")
def tune(req: TuneRequest):
    g, order = _build_graph_and_order(req.diagram)

    pid_block = next((b for b in req.diagram.blocks if b.id == req.pid_block_id), None)
    if pid_block is None:
        raise HTTPException(status_code=400, detail=f"pid_block_id '{req.pid_block_id}' not found in diagram")
    if req.error_block_id not in g.block_ids():
        raise HTTPException(status_code=400, detail=f"error_block_id '{req.error_block_id}' not found in diagram")

    p = pid_block.params or {}
    base_Kp, base_Ki, base_Kd = p.get("Kp", 1.0), p.get("Ki", 0.0), p.get("Kd", 0.0)

    def eval_cost(Kp, Ki, Kd):
        ise, effort, final_error, diverged = sm.simulate_closed_loop_cost(
            g, order, req.pid_block_id, req.error_block_id,
            Kp, Ki, Kd, req.Tf, req.dt, req.tEnd,
        )
        cost = 1e12 if diverged else ise + req.effort_lambda * effort
        return ise, effort, final_error, diverged, cost

    base_ise, base_effort, base_final_err, base_div, base_cost = eval_cost(base_Kp, base_Ki, base_Kd)

    def cost_fn(params):
        return eval_cost(params[0], params[1], params[2])[4]

    pso = sm.PSO(req.num_particles, req.lower_bounds, req.upper_bounds)
    result = pso.optimize(cost_fn, req.max_iters)

    tKp, tKi, tKd = result.best_params
    t_ise, t_effort, t_final_err, t_div, t_cost = eval_cost(tKp, tKi, tKd)

    pinned = any(
        abs(result.best_params[d] - req.lower_bounds[d]) < 1e-6
        or abs(result.best_params[d] - req.upper_bounds[d]) < 1e-6
        for d in range(3)
    )

    return {
        "baseline": {
            "Kp": base_Kp, "Ki": base_Ki, "Kd": base_Kd,
            "ise": base_ise, "effort": base_effort, "cost": base_cost,
            "final_error": base_final_err, "diverged": base_div,
        },
        "tuned": {
            "Kp": tKp, "Ki": tKi, "Kd": tKd,
            "ise": t_ise, "effort": t_effort, "cost": t_cost,
            "final_error": t_final_err, "diverged": t_div,
        },
        "improvement_factor": (base_cost / t_cost) if t_cost > 0 else None,
        "cost_history": result.cost_history,
        "gain_pinned_to_bound": pinned,
    }


@app.websocket("/ws/simulate")
async def ws_simulate(websocket: WebSocket):
    """Streams the simulation step-by-step as it computes, rather than
    waiting for the whole run and returning one blob (what /simulate does).
    Client sends one JSON message matching SimulateRequest to kick it off.
    """
    await websocket.accept()
    try:
        raw = await websocket.receive_json()
        req = SimulateRequest(**raw)

        diagram_json = json.dumps(req.diagram.model_dump(exclude_none=True))
        try:
            g = sm.load_graph_from_json(diagram_json)
            order = sm.Scheduler(g).compute_execution_order()
        except sm.AlgebraicLoopError as e:
            await websocket.send_json({"error": f"Algebraic loop detected: {e}"})
            return
        except Exception as e:
            await websocket.send_json({"error": f"Invalid diagram: {e}"})
            return

        sim = sm.Simulation(g, order)
        steps = int(req.tEnd / req.dt)
        stride = max(1, (steps + 1) // min(req.max_samples, 500))

        for i in range(steps + 1):
            if i % stride == 0:
                outputs = sim.evaluate_network(sim.state())
                await websocket.send_json({"t": i * req.dt, "outputs": outputs})
                await asyncio.sleep(0)  # yield control so the event loop can flush the frame
            if i < steps:
                sim.step_rk4(req.dt)

        await websocket.send_json({"done": True})
    except WebSocketDisconnect:
        pass
    except Exception as e:
        try:
            await websocket.send_json({"error": str(e)})
        except Exception:
            pass
    finally:
        try:
            await websocket.close()
        except Exception:
            pass
