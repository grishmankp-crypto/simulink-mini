# simulink-mini — A Block-Diagram Compiler with Algebraic Loop Detection

A simplified version of the compilation pipeline behind Simulink + Embedded
Coder: take a visual block diagram (JSON graph), detect whether it is
physically simulatable, compute a valid execution order, generate
standalone C++ code, and run a fixed-step RK4 numerical simulation.

**Week 1** built the graph model, scheduler, and algebraic loop detection.
**Week 2** added the numerical solver and code generator. **Week 3** added
PID control and PSO auto-tuning. **Week 4** bridges all of it to a real
Next.js editor via FastAPI and pybind11 — the full project is complete
end-to-end.

## Why this project (not a generic one)

MathWorks' core product differentiator isn't "a numerical language" — it's
Simulink: engineers wire up blocks visually, and Embedded Coder compiles
that diagram directly into deployable C code for real hardware (cars,
satellites, medical devices). The hard, interesting part of that pipeline
is deciding whether a wiring diagram is even *valid* — and if it's not,
explaining exactly why. That's what this project builds first.

## The core problem: algebraic loops

Consider two diagrams:

**Valid**: `Constant -> Sum -> Gain -> Integrator -> (feeds back to Sum)`
This is a legal closed-loop system (e.g. a simple first-order controller).
Even though there's a cycle in the *wiring*, it's not a problem, because
the Integrator's output at time `t` depends only on its accumulated
internal state — not on its input at time `t`. The loop is "broken" by
memory.

**Invalid**: `Constant -> Sum -> Gain -> (feeds back to Sum)` with no
Integrator or delay anywhere in the loop. This is unsolvable: `Sum`'s
output depends instantaneously on `Gain`'s output, which depends
instantaneously on `Sum`'s output. There's no way to compute either value
without already knowing the other. This is a **genuine algebraic loop**
and must be rejected at compile time, not discovered as a runtime hang or
NaN cascade.

## The algorithm

1. **Classify every block** as either "feedthrough" (its output at time
   `t` is a pure function of its inputs at time `t` — e.g. Gain, Sum,
   Product) or "stateful" (its output at time `t` depends only on
   internal state accumulated up to `t` — e.g. Integrator, UnitDelay,
   PID). See `hasMemory()` in `include/Graph.hpp`.

2. **Build the instantaneous-dependency graph**: for every wiring edge
   `u -> v`, include the edge in the dependency graph *only if* `v` is a
   feedthrough block. If `v` has memory, drop the edge — `v`'s output
   this step doesn't actually require `u`'s output this step.
   See `Scheduler::buildDependencyGraph()` in `include/Scheduler.hpp`.

3. **Topologically sort** this reduced graph using Kahn's algorithm
   (BFS with in-degree counting, O(V+E)). If every node is placed, the
   returned order is a valid execution order for computing every block's
   output at a given timestep.

4. **If Kahn's algorithm stalls** (some nodes never reach in-degree 0),
   a cycle exists in the feedthrough graph — a real algebraic loop. A
   second DFS pass (`findCycle()`) extracts and reports the exact cycle,
   e.g. `sum1 -> gain1 -> sum1`, so the error is actionable, not just
   "compilation failed."

This means loop detection and execution scheduling are **the same
algorithm run once** — not two separate passes — which is both more
efficient and a cleaner thing to explain on a whiteboard.

## Verified behavior

Both test fixtures are checked into `tests/` and run automatically:

- `tests/valid_feedback_loop.json` → scheduler succeeds, produces order
  `ref -> int1 -> sum1 -> scope1 -> gain1`
- `tests/broken_algebraic_loop.json` → scheduler throws
  `AlgebraicLoopError` with message
  `Algebraic loop detected: sum1 -> gain1 -> sum1 (no block with memory breaks this cycle)`

```
$ mkdir build && cd build && cmake .. && make
$ ./test_scheduler
[Test 1] Valid feedback loop (Sum -> Gain -> Integrator -> Sum)
  PASS: execution order computed successfully: ref -> int1 -> sum1 -> scope1 -> gain1

[Test 2] Broken algebraic loop (Sum -> Gain -> Sum, no memory block)
  PASS: correctly rejected with message:
    Algebraic loop detected: sum1 -> gain1 -> sum1 (no block with memory breaks this cycle)

ALL TESTS PASSED
```

## Week 2 — Simulation engine + code generation

Two things were added, and each is validated against an independent
source of truth rather than just "it runs":

### 1. RK4 simulation engine (`include/Simulation.hpp`)

The execution order from Week 1 tells you *what order* to compute block
outputs in for a single instant. Turning that into a real simulation
means integrating each stateful block's derivative forward through time.
Classic 4th-order Runge-Kutta requires evaluating the **entire coupled
network four times per timestep** (at `t`, twice at `t + dt/2`, and at
`t + dt`) — not integrating each state independently, since a real
system's states feed back into each other's derivatives through the
feedthrough network.

**Validation**: `tests/shm_oscillator.json` models an undamped spring-mass
system (`x'' = -4x`), which has an exact closed-form solution
`x(t) = cos(2t)` for `x(0)=1, v(0)=0`. Running the compiled diagram
through RK4 for 10 simulated seconds at `dt=0.001` tracks the analytical
solution to a **maximum absolute error of 2.4×10⁻¹²** — i.e. machine
precision, not "close enough to eyeball." This is the actual proof the
solver is implemented correctly, not just non-crashing.

### 2. Code generator (`include/CodeGen.hpp`)

Given a compiled diagram, `CodeGen::generate()` emits a **standalone,
human-readable `.cpp` file** — every block's output unrolled into a
named local variable in execution order, with no runtime graph-walking
at all. This mirrors what a real tool like Simulink's Embedded Coder
does: compile the specific model into deployable code, not ship a
generic interpreter. See `examples/example_generated_shm_oscillator.cpp`
for real generated output from the oscillator diagram above.

**Validation**: `tests/test_codegen.cpp` generates the code, compiles it
as a **fully separate binary** with no dependency on this project's
headers, runs it, and diffs its output trajectory against the
interpreter's trajectory sample-by-sample. Result: **max diff of
5.0×10⁻¹¹** across 10,001 timesteps — the generated code and the
interpreter agree to numerical noise, proving the codegen path is
semantically identical to the validated interpreter, not just
syntactically plausible.

```
$ ./test_simulation
Max absolute error over t in [0, 10.000000]: 2.435219e-12
PASS: RK4 simulation matches analytical solution (error < 1.000000e-06)

$ ./test_codegen
Generated generated_model.cpp (1777 bytes)
Max diff between interpreter and generated-code trajectories: 4.999501e-11
PASS: generated code and interpreter agree (diff < 1.000000e-09)
```

## Week 3 — PID control + PSO auto-tuning

### Fixing the known simplification from Weeks 1–2

The Week 1/2 README flagged a real gap: PID was modeled as purely
stateful, so a loop through PID's proportional path could slip past
loop detection undetected. Fixed properly: `hasMemory()` and a new,
SEPARATE `hasDirectFeedthrough()` are no longer conflated. A plain
Integrator has memory and no direct feedthrough (clean split); a PID
has **both** — its integral term is stateful, but `Kp*e` passes through
instantaneously. The scheduler now builds its dependency graph from
`hasDirectFeedthrough()`, not `hasMemory()`. Proof this isn't just a
paper fix — a new test fixture (`sum1 -> pid1 -> sum1`, no other memory
block anywhere) is now correctly rejected:

```
[Test 3] Algebraic loop through PID's proportional (Kp) path
  PASS: correctly rejected with message:
    Algebraic loop detected: sum1 -> pid1 -> sum1 (no block with memory breaks this cycle)
```

### PID block: two state slots, not one

A real PID never differentiates its error signal directly — pure
differentiation amplifies sensor noise without bound. Industrial PID
implementations run the error through a first-order low-pass filter and
use *the filter's own derivative* as the D-term estimate. That means PID
needs two integrated states, not one:

- `dI/dt = e` — the integral accumulator
- `dDf/dt = (e - Df) / Tf` — the filter state, where `Tf` is the filter
  time constant

Output: `y = Kp*e + Ki*I + Kd*(e - Df)/Tf`. `Simulation.hpp` was
generalized from "1 state slot per stateful block" to "N slots per
block" (`numStateSlots(BlockType)`) to support this, and `CodeGen.hpp`
was updated to emit the same two-state unrolled code.

**Validation, not just "it runs":** for a constant input `e0` starting
from zero state, this ODE system has an exact closed-form solution:
`y(t) = Kp*e0 + Ki*e0*t + (Kd*e0/Tf)*exp(-t/Tf)`. Feeding a step
directly into an isolated PID block and comparing against this formula:

```
Max absolute error over t in [0, 2.0]: 6.113710e-08
PASS: PID block matches closed-form solution (error < 1.000000e-04)
```

And the codegen path for PID is diffed against the interpreter exactly
as in Week 2 (two independent code paths, same result):

```
Max diff between interpreter and generated-code trajectories: 2.012612e-12
PASS: PID generated code and interpreter agree (diff < 1.000000e-09)
```

### PSO auto-tuning, reusing prior optimization work

`include/PSO.hpp` reimplements the PSO algorithm from an earlier project
(structural design optimization) as a standalone C++ component, coupled
directly to the RK4 solver: the swarm's cost function *is* "simulate the
closed-loop system and measure tracking error," which has no clean
analytical gradient with respect to `(Kp, Ki, Kd)` — exactly the kind of
problem PSO is suited for.

Test system: a PID controlling a bare Integrator plant (`dy/dt = u`) in
unity feedback, tracking a step reference. The plant has **no natural
damping of its own** — any good settling behavior has to come entirely
from the controller, so a badly-tuned PID will visibly ring or diverge,
not just be "a bit slow." This is an honest test, not a rigged one.

**A real finding, left in rather than smoothed over:** the first version
of the cost function was plain ISE (integral of squared error). PSO
correctly found that minimizing ISE alone has no reason to ever stop
increasing the gains — a more aggressive controller is always at least
as fast, right up until instability — so it pinned Kp, Ki, and Kd all to
the upper edge of the search box. That's not a bug, it's PSO doing
exactly what it was told; the objective itself was underspecified. Fixed
by adding a control-effort penalty (`cost = ISE + λ·∫u²dt`), which is
standard practice in real controller tuning, not a workaround — an
"optimal" controller that demands unbounded actuator effort isn't
realizable on real hardware. After the fix:

```
Baseline gains: Kp=0.5 Ki=0.1 Kd=0.0
  ISE = 8.34e-01, control effort = 3.46e-01, combined cost = 8.41e-01, final error = -1.42e-01

PSO-tuned gains: Kp=6.833005 Ki=0.025763 Kd=0.000000
  ISE = 7.57e-02, control effort = 3.54e+00, combined cost = 1.46e-01, final error = -5.42e-04

Improvement factor (baseline cost / tuned cost): 5.74x
PASS: PSO found gains that are stable, converge to the setpoint, and substantially beat the baseline.
```

## Week 4 — pybind11 bridge, FastAPI/WebSocket backend, Next.js editor

The full path is now real and tested end-to-end: **browser → Next.js →
FastAPI → pybind11 → the same C++ engine validated in Weeks 1–3 → back to
the browser.** Nothing here is a separate reimplementation — the FastAPI
layer calls the exact `Graph`/`Scheduler`/`Simulation`/`CodeGen`/`PSO`
classes that the C++ test suite already proved correct.

### Architecture

```
[Next.js editor (frontend/)]
  ReactFlow canvas: drag blocks from a palette, wire them by dragging
  between input/output handles, edit params inline on each node.
        |  POST/WS JSON: {"blocks":[...], "edges":[...]}
        v
[FastAPI backend (backend/app.py)]
  GET  /health      - liveness + confirms the C++ module loaded
  GET  /blocks       - block-type metadata for the editor's palette
  POST /simulate      - full trajectory, or a 422 with the EXACT algebraic-
                         loop cycle if the diagram is invalid
  POST /tune            - PSO auto-tunes a PID block's gains
  WS   /ws/simulate      - streams the simulation step-by-step, live
        |  calls into
        v
[simmini_py (python/bindings.cpp, pybind11)]
  Exposes Graph, Scheduler, Simulation, CodeGen, PSO directly. Also exposes
  simulate_closed_loop_cost(), a fast C++ function that mutates a PID
  block's gains in place and runs a full RK4 sim -- so PSO's inner loop
  never crosses the Python<->C++ boundary per-timestep, only once per
  fitness evaluation (~1000 calls, not ~1,000,000).
        |  same compiled .so, same classes as
        v
[The C++ engine -- include/*.hpp, validated in Weeks 1-3]
```

### Proof this is a real bridge, not a facade

Every layer was checked against the SAME closed-loop tuning result, run
independently four different ways:

```
C++ standalone (test_pso_tune.cpp):    Kp=6.833004877799299, Ki=0.025762748246341723
Python bindings (direct call):          Kp=6.833004877799299, Ki=0.025762748246341723
FastAPI (TestClient, in-process):       Kp=6.833004877799299, Ki=0.025762748246341723
FastAPI (real HTTP + Playwright browser): Kp=6.833, Ki=0.026 (rounded for display)
```

Bit-exact agreement across all four confirms the Python bindings and the
API layer aren't silently diverging from the validated C++ core.

### What's tested, concretely

- `backend/test_api.py` -- 16 checks against the FastAPI app via
  `TestClient`: health, block metadata, valid simulation, a broken
  algebraic loop correctly returned as a structured 422 (not a 500), PSO
  tuning end-to-end, and full WebSocket streaming including the
  algebraic-loop error path over the socket.
- A live `uvicorn` server was also hit with real `curl` requests and a
  real `websockets` client connection (not just `TestClient`'s in-process
  transport) to confirm actual socket behavior.
- `e2e_test.py` -- a genuine Playwright-driven browser test against the
  live Next.js + FastAPI stack: loads the app, confirms the backend
  status badge shows "connected", loads the PID+plant example onto the
  canvas, runs a batch simulation, streams a live simulation, and runs
  PSO auto-tuning through the actual UI -- checking for the exact same
  tuned gains as the C++ test, zero error banners, and zero browser
  console errors. All 11 checks pass.

### A real bug the multi-layer testing caught

The WebSocket error-reporting path was initially written using
`asyncio.ensure_future()` to send an error message from a synchronous
helper function -- which schedules a coroutine without awaiting it, so
the `finally: await websocket.close()` could run before the error message
actually got sent. Fixed by making the error path `async` and `await`ing
`send_json()` directly. Caught by testing the actual broken-loop-over-
WebSocket path, not just the happy path.

### Running it locally

```bash
# 1. Build the C++ engine + Python bindings (from the project root)
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make && cd ..

# 2. Start the backend (from backend/)
cd backend
pip install -r requirements.txt
uvicorn app:app --reload --port 8000
# leave this running; open a new terminal for step 3

# 3. Start the frontend (from frontend/)
cd frontend
npm install
npm run dev
# open http://localhost:3000
```

Click "Load PID+Plant example," then "Run simulation" to see the step
response, or "Tune PID" to auto-tune the gains live against the C++ RK4
solver.

### Known simplifications (Week 4)

- CORS is wide open (`allow_origins=["*"]`) for local development
  convenience. Tighten this before deploying anywhere public.
- The editor currently supports a fixed 2-input layout for Sum/Product
  blocks (matching the backend's default). Variable-arity sum/product
  blocks would need a small UI affordance to add/remove input handles.
- `/tune` always searches a fixed 3D box for `(Kp, Ki, Kd)` with a
  separately-fixed `Tf`; making all four tunable, or exposing the
  search bounds in the UI, is a natural next step, not a limitation of
  the underlying PSO/C++ code (which already accepts arbitrary bounds).

## Beyond the original 4-week scope (possible future extensions)

All four planned weeks are complete. If continuing beyond the interview
deadline, natural next steps would be:

- `UnitDelay` and `TransferFunctionFirstOrder` block implementations
  (currently declared but not implemented — see Known Simplifications).
- Report all cycles in a broken diagram, not just the first one DFS finds.
- Make the PID auto-tuner's search bounds and `Tf` editable from the UI,
  and expose the PSO convergence history (`cost_history`) as a live chart
  during tuning.
- A "download generated C++" button in the editor, wiring up the
  `generate_code()` binding that's already exposed but not yet used by
  the frontend.

## Known simplifications (be upfront about these in an interview)

- ~~`PID` is treated as a fully stateful block with no direct feedthrough~~
  — **fixed in Week 3**: see `hasDirectFeedthrough()` in `Graph.hpp`.
- Loop detection currently only reports one cycle per compile failure
  (the first one DFS finds), not all cycles in the graph.
- `UnitDelay` and `TransferFunctionFirstOrder` are declared in the type
  system but not yet implemented in `Simulation`/`CodeGen` — they throw
  a clear "not yet supported" error rather than silently misbehaving.
- The PSO cost function's control-effort weight (`λ = 0.02`) was chosen
  by inspection, not by a principled method (e.g. Bryson's rule for LQR
  weight selection). Worth mentioning as a "what I'd improve" if asked —
  a more rigorous approach would non-dimensionalize the error and effort
  terms before combining them.
- PSO's swarm size (25) and iteration count (40) were not tuned for
  minimal wall-clock time; they were chosen generously since the cost
  function (a full RK4 simulation) is cheap enough here (~1000 steps) that
  it didn't matter. On a more expensive plant model this would need
  revisiting.

## Building the C++ engine + test suite

Requires CMake ≥ 3.16, a C++17 compiler, and `nlohmann-json3-dev`.
(For the full stack — backend + frontend — see "Running it locally" in
the Week 4 section above.)

```
apt-get install cmake nlohmann-json3-dev   # if not already installed
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
./test_scheduler      # Week 1: algebraic loop detection + scheduling
./test_simulation     # Week 2: RK4 solver validated against analytical solution
./test_codegen        # Week 2: codegen validated against the interpreter
./test_pid            # Week 3: PID block validated against closed-form solution
./test_codegen_pid    # Week 3: codegen for PID validated against the interpreter
./test_pso_tune       # Week 3: PSO auto-tuning of PID gains on a closed-loop system
```
