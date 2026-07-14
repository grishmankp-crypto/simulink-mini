# Project Handoff: simulink-mini (MathWorks interview project) — COMPLETE

All four planned weeks are done. Paste this document as your first message
in a new Claude conversation, with `simulink-mini-week4-final.zip` uploaded,
to resume for interview prep, polish, or extensions.

---

## 1. Context

- Integrated Dual Degree CSE student, IIITDM Kancheepuram (2028), CGPA 7.48
  — portfolio needs to do the work an ATS filter won't.
- Built specifically for a MathWorks interview.
- Machine: HP Pavilion, i5-11300H, 8GB RAM, GTX 1650. Confirmed sufficient
  throughout — this is graph algorithms + light compilation + a small web
  stack, no GPU/heavy-RAM requirement anywhere.
- **Why this project**: MathWorks' actual differentiator is Simulink +
  Embedded Coder — a visual block-diagram tool whose compiler decides
  whether a diagram is valid and turns it into deployable C code. This
  project is a simplified version of exactly that pipeline. A prior
  suggestion (generic numerical-expression-DSL/JIT) was explicitly
  rejected as a well-known CS-coursework pattern; **do not suggest
  reverting to something generic** if this conversation continues.

## 2. What's built — all four weeks, complete and verified

- **Week 1**: Graph/Block model, Kahn's-algorithm scheduler, algebraic
  loop detection via DFS cycle extraction.
- **Week 2**: RK4 solver (validated to 2.4e-12 vs analytical `cos(2t)`
  over 10s) + code generator (standalone, unrolled C++, diffed against
  the interpreter to 5e-11).
- **Week 3**: PID block (2 state slots: integral + filtered derivative),
  validated to 6.1e-8 vs a derived closed-form solution. **Fixed a real
  bug** flagged in the Week 1/2 README: split `hasMemory()` from a new
  `hasDirectFeedthrough()`, since PID has both memory AND direct
  feedthrough — a loop through PID's Kp path is now correctly rejected.
  PSO auto-tuning (reimplemented from a prior Python optimization
  project) with a genuine finding: plain ISE cost pinned all gains to
  search bounds; fixed with a control-effort penalty term.
- **Week 4**: pybind11 bindings (`python/bindings.cpp`) exposing the
  entire C++ engine to Python; FastAPI backend (`backend/app.py`) with
  `/health`, `/blocks`, `/simulate`, `/tune`, and a `/ws/simulate`
  WebSocket; Next.js + ReactFlow drag-and-drop block editor
  (`frontend/`) with a live recharts scope trace and a PSO tuning panel.

## 3. The full architecture (all four weeks)

```
[Next.js editor, frontend/]  ReactFlow canvas, palette, live chart
        | JSON: {"blocks":[...], "edges":[...]}
        v
[FastAPI backend, backend/app.py]
  GET /health, GET /blocks, POST /simulate, POST /tune, WS /ws/simulate
        v
[simmini_py, python/bindings.cpp — pybind11]
  Exposes Graph, Scheduler, Simulation, CodeGen, PSO, and a fast
  simulate_closed_loop_cost() helper so PSO's inner loop stays at C++
  speed (one Python<->C++ call per fitness evaluation, not per timestep).
        v
[C++ engine, include/*.hpp]
  Graph.hpp      — Block/Edge model; hasMemory() vs hasDirectFeedthrough()
                    are SEPARATE properties (PID has both).
  Scheduler.hpp  — builds the feedthrough dependency graph, Kahn's topo
                    sort, DFS cycle extraction on failure.
  Simulation.hpp — RK4 interpreter, N-state-slots-per-block (Integrator:1,
                    PID:2 — integral + filtered derivative).
  CodeGen.hpp    — unrolls the scheduled graph into standalone,
                    dependency-free C++ (no map lookups, no type dispatch
                    at simulation time).
  PSO.hpp        — standard particle-swarm optimizer.
```

## 4. Verified results — reproduce ALL of these before trusting this doc

```bash
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make
./test_scheduler      # 3 tests incl. the PID-loop fix, all PASS
./test_simulation     # RK4 vs analytical: 2.4e-12 error, PASS
./test_codegen        # codegen vs interpreter: 5.0e-11 diff, PASS
./test_pid            # PID vs closed-form: 6.1e-8 error, PASS
./test_codegen_pid    # PID codegen vs interpreter: 2.0e-12 diff, PASS
./test_pso_tune       # PSO tuning: 5.74x improvement, PASS

cd ../backend && pip install -r requirements.txt
python3 test_api.py   # 16 checks, ALL PASS (health, blocks, simulate,
                       # algebraic-loop-as-422, tune, WebSocket streaming
                       # incl. algebraic-loop-over-WS error path)
```

**Cross-layer consistency check** (this is the single strongest proof the
whole stack is genuinely wired together, not faked): the SAME closed-loop
PID tuning problem was solved four independent ways and produced
BIT-IDENTICAL results:
```
C++ standalone:        Kp=6.833004877799299, Ki=0.025762748246341723
Python bindings direct: Kp=6.833004877799299, Ki=0.025762748246341723
FastAPI TestClient:      Kp=6.833004877799299, Ki=0.025762748246341723
Real browser (Playwright, live HTTP+WS): Kp=6.833, Ki=0.026 (displayed rounded)
```

A full Playwright browser test (`e2e_test.py`, run against a live
`uvicorn` + `next start`) drives the ACTUAL UI: loads the app, confirms
"backend: connected", loads the PID+plant example onto the canvas, runs
a batch simulation (checks a chart renders, no error banner), streams a
live simulation via WebSocket, opens the Tune panel, runs PSO through the
real UI, and checks the exact tuned gains plus zero browser console
errors. **All 11 checks pass.** Screenshots were also visually reviewed
(diagram renders correctly, chart shows the expected step response,
tune panel shows the right numbers).

## 5. A real bug the multi-layer testing caught (Week 4)

The WebSocket error path initially used `asyncio.ensure_future()` to send
an error message from a non-async context — this schedules a coroutine
without awaiting it, so `finally: await websocket.close()` could run
before the message actually sent. Fixed by making the error-reporting
path properly `async`/`await`. Caught by testing the actual
broken-diagram-over-WebSocket path, not just the happy path — a good
example of why testing failure modes matters as much as testing success.

## 6. Known simplifications across all four weeks (see README.md for the
full, organized version — this is a summary)

- `UnitDelay`/`TransferFunctionFirstOrder`: declared, not implemented
  (clear error, not silent misbehavior).
- Loop detection reports only the first cycle found, not all cycles.
- PSO's effort-penalty weight and swarm size/iterations were chosen by
  inspection/generously, not principled tuning — fine at this scale.
- CORS is wide open (`*`) for local dev convenience — tighten before any
  public deployment.
- Editor's Sum/Product blocks are fixed at 2 inputs (matches backend
  default); variable arity would need a small UI affordance.
- `/tune` searches a fixed 3D box with `Tf` fixed separately; both are
  easy to expose in the UI but aren't yet.

## 7. How to run everything locally

```bash
# Terminal 1: build + backend
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make && cd ..
cd backend && pip install -r requirements.txt && uvicorn app:app --reload --port 8000

# Terminal 2: frontend
cd frontend && npm install && npm run dev
# open http://localhost:3000, click "Load PID+Plant example"
```

## 8. Interview-ready talking points (the actual substance, not the plumbing)

1. **Loop detection = scheduling**: both come from the same
   feedthrough-dependency graph, computed once via Kahn's algorithm.
2. **PID has two properties, not one**: `hasMemory` (needs state) and
   `hasDirectFeedthrough` (output depends on current input) — conflating
   them is a real bug that was caught and fixed, with a test proving it.
3. **RK4 on a coupled system means 4 FULL network evaluations per
   step**, not per-state-independent integration — a common
   misunderstanding worth being able to explain clearly.
4. **Codegen vs interpreter**: the generated code has zero runtime type
   dispatch — every block became a named `double` and a straight-line
   assignment, diffed against the interpreter to machine precision.
5. **The PSO gain-pinning story**: plain ISE has no reason to stop
   increasing gains; a control-effort penalty is standard real practice,
   not a workaround — and finding this was a genuine "the optimizer told
   me my objective was wrong" moment.
6. **The full-stack bridge is bit-exact**: same tuning problem, same
   answer, four different ways of asking (C++, Python, HTTP API,
   browser) — proof the layers aren't secretly diverging.

## 9. If asked to extend further

See "Beyond the original 4-week scope" in README.md — UnitDelay/TF
implementation, all-cycles reporting, UI-exposed PSO bounds, a
"download generated C++" button (the binding already exists, just needs
a frontend button), and tightening CORS for any real deployment.
