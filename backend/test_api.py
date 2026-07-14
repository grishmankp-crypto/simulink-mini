"""
End-to-end test of the FastAPI app using TestClient -- no live server needed,
but exercises the exact same request/response path a real client would hit,
including the WebSocket.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("SIMMINI_BUILD_DIR", os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "build")))

from fastapi.testclient import TestClient
from app import app

client = TestClient(app)

VALID_LOOP_DIAGRAM = {
    "blocks": [
        {"id": "ref", "type": "Constant", "params": {"value": 1.0}},
        {"id": "sum1", "type": "Sum", "signs": ["+", "-"]},
        {"id": "gain1", "type": "Gain", "params": {"k": 3.0}},
        {"id": "int1", "type": "Integrator", "params": {"initial": 0.0}},
        {"id": "scope1", "type": "Scope"},
    ],
    "edges": [
        {"src": "ref", "dst": "sum1", "dstInput": 0},
        {"src": "int1", "dst": "sum1", "dstInput": 1},
        {"src": "sum1", "dst": "gain1", "dstInput": 0},
        {"src": "gain1", "dst": "int1", "dstInput": 0},
        {"src": "int1", "dst": "scope1", "dstInput": 0},
    ],
}

BROKEN_LOOP_DIAGRAM = {
    "blocks": [
        {"id": "ref", "type": "Constant", "params": {"value": 1.0}},
        {"id": "sum1", "type": "Sum", "signs": ["+", "-"]},
        {"id": "gain1", "type": "Gain", "params": {"k": 2.0}},
    ],
    "edges": [
        {"src": "ref", "dst": "sum1", "dstInput": 0},
        {"src": "gain1", "dst": "sum1", "dstInput": 1},
        {"src": "sum1", "dst": "gain1", "dstInput": 0},
    ],
}

CLOSED_LOOP_PID_DIAGRAM = {
    "blocks": [
        {"id": "ref", "type": "Constant", "params": {"value": 1.0}},
        {"id": "sum_e", "type": "Sum", "signs": ["+", "-"]},
        {"id": "pid1", "type": "PID", "params": {"Kp": 0.5, "Ki": 0.1, "Kd": 0.0, "Tf": 0.05}},
        {"id": "plant", "type": "Integrator", "params": {"initial": 0.0}},
        {"id": "scope", "type": "Scope"},
    ],
    "edges": [
        {"src": "ref", "dst": "sum_e", "dstInput": 0},
        {"src": "plant", "dst": "sum_e", "dstInput": 1},
        {"src": "sum_e", "dst": "pid1", "dstInput": 0},
        {"src": "pid1", "dst": "plant", "dstInput": 0},
        {"src": "plant", "dst": "scope", "dstInput": 0},
    ],
}

failures = 0

def check(name, cond, detail=""):
    global failures
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name}" + (f" -- {detail}" if detail and not cond else ""))
    if not cond:
        failures += 1


# --- /health ---
r = client.get("/health")
check("GET /health returns 200", r.status_code == 200, r.text)
check("GET /health reports module loaded", r.json().get("status") == "ok", r.text)

# --- /blocks ---
r = client.get("/blocks")
check("GET /blocks returns 200", r.status_code == 200, r.text)
block_types = {b["type"] for b in r.json()["blocks"]}
check("GET /blocks includes PID", "PID" in block_types, str(block_types))

# --- /simulate: valid diagram ---
r = client.post("/simulate", json={"diagram": VALID_LOOP_DIAGRAM, "dt": 0.01, "tEnd": 2.0})
check("POST /simulate (valid) returns 200", r.status_code == 200, r.text)
if r.status_code == 200:
    body = r.json()
    check("POST /simulate order has 5 blocks", len(body["order"]) == 5, str(body["order"]))
    check("POST /simulate traces include scope1", "scope1" in body["traces"])

# --- /simulate: algebraic loop should be a structured 422, not a 500 ---
r = client.post("/simulate", json={"diagram": BROKEN_LOOP_DIAGRAM, "dt": 0.01, "tEnd": 1.0})
check("POST /simulate (broken loop) returns 422", r.status_code == 422, r.text)
check("POST /simulate (broken loop) error names the cycle",
      "sum1" in r.json().get("detail", "") and "gain1" in r.json().get("detail", ""),
      r.text)

# --- /tune: PSO auto-tuning end-to-end through the API ---
r = client.post("/tune", json={
    "diagram": CLOSED_LOOP_PID_DIAGRAM,
    "pid_block_id": "pid1",
    "error_block_id": "sum_e",
    "lower_bounds": [0.0, 0.0, 0.0],
    "upper_bounds": [20.0, 50.0, 5.0],
    "Tf": 0.05,
    "effort_lambda": 0.02,
    "dt": 0.005,
    "tEnd": 5.0,
    "num_particles": 25,
    "max_iters": 40,
})
check("POST /tune returns 200", r.status_code == 200, r.text)
if r.status_code == 200:
    body = r.json()
    print("       baseline:", body["baseline"])
    print("       tuned:   ", body["tuned"])
    print("       improvement_factor:", body["improvement_factor"])
    check("POST /tune improves cost by >3x", body["improvement_factor"] > 3.0, str(body["improvement_factor"]))
    check("POST /tune tuned system converges", abs(body["tuned"]["final_error"]) < 0.05,
          str(body["tuned"]["final_error"]))
    check("POST /tune matches known C++/Python result (Kp~6.83)",
          abs(body["tuned"]["Kp"] - 6.833) < 0.01, str(body["tuned"]["Kp"]))

# --- /ws/simulate: WebSocket streaming ---
with client.websocket_connect("/ws/simulate") as ws:
    ws.send_json({"diagram": VALID_LOOP_DIAGRAM, "dt": 0.01, "tEnd": 1.0, "max_samples": 50})
    messages = []
    while True:
        msg = ws.receive_json()
        messages.append(msg)
        if msg.get("done") or "error" in msg:
            break
    check("WS /ws/simulate streamed multiple frames", len(messages) > 5, str(len(messages)))
    check("WS /ws/simulate ended with done:true", messages[-1].get("done") is True, str(messages[-1]))
    check("WS /ws/simulate frames contain outputs", "outputs" in messages[0], str(messages[0]))

# --- /ws/simulate: broken loop reported as a WebSocket error message, not a crash ---
with client.websocket_connect("/ws/simulate") as ws:
    ws.send_json({"diagram": BROKEN_LOOP_DIAGRAM, "dt": 0.01, "tEnd": 1.0})
    msg = ws.receive_json()
    check("WS /ws/simulate reports algebraic loop as error message",
          "error" in msg and "sum1" in msg["error"] and "gain1" in msg["error"], str(msg))

print()
if failures == 0:
    print("ALL API TESTS PASSED")
    sys.exit(0)
else:
    print(f"{failures} API TEST(S) FAILED")
    sys.exit(1)
