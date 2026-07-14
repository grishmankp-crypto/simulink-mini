"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import ReactFlow, {
  Background,
  Controls,
  MiniMap,
  addEdge,
  useEdgesState,
  useNodesState,
  Connection,
  Edge,
  Node,
  ReactFlowProvider,
} from "reactflow";
import "reactflow/dist/style.css";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from "recharts";
import BlockNode, { BlockNodeData } from "@/components/BlockNode";
import {
  BlockMeta,
  Diagram,
  fetchBlockPalette,
  runSimulate,
  runTune,
  streamSimulate,
  TuneResponse,
} from "@/lib/api";

const nodeTypes = { block: BlockNode };

const DEFAULT_PARAMS: Record<string, Record<string, number>> = {
  Constant: { value: 1.0 },
  Gain: { k: 1.0 },
  Sum: {},
  Product: {},
  Integrator: { initial: 0.0 },
  PID: { Kp: 1.0, Ki: 0.0, Kd: 0.0, Tf: 0.05 },
  Scope: {},
};

const FALLBACK_PALETTE: BlockMeta[] = [
  { type: "Constant", label: "Constant", params: ["value"], numInputs: 0 },
  { type: "Gain", label: "Gain", params: ["k"], numInputs: 1 },
  { type: "Sum", label: "Sum", params: [], numInputs: 2, hasSigns: true },
  { type: "Product", label: "Product", params: [], numInputs: 2 },
  { type: "Integrator", label: "Integrator", params: ["initial"], numInputs: 1 },
  { type: "PID", label: "PID", params: ["Kp", "Ki", "Kd", "Tf"], numInputs: 1 },
  { type: "Scope", label: "Scope", params: [], numInputs: 1 },
];

let idCounter = 1;

function EditorInner() {
  const [palette, setPalette] = useState<BlockMeta[]>(FALLBACK_PALETTE);
  const [nodes, setNodes, onNodesChange] = useNodesState<BlockNodeData>([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState<Edge[]>([]);
  const [backendStatus, setBackendStatus] = useState<"checking" | "ok" | "down">("checking");

  const [dt, setDt] = useState(0.005);
  const [tEnd, setTEnd] = useState(5.0);
  const [times, setTimes] = useState<number[]>([]);
  const [traces, setTraces] = useState<Record<string, number[]>>({});
  const [visibleSignals, setVisibleSignals] = useState<Set<string>>(new Set());
  const [errorMsg, setErrorMsg] = useState<string | null>(null);
  const [streaming, setStreaming] = useState(false);
  const closeStreamRef = useRef<(() => void) | null>(null);

  const [tuneOpen, setTuneOpen] = useState(false);
  const [tunePidId, setTunePidId] = useState("");
  const [tuneErrId, setTuneErrId] = useState("");
  const [tuneResult, setTuneResult] = useState<TuneResponse | null>(null);
  const [tuning, setTuning] = useState(false);

  useEffect(() => {
    fetchBlockPalette()
      .then((p) => {
        setPalette(p);
        setBackendStatus("ok");
      })
      .catch(() => setBackendStatus("down"));
  }, []);

  const updateNodeParam = useCallback(
    (nodeId: string, key: string, value: number) => {
      setNodes((nds) =>
        nds.map((n) =>
          n.id === nodeId
            ? { ...n, data: { ...n.data, params: { ...n.data.params, [key]: value } } }
            : n
        )
      );
    },
    [setNodes]
  );

  const updateNodeSign = useCallback(
    (nodeId: string, index: number, sign: string) => {
      setNodes((nds) =>
        nds.map((n) => {
          if (n.id !== nodeId) return n;
          const signs = [...(n.data.signs ?? ["+", "+"])];
          signs[index] = sign;
          return { ...n, data: { ...n.data, signs } };
        })
      );
    },
    [setNodes]
  );

  const addBlock = useCallback(
    (meta: BlockMeta) => {
      const id = `${meta.type.toLowerCase()}${idCounter++}`;
      const newNode: Node<BlockNodeData> = {
        id,
        type: "block",
        position: { x: 80 + Math.random() * 400, y: 60 + Math.random() * 300 },
        data: {
          blockType: meta.type,
          numInputs: meta.numInputs,
          params: { ...(DEFAULT_PARAMS[meta.type] ?? {}) },
          signs: meta.hasSigns ? ["+", "-"] : undefined,
          onParamChange: (key, value) => updateNodeParam(id, key, value),
          onSignChange: (index, sign) => updateNodeSign(id, index, sign),
        },
      };
      setNodes((nds) => [...nds, newNode]);
    },
    [setNodes, updateNodeParam, updateNodeSign]
  );

  const onConnect = useCallback(
    (connection: Connection) => setEdges((eds) => addEdge(connection, eds)),
    [setEdges]
  );

  const loadExample = useCallback(() => {
    idCounter = 100;
    const mk = (
      id: string,
      blockType: string,
      x: number,
      y: number,
      params: Record<string, number>,
      signs?: string[]
    ): Node<BlockNodeData> => ({
      id,
      type: "block",
      position: { x, y },
      data: {
        blockType,
        numInputs: FALLBACK_PALETTE.find((b) => b.type === blockType)?.numInputs ?? 1,
        params,
        signs,
        onParamChange: (key, value) => updateNodeParam(id, key, value),
        onSignChange: (index, sign) => updateNodeSign(id, index, sign),
      },
    });

    setNodes([
      mk("ref", "Constant", 20, 140, { value: 1.0 }),
      mk("sum_e", "Sum", 220, 140, {}, ["+", "-"]),
      mk("pid1", "PID", 420, 140, { Kp: 0.5, Ki: 0.1, Kd: 0.0, Tf: 0.05 }),
      mk("plant", "Integrator", 640, 140, { initial: 0.0 }),
      mk("scope", "Scope", 860, 140, {}),
    ]);
    setEdges([
      { id: "e1", source: "ref", target: "sum_e", targetHandle: "in-0" },
      { id: "e2", source: "plant", target: "sum_e", targetHandle: "in-1" },
      { id: "e3", source: "sum_e", target: "pid1", targetHandle: "in-0" },
      { id: "e4", source: "pid1", target: "plant", targetHandle: "in-0" },
      { id: "e5", source: "plant", target: "scope", targetHandle: "in-0" },
    ]);
    setTunePidId("pid1");
    setTuneErrId("sum_e");
  }, [setNodes, setEdges, updateNodeParam, updateNodeSign]);

  const buildDiagram = useCallback((): Diagram => {
    return {
      blocks: nodes.map((n) => ({
        id: n.id,
        type: n.data.blockType,
        params: Object.keys(n.data.params ?? {}).length ? n.data.params : undefined,
        signs: n.data.signs,
      })),
      edges: edges.map((e) => ({
        src: e.source,
        dst: e.target,
        dstInput: e.targetHandle ? parseInt(e.targetHandle.replace("in-", ""), 10) : 0,
      })),
    };
  }, [nodes, edges]);

  const handleRun = useCallback(async () => {
    setErrorMsg(null);
    try {
      const diagram = buildDiagram();
      const result = await runSimulate(diagram, dt, tEnd);
      setTimes(result.times);
      setTraces(result.traces);
      const scopeIds = nodes.filter((n) => n.data.blockType === "Scope").map((n) => n.id);
      setVisibleSignals(new Set(scopeIds.length ? scopeIds : Object.keys(result.traces)));
    } catch (e) {
      setErrorMsg(e instanceof Error ? e.message : String(e));
    }
  }, [buildDiagram, dt, tEnd, nodes]);

  const handleStream = useCallback(() => {
    setErrorMsg(null);
    setTimes([]);
    setTraces({});
    setStreaming(true);
    const liveTimes: number[] = [];
    const liveTraces: Record<string, number[]> = {};

    const close = streamSimulate(
      buildDiagram(),
      dt,
      tEnd,
      300,
      (t, outputs) => {
        liveTimes.push(t);
        for (const [k, v] of Object.entries(outputs)) {
          if (!liveTraces[k]) liveTraces[k] = [];
          liveTraces[k].push(v);
        }
        setTimes([...liveTimes]);
        setTraces({ ...liveTraces });
      },
      () => {
        setStreaming(false);
        const scopeIds = nodes.filter((n) => n.data.blockType === "Scope").map((n) => n.id);
        setVisibleSignals(new Set(scopeIds.length ? scopeIds : Object.keys(liveTraces)));
      },
      (msg) => {
        setErrorMsg(msg);
        setStreaming(false);
      }
    );
    closeStreamRef.current = close;
  }, [buildDiagram, dt, tEnd, nodes]);

  const handleTune = useCallback(async () => {
    setTuning(true);
    setTuneResult(null);
    setErrorMsg(null);
    try {
      const diagram = buildDiagram();
      const result = await runTune({
        diagram,
        pid_block_id: tunePidId,
        error_block_id: tuneErrId,
        lower_bounds: [0, 0, 0],
        upper_bounds: [20, 50, 5],
        Tf: 0.05,
        effort_lambda: 0.02,
        dt: 0.005,
        tEnd: 5.0,
        num_particles: 25,
        max_iters: 40,
      });
      setTuneResult(result);
      setNodes((nds) =>
        nds.map((n) =>
          n.id === tunePidId
            ? {
                ...n,
                data: {
                  ...n.data,
                  params: { ...n.data.params, Kp: result.tuned.Kp, Ki: result.tuned.Ki, Kd: result.tuned.Kd },
                },
              }
            : n
        )
      );
    } catch (e) {
      setErrorMsg(e instanceof Error ? e.message : String(e));
    } finally {
      setTuning(false);
    }
  }, [buildDiagram, tunePidId, tuneErrId, setNodes]);

  const chartData = times.map((t, i) => {
    const row: Record<string, number> = { t };
    for (const sig of visibleSignals) row[sig] = traces[sig]?.[i];
    return row;
  });

  const COLORS = ["#2563eb", "#dc2626", "#059669", "#d97706", "#7c3aed", "#0891b2"];

  return (
    <div className="h-screen w-screen flex flex-col bg-slate-50">
      <header className="flex items-center gap-3 px-4 py-2 border-b bg-white shadow-sm">
        <h1 className="font-bold text-slate-800">simulink-mini editor</h1>
        <span
          className={`text-xs px-2 py-0.5 rounded-full ${
            backendStatus === "ok"
              ? "bg-green-100 text-green-700"
              : backendStatus === "down"
              ? "bg-red-100 text-red-700"
              : "bg-slate-100 text-slate-500"
          }`}
        >
          backend: {backendStatus === "ok" ? "connected" : backendStatus === "down" ? "unreachable" : "checking..."}
        </span>
        <button onClick={loadExample} className="ml-4 text-xs px-2 py-1 rounded bg-slate-200 hover:bg-slate-300">
          Load PID+Plant example
        </button>
        <div className="flex items-center gap-2 ml-4 text-xs text-slate-600">
          <label>dt</label>
          <input
            type="number"
            step="any"
            value={dt}
            onChange={(e) => setDt(parseFloat(e.target.value))}
            className="w-16 border rounded px-1"
          />
          <label>tEnd</label>
          <input
            type="number"
            step="any"
            value={tEnd}
            onChange={(e) => setTEnd(parseFloat(e.target.value))}
            className="w-16 border rounded px-1"
          />
        </div>
        <button onClick={handleRun} className="ml-2 text-xs px-3 py-1 rounded bg-blue-600 text-white hover:bg-blue-700">
          Run simulation
        </button>
        <button
          onClick={handleStream}
          disabled={streaming}
          className="text-xs px-3 py-1 rounded bg-indigo-600 text-white hover:bg-indigo-700 disabled:opacity-50"
        >
          {streaming ? "Streaming..." : "Stream live"}
        </button>
        <button onClick={() => setTuneOpen((v) => !v)} className="text-xs px-3 py-1 rounded bg-red-600 text-white hover:bg-red-700">
          Tune PID
        </button>
      </header>

      {errorMsg && (
        <div className="bg-red-50 text-red-800 text-sm px-4 py-2 border-b border-red-200">
          <strong>Error:</strong> {errorMsg}
        </div>
      )}

      {tuneOpen && (
        <div className="bg-white border-b px-4 py-3 flex flex-col gap-2 text-sm">
          <div className="flex items-center gap-2 flex-wrap">
            <label>PID block id</label>
            <input value={tunePidId} onChange={(e) => setTunePidId(e.target.value)} className="border rounded px-2 py-1 w-28" />
            <label>Error signal block id</label>
            <input value={tuneErrId} onChange={(e) => setTuneErrId(e.target.value)} className="border rounded px-2 py-1 w-28" />
            <button
              onClick={handleTune}
              disabled={tuning || !tunePidId || !tuneErrId}
              className="px-3 py-1 rounded bg-red-600 text-white hover:bg-red-700 disabled:opacity-50"
            >
              {tuning ? "Running PSO..." : "Run PSO auto-tune"}
            </button>
          </div>
          {tuneResult && (
            <div className="text-xs bg-slate-50 rounded p-2 grid grid-cols-2 gap-x-6 gap-y-1">
              <div>
                <strong>Baseline</strong>: Kp={tuneResult.baseline.Kp.toFixed(3)}, Ki={tuneResult.baseline.Ki.toFixed(3)}, Kd=
                {tuneResult.baseline.Kd.toFixed(3)} — cost={tuneResult.baseline.cost.toExponential(2)}
              </div>
              <div>
                <strong>Tuned</strong>: Kp={tuneResult.tuned.Kp.toFixed(3)}, Ki={tuneResult.tuned.Ki.toFixed(3)}, Kd=
                {tuneResult.tuned.Kd.toFixed(3)} — cost={tuneResult.tuned.cost.toExponential(2)}
              </div>
              <div>
                Improvement factor: <strong>{tuneResult.improvement_factor?.toFixed(2)}x</strong>
              </div>
              <div>Final tracking error: {tuneResult.tuned.final_error.toExponential(2)}</div>
              {tuneResult.gain_pinned_to_bound && (
                <div className="col-span-2 text-amber-700">
                  Note: at least one tuned gain landed on a search-bound — the true optimum may lie outside the current
                  [Kp,Ki,Kd] search box.
                </div>
              )}
            </div>
          )}
        </div>
      )}

      <div className="flex flex-1 min-h-0">
        <aside className="w-48 border-r bg-white p-3 flex flex-col gap-2 overflow-y-auto">
          <h2 className="text-xs font-semibold text-slate-500 uppercase">Blocks</h2>
          {palette.map((meta) => (
            <button
              key={meta.type}
              onClick={() => addBlock(meta)}
              className="text-left text-sm px-2 py-1 rounded border border-slate-200 hover:bg-slate-100"
            >
              + {meta.label}
            </button>
          ))}
        </aside>

        <div className="flex-1 min-w-0">
          <ReactFlow
            nodes={nodes}
            edges={edges}
            onNodesChange={onNodesChange}
            onEdgesChange={onEdgesChange}
            onConnect={onConnect}
            nodeTypes={nodeTypes}
            fitView
          >
            <Background />
            <Controls />
            <MiniMap />
          </ReactFlow>
        </div>

        <aside className="w-[420px] border-l bg-white p-3 flex flex-col gap-2 overflow-y-auto">
          <h2 className="text-xs font-semibold text-slate-500 uppercase">Signals</h2>
          <div className="flex flex-wrap gap-2 text-xs">
            {Object.keys(traces).map((sig) => (
              <label key={sig} className="flex items-center gap-1">
                <input
                  type="checkbox"
                  checked={visibleSignals.has(sig)}
                  onChange={(e) => {
                    setVisibleSignals((prev) => {
                      const next = new Set(prev);
                      if (e.target.checked) next.add(sig);
                      else next.delete(sig);
                      return next;
                    });
                  }}
                />
                {sig}
              </label>
            ))}
          </div>
          <div className="h-64">
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={chartData}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="t" tickFormatter={(v) => v.toFixed(1)} fontSize={10} />
                <YAxis fontSize={10} />
                <Tooltip />
                <Legend />
                {[...visibleSignals].map((sig, i) => (
                  <Line key={sig} type="monotone" dataKey={sig} stroke={COLORS[i % COLORS.length]} dot={false} isAnimationActive={false} />
                ))}
              </LineChart>
            </ResponsiveContainer>
          </div>
        </aside>
      </div>
    </div>
  );
}

export default function Page() {
  return (
    <ReactFlowProvider>
      <EditorInner />
    </ReactFlowProvider>
  );
}
