// Typed helpers for talking to the FastAPI backend (../backend/app.py).
// Base URL is configurable via NEXT_PUBLIC_API_BASE (defaults to localhost:8000,
// where `uvicorn app:app --port 8000` runs per the project README).

export const API_BASE =
  process.env.NEXT_PUBLIC_API_BASE || "http://localhost:8000";

export interface DiagramBlock {
  id: string;
  type: string;
  params?: Record<string, number>;
  signs?: string[];
}

export interface DiagramEdge {
  src: string;
  dst: string;
  dstInput: number;
}

export interface Diagram {
  blocks: DiagramBlock[];
  edges: DiagramEdge[];
}

export interface SimulateResponse {
  order: string[];
  times: number[];
  traces: Record<string, number[]>;
}

export interface BlockMeta {
  type: string;
  label: string;
  params: string[];
  numInputs: number;
  hasSigns?: boolean;
}

export interface TuneResult {
  Kp: number;
  Ki: number;
  Kd: number;
  ise: number;
  effort: number;
  cost: number;
  final_error: number;
  diverged: boolean;
}

export interface TuneResponse {
  baseline: TuneResult;
  tuned: TuneResult;
  improvement_factor: number | null;
  cost_history: number[];
  gain_pinned_to_bound: boolean;
}

async function asJsonOrThrow(res: Response) {
  const text = await res.text();
  let body: unknown;
  try {
    body = JSON.parse(text);
  } catch {
    body = text;
  }
  if (!res.ok) {
    const detail =
      typeof body === "object" && body !== null && "detail" in body
        ? String((body as { detail: unknown }).detail)
        : text;
    throw new Error(detail || `Request failed with status ${res.status}`);
  }
  return body;
}

export async function fetchBlockPalette(): Promise<BlockMeta[]> {
  const res = await fetch(`${API_BASE}/blocks`);
  const body = (await asJsonOrThrow(res)) as { blocks: BlockMeta[] };
  return body.blocks;
}

export async function runSimulate(
  diagram: Diagram,
  dt: number,
  tEnd: number
): Promise<SimulateResponse> {
  const res = await fetch(`${API_BASE}/simulate`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ diagram, dt, tEnd }),
  });
  return (await asJsonOrThrow(res)) as SimulateResponse;
}

export interface TuneRequestParams {
  diagram: Diagram;
  pid_block_id: string;
  error_block_id: string;
  lower_bounds: number[];
  upper_bounds: number[];
  Tf: number;
  effort_lambda: number;
  dt: number;
  tEnd: number;
  num_particles: number;
  max_iters: number;
}

export async function runTune(params: TuneRequestParams): Promise<TuneResponse> {
  const res = await fetch(`${API_BASE}/tune`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(params),
  });
  return (await asJsonOrThrow(res)) as TuneResponse;
}

// Streams a simulation live over WebSocket, calling onFrame for every sample
// and onDone/onError at the end. Returns a function to close the socket early.
export function streamSimulate(
  diagram: Diagram,
  dt: number,
  tEnd: number,
  maxSamples: number,
  onFrame: (t: number, outputs: Record<string, number>) => void,
  onDone: () => void,
  onError: (msg: string) => void
): () => void {
  const wsBase = API_BASE.replace(/^http/, "ws");
  const ws = new WebSocket(`${wsBase}/ws/simulate`);

  ws.onopen = () => {
    ws.send(JSON.stringify({ diagram, dt, tEnd, max_samples: maxSamples }));
  };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.error) {
      onError(msg.error);
      ws.close();
      return;
    }
    if (msg.done) {
      onDone();
      return;
    }
    onFrame(msg.t, msg.outputs);
  };
  ws.onerror = () => onError("WebSocket connection error");

  return () => ws.close();
}
