"use client";

import { Handle, Position, NodeProps } from "reactflow";
import { memo } from "react";

export interface BlockNodeData {
  blockType: string;
  numInputs: number;
  params: Record<string, number>;
  signs?: string[];
  onParamChange: (key: string, value: number) => void;
  onSignChange: (index: number, sign: string) => void;
}

const BLOCK_COLORS: Record<string, string> = {
  Constant: "#64748b",
  Gain: "#2563eb",
  Sum: "#059669",
  Product: "#059669",
  Integrator: "#d97706",
  PID: "#dc2626",
  Scope: "#7c3aed",
};

function BlockNode({ data, selected }: NodeProps<BlockNodeData>) {
  const color = BLOCK_COLORS[data.blockType] ?? "#334155";
  const numInputs = data.numInputs ?? 0;

  return (
    <div
      style={{
        borderColor: color,
        boxShadow: selected ? `0 0 0 2px ${color}` : undefined,
      }}
      className="rounded-lg border-2 bg-white shadow-sm min-w-[140px]"
    >
      <div
        style={{ backgroundColor: color }}
        className="text-white text-xs font-semibold px-2 py-1 rounded-t-md"
      >
        {data.blockType}
      </div>
      <div className="p-2 flex flex-col gap-1">
        {Object.entries(data.params ?? {}).map(([key, value]) => (
          <label key={key} className="flex items-center gap-1 text-xs text-slate-700">
            <span className="w-8 shrink-0">{key}</span>
            <input
              type="number"
              step="any"
              defaultValue={value}
              onChange={(e) => data.onParamChange(key, parseFloat(e.target.value))}
              className="nodrag w-full border border-slate-300 rounded px-1 py-0.5 text-xs"
            />
          </label>
        ))}
        {data.blockType === "Sum" &&
          Array.from({ length: numInputs }).map((_, i) => (
            <label key={i} className="flex items-center gap-1 text-xs text-slate-700">
              <span className="w-8 shrink-0">sign{i}</span>
              <select
                defaultValue={data.signs?.[i] ?? "+"}
                onChange={(e) => data.onSignChange(i, e.target.value)}
                className="nodrag border border-slate-300 rounded px-1 py-0.5 text-xs"
              >
                <option value="+">+</option>
                <option value="-">-</option>
              </select>
            </label>
          ))}
      </div>

      {Array.from({ length: numInputs }).map((_, i) => (
        <Handle
          key={i}
          type="target"
          position={Position.Left}
          id={`in-${i}`}
          style={{ top: `${((i + 1) * 100) / (numInputs + 1)}%`, background: color }}
        />
      ))}

      {data.blockType !== "Scope" && (
        <Handle type="source" position={Position.Right} id="out" style={{ background: color }} />
      )}
    </div>
  );
}

export default memo(BlockNode);
