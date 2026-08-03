#!/usr/bin/env python3
"""Small Tk window that maps GPU usage back to demo node PID files.

The monitor intentionally stays outside ROS so it can keep running even when a
ROS node crashes. It reads `run_pids/*.pid`, samples `nvidia-smi`, and groups
GPU processes by Linux process group so children of a launched node are counted
under the same row.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class NodeProcess:
    name: str
    pid: int
    pgrp: Optional[int]
    command: str


@dataclass
class GpuProcess:
    pid: int
    process_name: str = ""
    used_memory_mb: Optional[int] = None
    gpu_index: Optional[int] = None
    sm_percent: Optional[int] = None
    mem_percent: Optional[int] = None
    command: str = ""


@dataclass
class Row:
    name: str
    root_pid: str = ""
    gpu_pids: set[int] = field(default_factory=set)
    process_names: set[str] = field(default_factory=set)
    memory_mb: int = 0
    sm_values: list[int] = field(default_factory=list)
    gpu_indices: set[int] = field(default_factory=set)
    state: str = "running"


@dataclass
class Snapshot:
    gpu_summary: str
    rows: list[Row]
    errors: list[str]
    updated_at: float


def _run(args: list[str], timeout_sec: float = 2.5) -> tuple[str, str]:
    try:
        completed = subprocess.run(
            args,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_sec,
        )
    except FileNotFoundError:
        return "", f"command not found: {args[0]}"
    except subprocess.TimeoutExpired:
        return "", f"command timed out: {' '.join(args)}"

    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        return "", detail or f"command failed with exit code {completed.returncode}"
    return completed.stdout, ""


def _safe_int(text: str) -> Optional[int]:
    cleaned = "".join(ch for ch in str(text) if ch.isdigit() or ch == "-")
    if cleaned in ("", "-"):
        return None
    try:
        return int(cleaned)
    except ValueError:
        return None


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _proc_stat(pid: int) -> tuple[Optional[int], Optional[int]]:
    """Return (ppid, pgrp) from /proc/<pid>/stat."""
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except OSError:
        return None, None

    end = raw.rfind(")")
    if end < 0:
        return None, None
    fields = raw[end + 2 :].split()
    if len(fields) < 3:
        return None, None
    ppid = _safe_int(fields[1])
    pgrp = _safe_int(fields[2])
    return ppid, pgrp


def _proc_pgrp(pid: int) -> Optional[int]:
    return _proc_stat(pid)[1]


def _proc_ppid(pid: int) -> Optional[int]:
    return _proc_stat(pid)[0]


def _proc_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        text = raw.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
        if text:
            return text
    except OSError:
        pass
    try:
        return Path(f"/proc/{pid}/comm").read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _is_descendant(pid: int, root_pid: int) -> bool:
    current = pid
    seen: set[int] = set()
    while current > 1 and current not in seen:
        if current == root_pid:
            return True
        seen.add(current)
        ppid = _proc_ppid(current)
        if ppid is None:
            break
        current = ppid
    return False


def read_nodes(pid_dir: Path) -> list[NodeProcess]:
    nodes: list[NodeProcess] = []
    if not pid_dir.exists():
        return nodes

    for pid_file in sorted(pid_dir.glob("*.pid")):
        try:
            pid = int(pid_file.read_text(encoding="utf-8").strip())
        except Exception:
            continue
        if not _pid_alive(pid):
            continue
        nodes.append(
            NodeProcess(
                name=pid_file.stem,
                pid=pid,
                pgrp=_proc_pgrp(pid),
                command=_proc_cmdline(pid),
            )
        )
    return nodes


def query_gpu_summary() -> tuple[str, list[str]]:
    stdout, error = _run(
        [
            "nvidia-smi",
            "--query-gpu=index,name,utilization.gpu,memory.used,memory.total",
            "--format=csv,noheader,nounits",
        ]
    )
    if error:
        return "GPU summary unavailable", [error]

    parts: list[str] = []
    for line in stdout.splitlines():
        columns = [item.strip() for item in line.split(",")]
        if len(columns) < 5:
            continue
        index, name, util, used, total = columns[:5]
        parts.append(f"GPU {index} {name}: util {util}%  mem {used}/{total} MiB")
    return " | ".join(parts) if parts else "No NVIDIA GPU data", []


def query_compute_apps() -> tuple[dict[int, GpuProcess], list[str]]:
    stdout, error = _run(
        [
            "nvidia-smi",
            "--query-compute-apps=pid,process_name,used_gpu_memory",
            "--format=csv,noheader,nounits",
        ]
    )
    if error:
        return {}, [error]

    processes: dict[int, GpuProcess] = {}
    for line in stdout.splitlines():
        if not line.strip() or "No running" in line:
            continue
        columns = [item.strip() for item in line.split(",")]
        if len(columns) < 3:
            continue
        pid = _safe_int(columns[0])
        if pid is None:
            continue
        processes[pid] = GpuProcess(
            pid=pid,
            process_name=columns[1],
            used_memory_mb=_safe_int(columns[2]),
            command=_proc_cmdline(pid),
        )
    return processes, []


def query_pmon() -> tuple[dict[int, GpuProcess], list[str]]:
    stdout, error = _run(["nvidia-smi", "pmon", "-c", "1", "-s", "um"], timeout_sec=4.0)
    if error:
        return {}, [f"pmon unavailable: {error}"]

    processes: dict[int, GpuProcess] = {}
    for line in stdout.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        columns = line.split(None, 7)
        if len(columns) < 8:
            continue
        gpu_index = _safe_int(columns[0])
        pid = _safe_int(columns[1])
        if pid is None:
            continue
        sm_percent = _safe_int(columns[3])
        mem_percent = _safe_int(columns[4])
        processes[pid] = GpuProcess(
            pid=pid,
            gpu_index=gpu_index,
            sm_percent=sm_percent,
            mem_percent=mem_percent,
            process_name=columns[7],
            command=_proc_cmdline(pid),
        )
    return processes, []


def _matching_node(pid: int, nodes: list[NodeProcess]) -> Optional[NodeProcess]:
    pid_pgrp = _proc_pgrp(pid)
    for node in nodes:
        if pid == node.pid:
            return node
        if pid_pgrp is not None and node.pgrp is not None and pid_pgrp == node.pgrp:
            return node
        if _is_descendant(pid, node.pid):
            return node
    return None


def collect_snapshot(pid_dir: Path) -> Snapshot:
    errors: list[str] = []
    gpu_summary, gpu_errors = query_gpu_summary()
    errors.extend(gpu_errors)

    nodes = read_nodes(pid_dir)
    apps, app_errors = query_compute_apps()
    errors.extend(app_errors)

    pmon, pmon_errors = query_pmon()
    # pmon is a best-effort source for SM%. It may be unsupported even when
    # memory accounting works, so keep its error visible but non-fatal.
    errors.extend(pmon_errors)

    for pid, pmon_proc in pmon.items():
        proc = apps.setdefault(pid, GpuProcess(pid=pid, command=_proc_cmdline(pid)))
        proc.gpu_index = pmon_proc.gpu_index
        proc.sm_percent = pmon_proc.sm_percent
        proc.mem_percent = pmon_proc.mem_percent
        if not proc.process_name:
            proc.process_name = pmon_proc.process_name
        if not proc.command:
            proc.command = pmon_proc.command

    rows_by_name: dict[str, Row] = {}
    for node in nodes:
        rows_by_name[node.name] = Row(
            name=node.name,
            root_pid=str(node.pid),
            process_names={Path(node.command.split(" ")[0]).name if node.command else ""},
            state="running",
        )

    for proc in apps.values():
        node = _matching_node(proc.pid, nodes)
        row_name = node.name if node else f"external:{Path(proc.process_name).name or proc.pid}"
        row = rows_by_name.setdefault(
            row_name,
            Row(name=row_name, root_pid=str(node.pid) if node else "", state="external"),
        )
        row.gpu_pids.add(proc.pid)
        if proc.used_memory_mb is not None:
            row.memory_mb += max(proc.used_memory_mb, 0)
        if proc.sm_percent is not None:
            row.sm_values.append(max(proc.sm_percent, 0))
        if proc.gpu_index is not None:
            row.gpu_indices.add(proc.gpu_index)
        process_label = proc.process_name or proc.command or str(proc.pid)
        row.process_names.add(Path(process_label.split(" ")[0]).name)

    rows = sorted(rows_by_name.values(), key=lambda item: (item.state != "running", item.name))
    return Snapshot(gpu_summary=gpu_summary, rows=rows, errors=errors, updated_at=time.time())


def format_row(row: Row) -> tuple[str, str, str, str, str, str, str]:
    gpu = ",".join(str(item) for item in sorted(row.gpu_indices)) or "-"
    gpu_pids = ",".join(str(item) for item in sorted(row.gpu_pids)) or "-"
    sm = f"{sum(row.sm_values)}%" if row.sm_values else "n/a"
    mem = f"{row.memory_mb} MiB" if row.memory_mb else "0 MiB"
    names = ", ".join(sorted(item for item in row.process_names if item)) or "-"
    return (row.name, row.root_pid or "-", gpu_pids, gpu, sm, mem, names)


def print_once(pid_dir: Path) -> None:
    snapshot = collect_snapshot(pid_dir)
    print(snapshot.gpu_summary)
    if snapshot.errors:
        print("Warnings:")
        for error in snapshot.errors:
            print(f"  - {error}")
    print("node\troot_pid\tgpu_pids\tgpu\tsm\tmemory\tprocess")
    for row in snapshot.rows:
        print("\t".join(format_row(row)))


class GpuMonitorWindow:
    def __init__(self, pid_dir: Path, refresh_sec: float):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.pid_dir = pid_dir
        self.refresh_ms = max(int(refresh_sec * 1000), 250)

        self.root = tk.Tk()
        self.root.title("GPU Node Monitor")
        self.root.geometry("920x360")

        self.summary_var = tk.StringVar(value="Starting GPU monitor...")
        self.error_var = tk.StringVar(value="")
        self.updated_var = tk.StringVar(value="")

        ttk.Label(self.root, textvariable=self.summary_var, anchor="w").pack(
            fill="x", padx=10, pady=(8, 2)
        )
        ttk.Label(self.root, textvariable=self.updated_var, anchor="w").pack(
            fill="x", padx=10, pady=(0, 2)
        )

        columns = ("node", "root_pid", "gpu_pids", "gpu", "sm", "memory", "process")
        self.tree = ttk.Treeview(self.root, columns=columns, show="headings", height=10)
        headings = {
            "node": "node",
            "root_pid": "root pid",
            "gpu_pids": "gpu pid(s)",
            "gpu": "gpu",
            "sm": "SM util",
            "memory": "GPU memory",
            "process": "process",
        }
        widths = {
            "node": 150,
            "root_pid": 80,
            "gpu_pids": 120,
            "gpu": 50,
            "sm": 70,
            "memory": 100,
            "process": 260,
        }
        for column in columns:
            self.tree.heading(column, text=headings[column])
            self.tree.column(column, width=widths[column], anchor="w")
        self.tree.pack(fill="both", expand=True, padx=10, pady=6)

        ttk.Label(self.root, textvariable=self.error_var, anchor="w", foreground="#9a3412").pack(
            fill="x", padx=10, pady=(0, 8)
        )

        self.refresh()

    def refresh(self):
        snapshot = collect_snapshot(self.pid_dir)
        self.summary_var.set(snapshot.gpu_summary)
        self.updated_var.set(time.strftime("Updated %H:%M:%S", time.localtime(snapshot.updated_at)))
        if snapshot.errors:
            self.error_var.set(" | ".join(snapshot.errors[:2]))
        else:
            self.error_var.set("")

        existing = set(self.tree.get_children())
        for index, row in enumerate(snapshot.rows):
            item_id = row.name
            values = format_row(row)
            if item_id in existing:
                self.tree.item(item_id, values=values)
                existing.remove(item_id)
            else:
                self.tree.insert("", index, iid=item_id, values=values)
        for item_id in existing:
            self.tree.delete(item_id)

        self.root.after(self.refresh_ms, self.refresh)

    def run(self):
        self.root.mainloop()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid-dir", type=Path, default=Path("run_pids"))
    parser.add_argument("--refresh-sec", type=float, default=1.0)
    parser.add_argument("--once", action="store_true", help="print one text snapshot and exit")
    args = parser.parse_args()

    if args.once:
        print_once(args.pid_dir)
        return 0

    window = GpuMonitorWindow(args.pid_dir, args.refresh_sec)
    window.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
