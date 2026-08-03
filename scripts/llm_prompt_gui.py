#!/usr/bin/env python3
"""Small Tk GUI that routes natural-language tasks to the demo launchers."""

from __future__ import annotations

import argparse
import os
import queue
import shlex
import signal
import subprocess
import threading
import tkinter as tk
import tkinter.font as tkfont
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk


WORKSPACE = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------------------
# Font settings: edit these values to customize the entire interface.
# ---------------------------------------------------------------------------
PREFERRED_UI_FONT = "Noto Sans CJK SC"
PREFERRED_MONO_FONT = "Noto Sans Mono CJK SC"
BASE_FONT_SIZE = 11
TITLE_FONT_SIZE = 21
PROMPT_FONT_SIZE = 13
LOG_FONT_SIZE = 10


@dataclass(frozen=True)
class Demo:
    key: str
    label: str
    script: str
    description: str
    example: str


DEMOS = (
    Demo(
        "rcm",
        "Multi-port Localization / RCM",
        "start_llm_vlm_rcm_demo.sh",
        "Locate a target port on the multi-port phantom, select a reachable insertion pose, and establish RCM.",
        "Locate the upper-left port in the camera view, automatically select the nearest reachable insertion pose within a 20-degree cone, and establish RCM",
    ),
    Demo(
        "medical",
        "Medical Object Localization",
        "start_medical_llm_demo.sh",
        "Open-vocabulary target localization and manipulation in a medical scene.",
        "Move above the tissue pad",
    ),
    Demo(
        "medical_grasp",
        "Medical Instrument Grasping",
        "start_medical_grasp_demo.sh",
        "Identify and grasp a medical instrument, then move it to the sorting tray.",
        "Grasp the silver surgical scissors on the left and place them in the center of the tray",
    ),
    Demo(
        "scissors_keypoint",
        "Scissors Keypoint",
        "start_give_me_scissors_keypoint_demo.sh",
        "Use a language target to trigger scissors segmentation, keypoint localization, and tracking.",
        "Locate a graspable keypoint on the silver surgical scissors on the left",
    ),
    Demo(
        "medical_keypoint",
        "Medical Instrument Keypoint Debug",
        "start_medical_keypoint_debug.sh",
        "Use an LLM task to trigger instrument segmentation and the keypoint debug view.",
        "Locate a stable grasp keypoint on the silver surgical instrument on the left",
    ),
    Demo(
        "household",
        "Open-vocabulary Household Objects",
        "start_open_vocab_household_demo.sh",
        "Open-vocabulary target localization and manipulation in a household scene.",
        "Move above the mug",
    ),
)
DEMO_BY_KEY = {demo.key: demo for demo in DEMOS}


def resolve_demo(key: str) -> Demo:
    try:
        return DEMO_BY_KEY[key]
    except KeyError as exc:
        raise ValueError(f"unknown demo {key!r}; choose: {', '.join(DEMO_BY_KEY)}") from exc


def command_for(demo: Demo, action: str, prompt: str = "") -> list[str]:
    script = WORKSPACE / demo.script
    command = [str(script), action]
    if action == "task":
        prompt = prompt.strip()
        if not prompt:
            raise ValueError("The task prompt cannot be empty")
        command.append(prompt)
    return command


class PromptWindow:
    BG = "#eef2f7"
    CARD = "#ffffff"
    INK = "#172033"
    MUTED = "#64748b"
    ACCENT = "#2563eb"
    SUCCESS = "#0f9f6e"
    DANGER = "#dc2626"

    def __init__(self, root: tk.Tk, selected_key: str):
        self.root = root
        self.output_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.process: subprocess.Popen[str] | None = None
        self.selected = tk.StringVar()
        self.status = tk.StringVar(value="Ready")
        self.description = tk.StringVar()
        self.command_preview = tk.StringVar()

        root.title("HiSurg LLM Task Input")
        root.geometry("920x720")
        root.minsize(760, 600)
        root.configure(bg=self.BG)
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._configure_fonts()
        self._configure_styles()
        self._build()

        initial = resolve_demo(selected_key)
        self.selected.set(initial.label)
        self._select_demo(fill_example=True)
        self.root.after(80, self._drain_output)

    def _configure_fonts(self) -> None:
        available = set(tkfont.families(self.root))
        cjk_candidates = (
            PREFERRED_UI_FONT,
            "Source Han Sans SC",
            "WenQuanYi Micro Hei",
            "Microsoft YaHei UI",
            "Microsoft YaHei",
            "Droid Sans Fallback",
        )
        mono_candidates = (
            PREFERRED_MONO_FONT,
            "Sarasa Mono SC",
            "WenQuanYi Zen Hei Mono",
            "DejaVu Sans Mono",
        )
        self.ui_font = next((name for name in cjk_candidates if name in available), "TkDefaultFont")
        self.mono_font = next((name for name in mono_candidates if name in available), self.ui_font)

        # Configure Tk named fonts as well, so Text/Combobox/dialog content has
        # the same legible Simplified-Chinese glyphs as ttk labels and buttons.
        for named_font in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
            tkfont.nametofont(named_font).configure(family=self.ui_font, size=BASE_FONT_SIZE)
        tkfont.nametofont("TkFixedFont").configure(family=self.mono_font, size=LOG_FONT_SIZE)
        self.root.option_add("*Font", (self.ui_font, BASE_FONT_SIZE))

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("App.TFrame", background=self.BG)
        style.configure("Card.TFrame", background=self.CARD)
        style.configure(
            "Title.TLabel", background=self.BG, foreground=self.INK, font=(self.ui_font, TITLE_FONT_SIZE, "bold")
        )
        style.configure(
            "Subtitle.TLabel", background=self.BG, foreground=self.MUTED, font=(self.ui_font, BASE_FONT_SIZE)
        )
        style.configure(
            "Field.TLabel", background=self.CARD, foreground=self.INK, font=(self.ui_font, BASE_FONT_SIZE, "bold")
        )
        style.configure("Hint.TLabel", background=self.CARD, foreground=self.MUTED, font=(self.ui_font, BASE_FONT_SIZE))
        style.configure("Accent.TButton", font=(self.ui_font, BASE_FONT_SIZE, "bold"), padding=(18, 9))
        style.map("Accent.TButton", background=[("!disabled", self.ACCENT)], foreground=[("!disabled", "white")])
        style.configure("Soft.TButton", font=(self.ui_font, BASE_FONT_SIZE), padding=(12, 8))

    def _build(self) -> None:
        outer = ttk.Frame(self.root, style="App.TFrame", padding=24)
        outer.pack(fill="both", expand=True)

        ttk.Label(outer, text="HiSurg Intelligent Surgery Demo Console", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            outer,
            text="Enter a natural-language task and route it to the selected demo's LLM planner",
            style="Subtitle.TLabel",
        ).pack(anchor="w", pady=(3, 16))

        card = ttk.Frame(outer, style="Card.TFrame", padding=18)
        card.pack(fill="x")
        ttk.Label(card, text="Demo Scene", style="Field.TLabel").grid(row=0, column=0, sticky="w")
        self.combo = ttk.Combobox(
            card,
            state="readonly",
            textvariable=self.selected,
            values=[demo.label for demo in DEMOS],
            font=(self.ui_font, 11),
        )
        self.combo.grid(row=1, column=0, sticky="ew", pady=(6, 3))
        self.combo.bind("<<ComboboxSelected>>", lambda _event: self._select_demo(fill_example=True))
        ttk.Label(card, textvariable=self.description, style="Hint.TLabel").grid(
            row=2, column=0, sticky="w", pady=(2, 14)
        )

        ttk.Label(card, text="Task Prompt", style="Field.TLabel").grid(row=3, column=0, sticky="w")
        self.prompt = tk.Text(
            card,
            height=5,
            wrap="word",
            font=(self.ui_font, PROMPT_FONT_SIZE),
            relief="solid",
            borderwidth=1,
            padx=12,
            pady=10,
            foreground=self.INK,
            insertbackground=self.INK,
        )
        self.prompt.grid(row=4, column=0, sticky="ew", pady=(6, 6))
        self.prompt.bind("<Control-Return>", self._submit_shortcut)
        self.prompt.bind("<KeyRelease>", lambda _event: self._refresh_preview())
        ttk.Label(card, text="Press Ctrl+Enter to submit", style="Hint.TLabel").grid(row=5, column=0, sticky="e")

        buttons = ttk.Frame(card, style="Card.TFrame")
        buttons.grid(row=6, column=0, sticky="ew", pady=(12, 0))
        self.start_button = ttk.Button(buttons, text="Start Scene", style="Soft.TButton", command=lambda: self._run("start"))
        self.start_button.pack(side="left")
        self.status_button = ttk.Button(buttons, text="Check Status", style="Soft.TButton", command=lambda: self._run("status"))
        self.status_button.pack(side="left", padx=7)
        self.stop_button = ttk.Button(buttons, text="Stop Scene", style="Soft.TButton", command=lambda: self._run("stop"))
        self.stop_button.pack(side="left")
        self.submit_button = ttk.Button(buttons, text="Send to LLM", style="Accent.TButton", command=lambda: self._run("task"))
        self.submit_button.pack(side="right")
        card.columnconfigure(0, weight=1)

        info = ttk.Frame(outer, style="App.TFrame")
        info.pack(fill="x", pady=(13, 5))
        self.status_dot = tk.Label(info, text="●", bg=self.BG, fg=self.SUCCESS, font=(self.ui_font, 12))
        self.status_dot.pack(side="left")
        ttk.Label(info, textvariable=self.status, style="Subtitle.TLabel").pack(side="left", padx=(5, 0))
        ttk.Label(info, textvariable=self.command_preview, style="Subtitle.TLabel").pack(side="right")

        log_card = ttk.Frame(outer, style="Card.TFrame", padding=10)
        log_card.pack(fill="both", expand=True)
        self.log = tk.Text(
            log_card,
            height=12,
            wrap="word",
            state="disabled",
            bg="#111827",
            fg="#d1fae5",
            insertbackground="white",
            font=(self.mono_font, LOG_FONT_SIZE),
            padx=10,
            pady=8,
        )
        scrollbar = ttk.Scrollbar(log_card, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def _current_demo(self) -> Demo:
        label = self.selected.get()
        return next(demo for demo in DEMOS if demo.label == label)

    def _select_demo(self, fill_example: bool) -> None:
        demo = self._current_demo()
        self.description.set(demo.description)
        if fill_example:
            self.prompt.delete("1.0", "end")
            self.prompt.insert("1.0", demo.example)
        self._refresh_preview()
        self.prompt.focus_set()

    def _refresh_preview(self) -> None:
        try:
            command = command_for(self._current_demo(), "task", self.prompt.get("1.0", "end"))
            self.command_preview.set(shlex.join(command))
        except ValueError:
            self.command_preview.set("Waiting for a prompt")

    def _append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _set_busy(self, busy: bool, action: str = "") -> None:
        state = "disabled" if busy else "normal"
        for button in (self.start_button, self.status_button, self.stop_button, self.submit_button):
            button.configure(state=state)
        self.combo.configure(state="disabled" if busy else "readonly")
        self.status.set(f"Running: {action}" if busy else "Ready")
        self.status_dot.configure(fg="#f59e0b" if busy else self.SUCCESS)

    def _submit_shortcut(self, _event: tk.Event) -> str:
        self._run("task")
        return "break"

    def _run(self, action: str) -> None:
        if self.process is not None:
            messagebox.showinfo("Command in progress", "Please wait for the current command to finish.")
            return
        prompt = self.prompt.get("1.0", "end") if action == "task" else ""
        try:
            command = command_for(self._current_demo(), action, prompt)
        except ValueError as exc:
            messagebox.showwarning("Cannot submit", str(exc))
            return
        self._append_log(f"\n$ {shlex.join(command)}\n")
        self._set_busy(True, action)
        threading.Thread(target=self._worker, args=(command,), daemon=True).start()

    def _worker(self, command: list[str]) -> None:
        try:
            self.process = subprocess.Popen(
                command,
                cwd=WORKSPACE,
                env=os.environ.copy(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            assert self.process.stdout is not None
            for line in self.process.stdout:
                self.output_queue.put(("line", line))
            code = self.process.wait()
            self.output_queue.put(("done", code))
        except Exception as exc:  # surfaced in the GUI instead of killing it
            self.output_queue.put(("error", str(exc)))

    def _drain_output(self) -> None:
        try:
            while True:
                kind, value = self.output_queue.get_nowait()
                if kind == "line":
                    self._append_log(str(value))
                elif kind == "done":
                    code = int(value)
                    self._append_log(f"[Done] Exit code: {code}\n")
                    self.process = None
                    self._set_busy(False)
                    self.status.set("Completed successfully" if code == 0 else f"Failed (exit code: {code})")
                    self.status_dot.configure(fg=self.SUCCESS if code == 0 else self.DANGER)
                else:
                    self._append_log(f"[Error] {value}\n")
                    self.process = None
                    self._set_busy(False)
                    self.status.set("Failed to start command")
                    self.status_dot.configure(fg=self.DANGER)
        except queue.Empty:
            pass
        self.root.after(80, self._drain_output)

    def _on_close(self) -> None:
        if self.process is not None and self.process.poll() is None:
            if not messagebox.askyesno("Command still running", "Close the window and stop the current foreground command?"):
                return
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", choices=DEMO_BY_KEY, default=os.getenv("PROMPT_GUI_DEMO_KEY", "rcm"))
    parser.add_argument("--list-demos", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Print the routed task command without opening Tk")
    parser.add_argument("--prompt", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list_demos:
        for demo in DEMOS:
            print(f"{demo.key}\t{demo.label}\t{demo.script}")
        return 0
    if args.dry_run:
        demo = resolve_demo(args.demo)
        print(shlex.join(command_for(demo, "task", args.prompt or demo.example)))
        return 0
    root = tk.Tk()
    PromptWindow(root, args.demo)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
