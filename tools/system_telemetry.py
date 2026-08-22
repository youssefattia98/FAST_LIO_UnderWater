#!/usr/bin/env python3
"""Record low-overhead host and GPU telemetry for a UWFL2 benchmark."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

import psutil


class TegraStatsSampler:
    def __init__(self, output: Path, start_monotonic: float) -> None:
        self.output = output
        self.start_monotonic = start_monotonic
        self.process: subprocess.Popen[str] | None = None
        self.thread: threading.Thread | None = None
        self.count = 0

    def start(self) -> None:
        self.process = subprocess.Popen(
            ["tegrastats", "--interval", "1000"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        with self.output.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("monotonic_s", "elapsed_s", "raw"))
            for line in self.process.stdout:
                now = time.monotonic()
                writer.writerow((f"{now:.6f}", f"{now - self.start_monotonic:.6f}", line.strip()))
                stream.flush()
                self.count += 1

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
        if self.thread is not None:
            self.thread.join(timeout=5.0)


class NvidiaSmiSampler:
    def __init__(self, output: Path, start_monotonic: float) -> None:
        self.output = output
        self.start_monotonic = start_monotonic
        self.process: subprocess.Popen[str] | None = None
        self.thread: threading.Thread | None = None
        self.count = 0

    def start(self) -> None:
        self.process = subprocess.Popen(
            [
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used,power.draw,temperature.gpu,clocks.sm",
                "--format=csv,noheader,nounits",
                "--loop-ms=1000",
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        with self.output.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "monotonic_s", "elapsed_s", "gpu_percent", "memory_used_mb",
                    "power_w", "temperature_c", "clock_sm_mhz",
                )
            )
            for line in self.process.stdout:
                values = [value.strip() for value in line.split(",")]
                if len(values) != 5:
                    continue
                now = time.monotonic()
                writer.writerow((f"{now:.6f}", f"{now - self.start_monotonic:.6f}", *values))
                stream.flush()
                self.count += 1

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
        if self.thread is not None:
            self.thread.join(timeout=5.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--interval", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.interval <= 0.0:
        raise ValueError("--interval must be positive")
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    stop_event = threading.Event()

    def stop(_signum, _frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    started = time.monotonic()
    gpu_sampler: TegraStatsSampler | NvidiaSmiSampler | None = None
    provider = "none"
    if shutil.which("tegrastats"):
        provider = "tegrastats"
        gpu_sampler = TegraStatsSampler(output / "tegrastats_samples.csv", started)
    elif shutil.which("nvidia-smi"):
        provider = "nvidia-smi"
        gpu_sampler = NvidiaSmiSampler(output / "gpu_samples.csv", started)
    if gpu_sampler is not None:
        gpu_sampler.start()

    system_count = 0
    psutil.cpu_percent(None, percpu=True)
    psutil.cpu_percent(None)
    with (output / "system_samples.csv").open("w", newline="") as system_stream:
        system_writer = csv.writer(system_stream)
        system_writer.writerow(
            (
                "monotonic_s", "elapsed_s", "cpu_percent", "cpu_per_core_percent",
                "ram_used_mb", "ram_available_mb", "swap_used_mb",
            )
        )
        deadline = started
        while not stop_event.is_set():
            now = time.monotonic()
            memory = psutil.virtual_memory()
            swap = psutil.swap_memory()
            per_core = psutil.cpu_percent(None, percpu=True)
            cpu = psutil.cpu_percent(None)
            system_writer.writerow(
                (
                    f"{now:.6f}", f"{now - started:.6f}", f"{cpu:.3f}",
                    ";".join(f"{value:.1f}" for value in per_core),
                    f"{memory.used / (1024.0 * 1024.0):.3f}",
                    f"{memory.available / (1024.0 * 1024.0):.3f}",
                    f"{swap.used / (1024.0 * 1024.0):.3f}",
                )
            )
            system_stream.flush()
            system_count += 1

            deadline += args.interval
            stop_event.wait(max(0.0, deadline - time.monotonic()))

    if gpu_sampler is not None:
        gpu_sampler.stop()
    summary = {
        "provider": provider,
        "start_monotonic_s": started,
        "duration_s": time.monotonic() - started,
        "system_samples": system_count,
        "gpu_samples": gpu_sampler.count if gpu_sampler is not None else 0,
    }
    (output / "telemetry_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
