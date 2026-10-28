#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fcntl
import os
import signal
import site
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path


def directory_has_native_fpga_udp(path: Path) -> bool:
    return path.exists() and (any(path.glob("fpga_udp*.so")) or any(path.glob("fpga_udp*.pyd")))


def resolve_native_fpga_udp_dir(path: Path) -> Path | None:
    if directory_has_native_fpga_udp(path):
        return path
    if path.exists():
        for child in sorted(path.iterdir()):
            if child.is_dir() and directory_has_native_fpga_udp(child):
                return child
    return None


def find_native_fpga_udp_dir(pyradar_root: Path) -> Path | None:
    override_dir = os.environ.get("FPGA_UDP_OVERRIDE_DIR", "").strip()
    if override_dir:
        resolved = resolve_native_fpga_udp_dir(Path(override_dir).expanduser())
        if resolved is not None:
            return resolved

    for candidate in [
        pyradar_root / "fpga_udp" / "build",
        Path("/home/nvidia/pyRadar-main/fpga_udp/build"),
    ]:
        resolved = resolve_native_fpga_udp_dir(candidate)
        if resolved is not None:
            return resolved

    for site_dir in site.getsitepackages():
        resolved = resolve_native_fpga_udp_dir(Path(site_dir))
        if resolved is not None:
            return resolved
    user_site = site.getusersitepackages()
    if user_site:
        return resolve_native_fpga_udp_dir(Path(user_site))
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Configure AWR2243/DCA1000, then run fft_imaging_live on the DCA1000 UDP stream."
    )
    parser.add_argument("--pyradar-root", type=Path, default=Path("/home/radar/pyRadar-main"))
    parser.add_argument(
        "--radar-config",
        type=Path,
        default=Path("/home/radar/pyRadar-main/configFiles/mmwaveconfig_0508_50f128_test.txt"),
    )
    parser.add_argument("--dca-config", type=Path, default=Path("/home/radar/pyRadar-main/configFiles/cf.json"))
    parser.add_argument(
        "--fft-config",
        type=Path,
        default=Path("/home/radar/fft_image_NX_live/release/orin_nx_min/config/fft_imaging_orin.cfg"),
    )
    parser.add_argument("--fft-bin", type=Path, default=Path("/home/radar/fft_image_NX_live/build/fft_imaging_live"))
    parser.add_argument("--dca-static-ip", default="192.168.33.30")
    parser.add_argument("--dca-adc-ip", default="192.168.33.180")
    parser.add_argument("--dca-data-port", type=int, default=4098)
    parser.add_argument("--dca-timeout-ms", type=int, default=10000)
    parser.add_argument(
        "--receiver-mode",
        choices=("pyradar", "direct"),
        default="pyradar",
        help="Use pyRadar's fpga_udp receiver by default; direct lets fft_imaging_live bind DCA1000 UDP itself.",
    )
    parser.add_argument(
        "--pyradar-chunk-frames",
        type=int,
        default=1,
        help="Frames per pyRadar receiver chunk written into the live FFT FIFO.",
    )
    parser.add_argument(
        "--pyradar-buffer-frames",
        type=int,
        default=512,
        help="Frame capacity for pyRadar's fpga_udp receive ring.",
    )
    parser.add_argument(
        "--min-raw-nonzero-fraction",
        type=float,
        default=0.001,
        help="Drop a pyRadar raw chunk when too few bytes are non-zero; catches severe UDP loss chunks.",
    )
    parser.add_argument(
        "--max-bad-raw-chunks",
        type=int,
        default=3,
        help="Stop live capture after this many consecutive bad raw chunks. Use 0 to never stop.",
    )
    parser.add_argument(
        "--pyradar-sort-in-c",
        action="store_true",
        help="Use fpga_udp's C packet sorter. Default uses pyRadar's Python sorter because this build returns packet buffers.",
    )
    parser.add_argument(
        "--allow-empty-raw",
        action="store_true",
        help="Write all-zero raw chunks to FFT anyway. Default drops them to avoid blank white videos.",
    )
    parser.add_argument("--radar-interface", default="eno1")
    parser.add_argument("--radar-interface-cidr", default="192.168.33.30/24")
    parser.add_argument("--skip-network-setup", action="store_true")
    parser.add_argument(
        "--skip-usb-reset",
        action="store_true",
        help="Do not reset the TI AR-DevPack USB device before mmWaveLink init.",
    )
    parser.add_argument(
        "--keep-ftdi-kernel-driver",
        action="store_true",
        help="Do not unload Linux ftdi_sio before AWR2243_init.",
    )
    parser.add_argument("--radar-numframes", type=int, default=50)
    parser.add_argument("--receiver-warmup-seconds", type=float, default=0.5)
    parser.add_argument("--velocity-mps", type=float, default=1.0)
    parser.add_argument(
        "--video-output",
        type=Path,
        default=None,
        help="Output AVI path. Default writes radar_live.avi into this run's metrics directory.",
    )
    parser.add_argument("--video-fps", type=float, default=5.0)
    parser.add_argument("--mjpeg-port", type=int, default=0, help="Serve live imaging as MJPEG HTTP on this port.")
    parser.add_argument("--mjpeg-quality", type=int, default=85, help="JPEG quality for MJPEG HTTP frames.")
    parser.add_argument("--timestamp-overlay", action="store_true", help="Draw wall-clock timestamp on radar frames.")
    parser.add_argument("--resource-log-interval", type=float, default=1.0)
    parser.add_argument(
        "--metrics-dir",
        type=Path,
        default=Path("/home/radar/fft_live_metrics"),
        help="Directory root for CSV metrics logs. A timestamped subdirectory is created for each run.",
    )
    parser.add_argument("--max-live-frames", type=int, default=0)
    parser.add_argument("--fft-step", type=int, default=200)
    parser.add_argument("--fft-frag", type=int, default=2000)
    parser.add_argument("--display", action="store_true", help="Enable the OpenCV window. Default is video-only.")
    parser.add_argument("--jpg-output-dir", type=Path, default=None)
    return parser.parse_args()


def relaunch_with_sudo_if_needed() -> None:
    if os.name == "posix" and hasattr(os, "geteuid") and os.geteuid() != 0:
        os.execvp("sudo", ["sudo", "-E", sys.executable, *sys.argv])


def write_runtime_fft_config(
    base_config: Path,
    display_enabled: bool,
    fft_frag: int,
    fft_step: int,
    stream_source: str,
) -> Path:
    text = base_config.read_text(encoding="utf-8")
    fd, name = tempfile.mkstemp(prefix="fft_live_dca1000_", suffix=".cfg")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        handle.write(text)
        handle.write("\n# Live DCA1000 launcher overrides\n")
        handle.write(f"stream.source = {stream_source}\n")
        if fft_frag > 0:
            handle.write(f"fft.frag = {fft_frag}\n")
        if fft_step > 0:
            handle.write(f"fft.step = {fft_step}\n")
        handle.write(f"display.enabled = {'true' if display_enabled else 'false'}\n")
        handle.write("imaging.scene.squint_angle_deg = 0\n")
        handle.write("render.low_value_white = false\n")
        handle.write("render.output_gamma = 0.65\n")
    return Path(name)


def run_checked(command: list[str]) -> str:
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip() or f"exit code {completed.returncode}"
        raise RuntimeError(f"{' '.join(command)}: {detail}")
    return completed.stdout.strip()


def prepare_radar_interface(interface: str, cidr: str) -> None:
    run_checked(["ip", "link", "set", interface, "up"])
    run_checked(["ip", "address", "replace", cidr, "dev", interface])
    state = run_checked(["ip", "-brief", "addr", "show", "dev", interface])
    print(f"[INFO][launcher] radar interface ready: {state}", flush=True)


def release_ftdi_kernel_driver() -> None:
    modules = Path("/proc/modules").read_text(encoding="utf-8", errors="replace")
    if "ftdi_sio " not in modules:
        return
    print("[INFO][launcher] unloading ftdi_sio so mmWaveLink can open FTDI board control", flush=True)
    completed = subprocess.run(["modprobe", "-r", "ftdi_sio"], capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip() or f"exit code {completed.returncode}"
        print(f"[WARN][launcher] could not unload ftdi_sio: {detail}", flush=True)
    time.sleep(0.3)


def find_ti_devpack_usb_nodes() -> list[Path]:
    nodes: list[Path] = []
    for vendor_file in Path("/sys/bus/usb/devices").glob("*/idVendor"):
        device_dir = vendor_file.parent
        product_file = device_dir / "idProduct"
        bus_file = device_dir / "busnum"
        dev_file = device_dir / "devnum"
        try:
            vendor = vendor_file.read_text(encoding="ascii").strip().lower()
            product = product_file.read_text(encoding="ascii").strip().lower()
            busnum = int(bus_file.read_text(encoding="ascii").strip())
            devnum = int(dev_file.read_text(encoding="ascii").strip())
        except (OSError, ValueError):
            continue
        if vendor == "0451" and product == "fd03":
            nodes.append(Path(f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"))
    return nodes


def reset_ti_devpack_usb_devices() -> None:
    # USBDEVFS_RESET from linux/usbdevice_fs.h: _IO('U', 20)
    usbdevfs_reset = (ord("U") << 8) | 20
    nodes = find_ti_devpack_usb_nodes()
    if not nodes:
        print("[WARN][launcher] TI AR-DevPack USB 0451:fd03 not found before init", flush=True)
        return
    for node in nodes:
        if not node.exists():
            print(f"[WARN][launcher] TI USB node disappeared before reset: {node}", flush=True)
            continue
        print(f"[INFO][launcher] resetting TI AR-DevPack USB device: {node}", flush=True)
        with node.open("r+b", buffering=0) as handle:
            fcntl.ioctl(handle.fileno(), usbdevfs_reset, 0)
    time.sleep(2.0)


def stop_child_process(proc: subprocess.Popen[str] | None, *, interrupt_first: bool) -> None:
    if proc is None or proc.poll() is not None:
        return
    if interrupt_first and hasattr(os, "killpg"):
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            return
    else:
        proc.terminate()
    try:
        proc.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass

    if hasattr(os, "killpg"):
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
    else:
        proc.terminate()
    try:
        proc.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass

    if hasattr(os, "killpg"):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
    else:
        proc.kill()
    proc.wait(timeout=5)


def make_live_fifo() -> Path:
    fifo_path = Path(tempfile.mktemp(prefix="fft_live_dca1000_raw_", suffix=".fifo"))
    os.mkfifo(fifo_path, 0o600)
    return fifo_path


def write_chunk_to_fifo(
    fifo_handle,
    chunk_data: object,
    expected_bytes: int,
    *,
    allow_empty_raw: bool,
    min_nonzero_fraction: float,
) -> tuple[int, float]:
    if hasattr(chunk_data, "tobytes"):
        data = chunk_data.tobytes()
    else:
        data = bytes(chunk_data)
    if len(data) < expected_bytes:
        raise RuntimeError(f"pyRadar chunk shorter than expected: actual={len(data)}, expected={expected_bytes}")
    if len(data) > expected_bytes:
        print(
            f"[WARN][launcher] trimming pyRadar chunk: actual={len(data)}, expected={expected_bytes}",
            flush=True,
        )
        data = data[:expected_bytes]
    if not allow_empty_raw and not any(data):
        print(
            "[ERROR][launcher] dropped all-zero radar raw chunk; DCA1000/LVDS data stream is not valid",
            flush=True,
        )
        return 0, 0.0
    nonzero_fraction = sum(1 for byte in data if byte != 0) / max(1, len(data))
    if not allow_empty_raw and min_nonzero_fraction > 0.0:
        if nonzero_fraction < min_nonzero_fraction:
            print(
                f"[ERROR][launcher] dropped sparse radar raw chunk; nonzero_fraction={nonzero_fraction:.6f}, "
                f"threshold={min_nonzero_fraction:.6f}. This usually means severe DCA1000 UDP packet loss.",
                flush=True,
            )
            return 0, nonzero_fraction
    fifo_handle.write(data)
    fifo_handle.flush()
    return len(data), nonzero_fraction


def open_metrics_csv(path: Path, header: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w", encoding="utf-8", buffering=1)
    handle.write(header + "\n")
    return handle


def read_system_cpu_times() -> tuple[int, int]:
    fields = Path("/proc/stat").read_text(encoding="ascii").splitlines()[0].split()[1:]
    values = [int(value) for value in fields]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def read_process_usage(pid: int) -> tuple[int, float]:
    stat_fields = Path(f"/proc/{pid}/stat").read_text(encoding="ascii").split()
    utime = int(stat_fields[13])
    stime = int(stat_fields[14])
    rss_pages = int(stat_fields[23])
    rss_mb = rss_pages * os.sysconf("SC_PAGE_SIZE") / 1024.0 / 1024.0
    return utime + stime, rss_mb


def start_resource_monitor(
    output_path: Path,
    fft_pid: int,
    launcher_pid: int,
    run_start_time: float,
    stop_event: threading.Event,
    interval_s: float,
) -> threading.Thread:
    def monitor() -> None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8", buffering=1) as handle:
            handle.write(
                "runtime_s,system_cpu_percent,fft_pid,fft_cpu_percent,fft_rss_mb,"
                "launcher_pid,launcher_cpu_percent,launcher_rss_mb\n"
            )
            try:
                prev_total, prev_idle = read_system_cpu_times()
                prev_fft_cpu, _ = read_process_usage(fft_pid)
                prev_launcher_cpu, _ = read_process_usage(launcher_pid)
            except Exception as exc:
                handle.write(f"0,-1,{fft_pid},-1,-1,{launcher_pid},-1,-1\n")
                print(f"[WARN][launcher] resource monitor disabled: {exc}", flush=True)
                return

            cpu_count = max(1, os.cpu_count() or 1)
            while not stop_event.wait(max(0.1, interval_s)):
                try:
                    total, idle = read_system_cpu_times()
                    fft_cpu, fft_rss_mb = read_process_usage(fft_pid)
                    launcher_cpu, launcher_rss_mb = read_process_usage(launcher_pid)
                except FileNotFoundError:
                    break
                except Exception as exc:
                    print(f"[WARN][launcher] resource monitor sample failed: {exc}", flush=True)
                    continue

                delta_total = max(1, total - prev_total)
                delta_idle = max(0, idle - prev_idle)
                system_cpu_percent = (delta_total - delta_idle) / delta_total * 100.0
                fft_cpu_percent = (fft_cpu - prev_fft_cpu) * cpu_count / delta_total * 100.0
                launcher_cpu_percent = (launcher_cpu - prev_launcher_cpu) * cpu_count / delta_total * 100.0

                handle.write(
                    f"{time.monotonic() - run_start_time:.6f},"
                    f"{system_cpu_percent:.6f},{fft_pid},{fft_cpu_percent:.6f},{fft_rss_mb:.6f},"
                    f"{launcher_pid},{launcher_cpu_percent:.6f},{launcher_rss_mb:.6f}\n"
                )

                prev_total, prev_idle = total, idle
                prev_fft_cpu = fft_cpu
                prev_launcher_cpu = launcher_cpu

    thread = threading.Thread(target=monitor, name="resource-monitor", daemon=True)
    thread.start()
    return thread


def main() -> int:
    args = parse_args()
    relaunch_with_sudo_if_needed()

    metrics_root = args.metrics_dir.expanduser()
    metrics_run_dir = metrics_root / time.strftime("%Y%m%d_%H%M%S")
    frame_metrics_path = metrics_run_dir / "fft_live_frames.csv"
    raw_metrics_path = metrics_run_dir / "raw_chunks.csv"
    resource_metrics_path = metrics_run_dir / "resource_usage.csv"
    video_output_path = (
        args.video_output.expanduser()
        if args.video_output is not None
        else metrics_run_dir / "radar_live.avi"
    )
    raw_metrics = open_metrics_csv(
        raw_metrics_path,
        "runtime_s,chunk_index,requested_frames,expected_bytes,written_bytes,nonzero_fraction,"
        "chunk_interval_s,raw_write_mbps,raw_cumulative_mbps,"
        "received_packets,expected_packets,lost_packets,loss_rate,first_packet,last_packet,"
        "frames_written,total_bytes_written,bad_chunk",
    )
    run_start_time = time.monotonic()
    print(f"[INFO][launcher] metrics_dir={metrics_run_dir}", flush=True)
    print(f"[INFO][launcher] video_output={video_output_path}", flush=True)
    print(f"[INFO][launcher] resource_metrics={resource_metrics_path}", flush=True)

    pyradar_root = args.pyradar_root.expanduser().resolve()
    native_dir = find_native_fpga_udp_dir(pyradar_root)
    if native_dir is not None:
        os.environ["FPGA_UDP_OVERRIDE_DIR"] = str(native_dir)
        sys.path.insert(0, str(native_dir))
    sys.path.insert(0, str(pyradar_root))
    os.chdir(pyradar_root)
    print(f"[INFO][launcher] cwd={Path.cwd()}", flush=True)
    if native_dir is not None:
        print(f"[INFO][launcher] FPGA_UDP_OVERRIDE_DIR={native_dir}", flush=True)

    import fpga_udp
    from mmwave.dataloader import DCA1000

    runtime_config = write_runtime_fft_config(
        args.fft_config.expanduser(),
        args.display,
        args.fft_frag,
        args.fft_step,
        "dca1000" if args.receiver_mode == "direct" else "bin_replay",
    )
    proc: subprocess.Popen[str] | None = None
    dca = None
    fifo_path: Path | None = None
    udp_thread_started = False
    sensor_started = False
    shutdown_requested = False
    resource_stop = threading.Event()
    resource_thread: threading.Thread | None = None

    def request_shutdown(signum: int, _frame: object) -> None:
        nonlocal shutdown_requested, proc
        shutdown_requested = True
        signame = signal.Signals(signum).name
        print(f"\n[INFO][launcher] received {signame}; stopping live capture...", flush=True)
        stop_child_process(proc, interrupt_first=True)

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    try:
        if not args.skip_network_setup:
            prepare_radar_interface(args.radar_interface, args.radar_interface_cidr)
        if not args.keep_ftdi_kernel_driver:
            release_ftdi_kernel_driver()
        if not args.skip_usb_reset:
            reset_ti_devpack_usb_devices()

        print(f"[INFO][launcher] radar_config={args.radar_config}", flush=True)
        print(f"[INFO][launcher] dca_config={args.dca_config}", flush=True)
        dca = DCA1000(static_ip=args.dca_static_ip, adc_ip=args.dca_adc_ip)
        dca.reset_radar()
        dca.reset_fpga()
        time.sleep(1.0)

        if hasattr(fpga_udp, "AWR2243_setSOP"):
            print("[INFO][launcher] pre-latching SOP4 with DCA1000 radar reset", flush=True)
            sop_result = fpga_udp.AWR2243_setSOP(4)
            if isinstance(sop_result, int) and sop_result != 0:
                print(f"[WARN][launcher] AWR2243_setSOP returned {sop_result}", flush=True)
            try:
                dca.reset_radar()
            except Exception as exc:
                print(f"[WARN][launcher] pre-init reset_radar failed: {exc}", flush=True)
            time.sleep(1.0)

        result = fpga_udp.AWR2243_init(str(args.radar_config))
        if isinstance(result, int) and result != 0:
            raise RuntimeError(f"AWR2243_init failed with code {result}")
        requested_radar_frames = int(args.radar_numframes)
        device_radar_frames = 65535 if requested_radar_frames == 0 else requested_radar_frames
        if requested_radar_frames == 0:
            print(
                "[INFO][launcher] continuous mode requested; using legacy frameCfg numFrames=65535 "
                "instead of 0 to avoid short hardware/DFP finite capture",
                flush=True,
            )
        result = fpga_udp.AWR2243_setFrameCfg(device_radar_frames)
        if isinstance(result, int) and result != 0:
            raise RuntimeError(f"AWR2243_setFrameCfg failed with code {result}")

        lvds_bytes, max_send_bytes, adc_params, cfg_params = DCA1000.AWR2243_read_config_compat(str(args.radar_config))
        dca.refresh_parameter()
        print(f"[INFO][launcher] ADC_PARAMS={adc_params}", flush=True)
        print(f"[INFO][launcher] CFG_PARAMS={cfg_params}", flush=True)
        if requested_radar_frames == 0 and float(adc_params.get("frame_periodicity", 0.0)) > 0.0:
            estimated_seconds = device_radar_frames * float(adc_params["frame_periodicity"]) / 1000.0
            print(
                f"[INFO][launcher] device legacy frame limit={device_radar_frames}, "
                f"estimated run before sensor re-arm={estimated_seconds:.1f}s",
                flush=True,
            )
        print(
            f"[INFO][launcher] LVDSDataSizePerChirp={lvds_bytes}, maxSendBytesPerChirp={max_send_bytes}",
            flush=True,
        )
        print(f"[INFO][launcher] System connection check: {dca.sys_alive_check()}", flush=True)
        print(f"[INFO][launcher] FPGA version: {dca.read_fpga_version()}", flush=True)
        print(f"[INFO][launcher] Config fpga: {dca.config_fpga(str(args.dca_config))}", flush=True)
        print(f"[INFO][launcher] Config record: {dca.config_record(str(args.dca_config))}", flush=True)

        if args.receiver_mode == "pyradar":
            fifo_path = make_live_fifo()
            source_args = ["--source-bin", str(fifo_path)]
        else:
            source_args = [
                "--source-dca1000",
                "--dca-bind-ip",
                args.dca_static_ip,
                "--dca-data-port",
                str(args.dca_data_port),
                "--dca-timeout-ms",
                str(args.dca_timeout_ms),
            ]

        cmd = [
            str(args.fft_bin.expanduser()),
            "--config",
            str(runtime_config),
            *source_args,
            "--velocity-mps",
            str(args.velocity_mps),
            "--video-output",
            str(video_output_path),
            "--video-fps",
            str(args.video_fps),
            "--metrics-log",
            str(frame_metrics_path),
        ]
        if args.max_live_frames > 0:
            cmd.extend(["--max-live-frames", str(args.max_live_frames)])
        if args.mjpeg_port > 0:
            cmd.extend(["--mjpeg-port", str(args.mjpeg_port), "--mjpeg-quality", str(args.mjpeg_quality)])
        if args.timestamp_overlay:
            cmd.append("--timestamp-overlay")
        if args.jpg_output_dir is not None:
            cmd.extend(["--jpg-output-dir", str(args.jpg_output_dir.expanduser())])

        if args.receiver_mode == "direct":
            try:
                dca.data_socket.close()
                print("[INFO][launcher] released pyRadar data socket before fft_imaging_live bind", flush=True)
            except Exception as exc:
                print(f"[WARN][launcher] could not close pyRadar data socket before live bind: {exc}", flush=True)

        print("[INFO][launcher] starting fft_imaging_live:", " ".join(cmd), flush=True)
        proc = subprocess.Popen(cmd, text=True, start_new_session=True)
        if args.resource_log_interval > 0.0:
            resource_thread = start_resource_monitor(
                resource_metrics_path,
                proc.pid,
                os.getpid(),
                run_start_time,
                resource_stop,
                float(args.resource_log_interval),
            )
        time.sleep(max(0.0, args.receiver_warmup_seconds))
        if proc.poll() is not None:
            raise RuntimeError(f"fft_imaging_live exited before sensorStart with code {proc.returncode}")
        if shutdown_requested:
            return 130

        dca.stream_start()
        print("[INFO][launcher] DCA1000 stream_start sent", flush=True)
        if args.receiver_mode == "direct":
            try:
                dca.config_socket.close()
            except Exception:
                pass
            dca = None
        else:
            buffer_frames = max(int(args.pyradar_buffer_frames), int(args.pyradar_chunk_frames))
            print(
                f"[INFO][launcher] starting pyRadar UDP thread: chunk_frames={args.pyradar_chunk_frames}, "
                f"buffer_frames={buffer_frames}",
                flush=True,
            )
            dca.fastRead_in_Cpp_thread_start(buffer_frames)
            udp_thread_started = True

        print("[INFO][launcher] AWR2243_sensorStart", flush=True)
        result = fpga_udp.AWR2243_sensorStart()
        if isinstance(result, int) and result != 0:
            raise RuntimeError(f"AWR2243_sensorStart failed with code {result}")
        sensor_started = True

        exit_code = 130
        if args.receiver_mode == "pyradar":
            chunk_frames = max(1, int(args.pyradar_chunk_frames))
            requested_frames = int(args.radar_numframes)
            frame_bytes = int(adc_params["frame_bytes"])
            frames_written = 0
            bytes_written = 0
            consecutive_bad_chunks = 0
            chunk_index = 0
            last_chunk_time = time.monotonic()
            print(f"[INFO][launcher] opening live raw FIFO for pyRadar bridge: {fifo_path}", flush=True)
            with fifo_path.open("wb", buffering=1024 * 1024) as fifo_handle:
                while proc.poll() is None and not shutdown_requested:
                    if requested_frames > 0:
                        remaining = requested_frames - frames_written
                        if remaining <= 0:
                            break
                        this_frames = min(chunk_frames, remaining)
                    else:
                        this_frames = chunk_frames
                    chunk_data = dca.fastRead_in_Cpp_thread_get(
                        numframes=this_frames,
                        timeOut=2,
                        verbose=True,
                        sortInC=bool(args.pyradar_sort_in_c),
                    )
                    try:
                        expected_packets = int(fpga_udp.get_expectedPacketNum())
                        first_packet = int(fpga_udp.get_firstPacketNum())
                        last_packet = int(fpga_udp.get_lastPacketNum())
                        if args.pyradar_sort_in_c:
                            received_packets = int(fpga_udp.get_receivedPacketNum())
                        else:
                            # Python packet sorting prints accurate packet counts internally,
                            # but the C-side getter remains zero in this mode.
                            received_packets = -1
                    except Exception:
                        received_packets = -1
                        expected_packets = -1
                        first_packet = -1
                        last_packet = -1
                    try:
                        chunk_bytes, nonzero_fraction = write_chunk_to_fifo(
                            fifo_handle,
                            chunk_data,
                            this_frames * frame_bytes,
                            allow_empty_raw=bool(args.allow_empty_raw),
                            min_nonzero_fraction=float(args.min_raw_nonzero_fraction),
                        )
                    except BrokenPipeError:
                        print("[INFO][launcher] fft_imaging_live closed FIFO; stopping pyRadar bridge", flush=True)
                        break
                    lost_packets = (
                        max(0, expected_packets - received_packets)
                        if expected_packets >= 0 and received_packets >= 0
                        else -1
                    )
                    loss_rate = (
                        lost_packets / expected_packets
                        if expected_packets > 0 and lost_packets >= 0
                        else -1.0
                    )
                    bad_chunk = 1 if chunk_bytes == 0 else 0
                    frames_written_after = frames_written + (this_frames if chunk_bytes > 0 else 0)
                    bytes_written_after = bytes_written + chunk_bytes
                    now = time.monotonic()
                    runtime_s = now - run_start_time
                    chunk_interval_s = max(1e-9, now - last_chunk_time)
                    raw_write_mbps = chunk_bytes * 8.0 / chunk_interval_s / 1_000_000.0
                    raw_cumulative_mbps = bytes_written_after * 8.0 / max(1e-9, runtime_s) / 1_000_000.0
                    last_chunk_time = now
                    raw_metrics.write(
                        f"{runtime_s:.6f},{chunk_index},{this_frames},"
                        f"{this_frames * frame_bytes},{chunk_bytes},{nonzero_fraction:.8f},"
                        f"{chunk_interval_s:.6f},{raw_write_mbps:.6f},{raw_cumulative_mbps:.6f},"
                        f"{received_packets},{expected_packets},{lost_packets},{loss_rate:.8f},"
                        f"{first_packet},{last_packet},{frames_written_after},{bytes_written_after},{bad_chunk}\n"
                    )
                    chunk_index += 1
                    if chunk_bytes == 0:
                        consecutive_bad_chunks += 1
                        if args.max_bad_raw_chunks > 0 and consecutive_bad_chunks >= args.max_bad_raw_chunks:
                            raise RuntimeError(
                                "DCA1000 raw stream became invalid for "
                                f"{consecutive_bad_chunks} consecutive chunks. "
                                "The radar data rate is likely too high for sustained 1GbE capture; "
                                "use a lower-fps radar config such as 30f128 or reduce chirps/samples/RX."
                            )
                        if requested_frames > 0:
                            frames_written += this_frames
                        continue
                    consecutive_bad_chunks = 0
                    frames_written += this_frames
                    bytes_written += chunk_bytes
                    print(
                        f"[INFO][launcher] pyRadar chunk frames={this_frames}, bytes={chunk_bytes}, "
                        f"total_frames={frames_written}, total_bytes={bytes_written}",
                        flush=True,
                    )
            if proc.poll() is None:
                try:
                    exit_code = proc.wait(timeout=30 if requested_frames > 0 else 1)
                except subprocess.TimeoutExpired:
                    stop_child_process(proc, interrupt_first=True)
            if proc.poll() is not None:
                exit_code = proc.returncode
        else:
            while proc.poll() is None:
                if shutdown_requested:
                    stop_child_process(proc, interrupt_first=True)
                    break
                try:
                    exit_code = proc.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    continue
        if proc.poll() is not None:
            exit_code = proc.returncode
        proc = None
        print(f"[INFO][launcher] fft_imaging_live exited with code {exit_code}", flush=True)

        if shutdown_requested:
            return 130
        if int(args.radar_numframes) == 0:
            print("[INFO][launcher] live imaging exited during continuous radar mode; stopping sensor", flush=True)
            try:
                fpga_udp.AWR2243_sensorStop()
            except Exception as exc:
                print(f"[WARN][launcher] AWR2243_sensorStop skipped/failed: {exc}", flush=True)
        else:
            try:
                fpga_udp.AWR2243_waitSensorStop()
            except Exception as exc:
                print(f"[WARN][launcher] AWR2243_waitSensorStop skipped/failed: {exc}", flush=True)
        return int(exit_code)
    finally:
        resource_stop.set()
        if resource_thread is not None:
            try:
                resource_thread.join(timeout=3)
            except Exception:
                pass
        stop_child_process(proc, interrupt_first=shutdown_requested)
        if udp_thread_started:
            try:
                dca.fastRead_in_Cpp_thread_stop()
            except Exception:
                pass
        try:
            if sensor_started:
                fpga_udp.AWR2243_sensorStop()
        except Exception:
            pass
        try:
            stopper = DCA1000(static_ip=args.dca_static_ip, adc_ip=args.dca_adc_ip)
            try:
                stopper.stream_stop()
            finally:
                stopper.close()
        except Exception:
            pass
        if dca is not None:
            try:
                dca.close()
            except Exception:
                pass
        try:
            fpga_udp.AWR2243_poweroff()
        except Exception:
            pass
        try:
            runtime_config.unlink()
        except Exception:
            pass
        if fifo_path is not None:
            try:
                fifo_path.unlink()
            except Exception:
                pass
        try:
            raw_metrics.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
