import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from svibspec_simulator import (
    FFTTYPE,
    HyperParamAndSignal,
    Window,
    calc_decay_ratio,
    find_local_maxima_manual,
    freq_estimate,
    freq_hilbert_envelope,
    local_fft_api,
    sensor_noise_freq_metric,
    stabilize_envelope_edges,
)
from vibrating_wire_simulator import (
    ElectromagneticVibratingWireSensorSimulator,
    IdealVibratingWireSensorSimulator,
    VibratingWireSensorSimulator,
)


@dataclass
class StaticMeasurementResult:
    full_time: np.ndarray
    full_excitation: np.ndarray
    full_sensor_signal: np.ndarray
    captured_time: np.ndarray
    clean_signal: np.ndarray
    measured_signal: np.ndarray
    fft_frequency: np.ndarray
    fft_magnitude: np.ndarray
    sensor_frequency: float
    sensor_amplitude: float
    noise_frequency: float
    noise_amplitude: float
    sensor_to_noise_ratio: float
    decay_ratio: float
    analysis_signal: np.ndarray
    window_value: np.ndarray
    digital_signal: np.ndarray
    unshifted_fft_magnitude: np.ndarray
    signal_envelope: np.ndarray


@dataclass
class DynamicMeasurementResult:
    cycle_index: int
    excitation_frequency: float
    full_time: np.ndarray
    full_excitation: np.ndarray
    full_sensor_signal: np.ndarray
    captured_time: np.ndarray
    clean_signal: np.ndarray
    measured_signal: np.ndarray
    fft_frequency: np.ndarray
    fft_magnitude: np.ndarray
    sensor_frequency: float
    sensor_amplitude: float
    noise_frequency: float
    noise_amplitude: float
    sensor_to_noise_ratio: float
    decay_ratio: float
    analysis_signal: np.ndarray
    window_value: np.ndarray
    digital_signal: np.ndarray
    unshifted_fft_magnitude: np.ndarray
    signal_envelope: np.ndarray


class DataloggerSimulator:
    def __init__(
        self,
        sensor: VibratingWireSensorSimulator,
        raw_sample_rate: float = 6_500_000.0,
        analysis_sample_rate: float = 65_000.0,
        fft_points: int = 16_384,
        capture_delay: float = 0.001,
        gaussian_noise_std: float = 0.002,
        interference_frequency: float = 50.0,
        interference_amplitude: float = 0.02,
    ) -> None:
        if fft_points % 1024 != 0:
            raise ValueError("fft_points must be a multiple of 1024 for local_fft_api.")

        self.sensor = sensor
        self.raw_sample_rate = raw_sample_rate
        self.analysis_sample_rate = analysis_sample_rate
        self.fft_points = fft_points
        self.capture_delay = capture_delay
        self.gaussian_noise_std = gaussian_noise_std
        self.interference_frequency = interference_frequency
        self.interference_amplitude = interference_amplitude
        self.rng = np.random.default_rng(13)
        self.analysis_fig = None
        self.analysis_axes = None

        downsample_factor = raw_sample_rate / analysis_sample_rate
        if not np.isclose(downsample_factor, round(downsample_factor)):
            raise ValueError("raw_sample_rate must be an integer multiple of analysis_sample_rate.")
        self.downsample_factor = int(round(downsample_factor))

    def run_static_full_measurement(
        self,
        sweep_start: float,
        sweep_end: float,
        sweep_duration: float,
        sweep_peak_to_peak_voltage: float,
    ) -> StaticMeasurementResult:
        post_excitation_duration = self.capture_delay + self.fft_points / self.analysis_sample_rate
        simulation = self.sensor.simulate_sweep(
            start_frequency=sweep_start,
            end_frequency=sweep_end,
            sweep_duration=sweep_duration,
            sample_rate=self.raw_sample_rate,
            amplitude=sweep_peak_to_peak_voltage / 2.0,
            output_duration=post_excitation_duration,
        )

        capture_start_index = int(round((sweep_duration + self.capture_delay) * self.raw_sample_rate))
        raw_capture_points = self.fft_points * self.downsample_factor
        capture_end_index = capture_start_index + raw_capture_points
        if capture_end_index > len(simulation.sensor_signal):
            raise RuntimeError("Simulation output is shorter than the requested FFT capture window.")

        raw_clean_signal = simulation.sensor_signal[capture_start_index:capture_end_index]
        clean_signal = raw_clean_signal[:: self.downsample_factor][: self.fft_points]
        captured_time = np.arange(self.fft_points) / self.analysis_sample_rate
        captured_time = captured_time - captured_time[0]
        measured_signal = self._add_measurement_noise(captured_time, clean_signal)

        analysis = self._analyze_frequency_and_noise(measured_signal)
        return StaticMeasurementResult(
            full_time=simulation.time,
            full_excitation=simulation.excitation,
            full_sensor_signal=simulation.sensor_signal,
            captured_time=captured_time,
            clean_signal=clean_signal,
            measured_signal=measured_signal,
            fft_frequency=analysis["fft_frequency"],
            fft_magnitude=analysis["fft_magnitude"],
            sensor_frequency=analysis["sensor_frequency"],
            sensor_amplitude=analysis["sensor_amplitude"],
            noise_frequency=analysis["noise_frequency"],
            noise_amplitude=analysis["noise_amplitude"],
            sensor_to_noise_ratio=analysis["sensor_to_noise_ratio"],
            decay_ratio=analysis["decay_ratio"],
            analysis_signal=analysis["analysis_signal"],
            window_value=analysis["window_value"],
            digital_signal=analysis["digital_signal"],
            unshifted_fft_magnitude=analysis["unshifted_fft_magnitude"],
            signal_envelope=analysis["signal_envelope"],
        )

    def run_dynamic_measurements(
        self,
        sweep_start: float,
        sweep_end: float,
        sweep_duration: float,
        sweep_peak_to_peak_voltage: float,
        excitation_peak_to_peak_voltage: float,
        excitation_cycles: float,
        cycle_count: int,
        startup_output_duration: float,
        cycle_period: float,
    ):
        fft_capture_duration = self.fft_points / self.analysis_sample_rate
        post_excitation_duration = self.capture_delay + fft_capture_duration
        dt = 1.0 / self.raw_sample_rate

        startup = self.sensor.simulate_sweep(
            start_frequency=sweep_start,
            end_frequency=sweep_end,
            sweep_duration=sweep_duration,
            sample_rate=self.raw_sample_rate,
            amplitude=sweep_peak_to_peak_voltage / 2.0,
            output_duration=startup_output_duration,
        )
        history_time = startup.time.copy()
        history_excitation = startup.excitation.copy()
        history_sensor_signal = startup.sensor_signal.copy()

        excitation_frequency = self.sensor.resonant_frequency
        next_segment_start = history_time[-1] + dt if len(history_time) > 0 else 0.0

        for cycle_index in range(1, cycle_count + 1):
            cycle_start_time = next_segment_start
            excitation_duration = excitation_cycles / max(excitation_frequency, 1.0)
            segment = self.sensor.simulate_phase_aligned_single_frequency(
                frequency=excitation_frequency,
                duration=excitation_duration,
                sample_rate=self.raw_sample_rate,
                amplitude=excitation_peak_to_peak_voltage / 2.0,
                output_duration=post_excitation_duration,
            )
            segment_time = segment.time + next_segment_start
            next_segment_start = segment_time[-1] + dt

            history_time = np.concatenate((history_time, segment_time))
            history_excitation = np.concatenate((history_excitation, segment.excitation))
            history_sensor_signal = np.concatenate((history_sensor_signal, segment.sensor_signal))

            capture_start_time = excitation_duration + self.capture_delay
            capture_start_index = int(round(capture_start_time * self.raw_sample_rate))
            raw_capture_points = self.fft_points * self.downsample_factor
            capture_end_index = capture_start_index + raw_capture_points
            if capture_end_index > len(segment.sensor_signal):
                raise RuntimeError("Dynamic segment is shorter than the requested FFT capture window.")

            raw_clean_signal = segment.sensor_signal[capture_start_index:capture_end_index]
            clean_signal = raw_clean_signal[:: self.downsample_factor][: self.fft_points]
            captured_time = np.arange(self.fft_points) / self.analysis_sample_rate
            measured_signal = self._add_measurement_noise(
                captured_time + segment_time[0] + capture_start_time,
                clean_signal,
            )
            analysis = self._analyze_frequency_and_noise(measured_signal)

            result = DynamicMeasurementResult(
                cycle_index=cycle_index,
                excitation_frequency=excitation_frequency,
                full_time=history_time.copy(),
                full_excitation=history_excitation.copy(),
                full_sensor_signal=history_sensor_signal.copy(),
                captured_time=captured_time,
                clean_signal=clean_signal,
                measured_signal=measured_signal,
                fft_frequency=analysis["fft_frequency"],
                fft_magnitude=analysis["fft_magnitude"],
                sensor_frequency=analysis["sensor_frequency"],
                sensor_amplitude=analysis["sensor_amplitude"],
                noise_frequency=analysis["noise_frequency"],
                noise_amplitude=analysis["noise_amplitude"],
                sensor_to_noise_ratio=analysis["sensor_to_noise_ratio"],
                decay_ratio=analysis["decay_ratio"],
                analysis_signal=analysis["analysis_signal"],
                window_value=analysis["window_value"],
                digital_signal=analysis["digital_signal"],
                unshifted_fft_magnitude=analysis["unshifted_fft_magnitude"],
                signal_envelope=analysis["signal_envelope"],
            )
            yield result

            if result.sensor_frequency > 0.0:
                excitation_frequency = result.sensor_frequency

            if cycle_index < cycle_count:
                elapsed_cycle_time = next_segment_start - cycle_start_time
                idle_duration = max(0.0, cycle_period - elapsed_cycle_time)
                idle_segment = self._simulate_idle(idle_duration)
                if idle_segment is not None:
                    idle_time = idle_segment.time + next_segment_start
                    next_segment_start = idle_time[-1] + dt
                    history_time = np.concatenate((history_time, idle_time))
                    history_excitation = np.concatenate((history_excitation, idle_segment.excitation))
                    history_sensor_signal = np.concatenate((history_sensor_signal, idle_segment.sensor_signal))

    def _add_measurement_noise(self, time: np.ndarray, signal: np.ndarray) -> np.ndarray:
        gaussian_noise = self.rng.normal(0.0, self.gaussian_noise_std, size=len(signal))
        single_frequency_interference = self.interference_amplitude * np.sin(
            2.0 * np.pi * self.interference_frequency * time
        )
        return signal + gaussian_noise + single_frequency_interference

    def _simulate_idle(self, duration: float):
        if duration <= 0.0:
            return None
        idle_samples = int(round(duration * self.raw_sample_rate))
        if idle_samples <= 0:
            return None
        idle_excitation = np.zeros(idle_samples)
        return self.sensor.simulate(
            excitation=idle_excitation,
            sample_rate=self.raw_sample_rate,
            output_duration=-1.0,
        )

    def _analyze_frequency_and_noise(self, signal: np.ndarray) -> dict[str, float | np.ndarray]:
        hps = HyperParamAndSignal(
            freq_sample=int(self.analysis_sample_rate),
            n_sample=self.fft_points,
            decay_rate=0.0,
            white_noise_level=0.0,
        )
        voltage_reference = 5.0
        adc_bits = 12
        truncation_points = min(16, max(1, self.fft_points // 8))
        win_type = Window.RECT

        osignal_quant, digital_signal = hps.adc_quantize(signal, voltage_reference, adc_bits)
        windowed_signal, window_value = hps.apply_window(osignal_quant, win_type)
        fft_result = local_fft_api(
            self.fft_points,
            windowed_signal,
            FT=FFTTYPE.FFT,
        )
        fft_real = fft_result.real
        fft_imag = fft_result.imag
        fft_magnitude = np.sqrt(fft_real**2 + fft_imag**2) * (2.0 / self.fft_points)
        fft_frequency = np.fft.fftfreq(self.fft_points, d=1.0 / self.analysis_sample_rate)

        edge_samples = min(
            self.fft_points // 4,
            max(
                truncation_points,
                int(round(8.0 * self.analysis_sample_rate / max(self.sensor.resonant_frequency, 1.0))),
            ),
        )
        signal_envelope = stabilize_envelope_edges(
            freq_hilbert_envelope(osignal_quant, self.fft_points),
            edge_samples=edge_samples,
        )
        log_envelope = np.log(np.maximum(signal_envelope, 1.0e-12))
        time_points = np.linspace(0, hps.DT(), hps.N(), endpoint=False)
        decay_ratio = calc_decay_ratio(
            hps.N() - 2 * edge_samples,
            log_envelope[edge_samples:-edge_samples],
            time_points[edge_samples:-edge_samples],
        ) * (-1.0)

        peak_positions = self._find_candidate_peaks(fft_magnitude)
        detect_result = []
        for position in peak_positions:
            if position == 0 or position == self.fft_points // 2:
                continue
            direct_freq, candan_freq, amplitude = freq_estimate(
                int(self.analysis_sample_rate),
                self.fft_points,
                int(position),
                fft_real,
                fft_imag,
                osignal_quant,
                win_type,
            )
            detect_result.append(
                {
                    "pos": int(position),
                    "val": float(amplitude),
                    "direct_detect_freq": float(direct_freq),
                    "candan_detect_freq": float(candan_freq),
                }
            )

        if detect_result:
            (
                sensor_frequency,
                sensor_amplitude,
                noise_frequency,
                noise_amplitude,
                sensor_to_noise_ratio,
            ) = sensor_noise_freq_metric(detect_result)
        else:
            sensor_frequency = 0.0
            sensor_amplitude = 0.0
            noise_frequency = 0.0
            noise_amplitude = 0.0
            sensor_to_noise_ratio = 0.0

        decay_samples = np.abs(np.exp(-decay_ratio * time_points))
        decay_factor = np.sum(np.abs(window_value) * decay_samples) / np.sum(np.abs(window_value))
        sensor_amplitude = float(sensor_amplitude / max(decay_factor, 1.0e-12))

        analysis_result = {
            "fft_frequency": np.fft.fftshift(fft_frequency),
            "fft_magnitude": np.fft.fftshift(fft_magnitude),
            "unshifted_fft_magnitude": fft_magnitude,
            "sensor_frequency": sensor_frequency,
            "sensor_amplitude": sensor_amplitude,
            "noise_frequency": noise_frequency,
            "noise_amplitude": noise_amplitude,
            "sensor_to_noise_ratio": sensor_to_noise_ratio,
            "decay_ratio": decay_ratio,
            "analysis_signal": windowed_signal,
            "window_value": window_value,
            "digital_signal": digital_signal,
            "signal_envelope": signal_envelope,
        }
        # self._plot_analysis_result(analysis_result)
        return analysis_result

    def _plot_analysis_result(self, analysis_result: dict[str, float | np.ndarray]) -> None:
        if self.analysis_fig is None or self.analysis_axes is None:
            self.analysis_fig, self.analysis_axes = plt.subplots(3, 2, figsize=(14, 10))
            self.analysis_fig.canvas.manager.set_window_title("Frequency analysis details")

        axes = self.analysis_axes.ravel()
        for axis in axes:
            axis.clear()

        time_points = np.arange(self.fft_points) / self.analysis_sample_rate
        bin_index = np.arange(self.fft_points)
        fft_frequency = analysis_result["fft_frequency"]
        fft_magnitude = np.maximum(analysis_result["fft_magnitude"], 1.0e-12)
        unshifted_fft_magnitude = np.maximum(analysis_result["unshifted_fft_magnitude"], 1.0e-12)

        axes[0].plot(time_points, analysis_result["analysis_signal"], linewidth=0.9)
        axes[0].set_title("Windowed analysis signal")
        axes[0].set_xlabel("Time (s)")
        axes[0].set_ylabel("Amplitude")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(time_points, analysis_result["window_value"], linewidth=0.9)
        axes[1].set_title("Window value")
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Weight")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(time_points, analysis_result["digital_signal"], linewidth=0.9)
        axes[2].set_title("ADC digital signal")
        axes[2].set_xlabel("Time (s)")
        axes[2].set_ylabel("Code")
        axes[2].grid(True, alpha=0.3)

        axes[3].plot(time_points, analysis_result["signal_envelope"], linewidth=1.0)
        axes[3].set_title(f"Hilbert envelope, decay={analysis_result['decay_ratio']:.6f}")
        axes[3].set_xlabel("Time (s)")
        axes[3].set_ylabel("Envelope")
        axes[3].grid(True, alpha=0.3)

        axes[4].plot(fft_frequency, fft_magnitude, linewidth=1.0)
        axes[4].axvline(analysis_result["sensor_frequency"], color="C2", linestyle="--", label="Sensor")
        axes[4].axvline(-analysis_result["sensor_frequency"], color="C2", linestyle="--")
        if analysis_result["noise_frequency"] > 0.0:
            axes[4].axvline(analysis_result["noise_frequency"], color="red", linestyle="--", label="Noise")
            axes[4].axvline(-analysis_result["noise_frequency"], color="red", linestyle="--")
        axes[4].set_title(
            f"Double-sided spectrum, sensor={analysis_result['sensor_frequency']:.6f} Hz"
        )
        axes[4].set_xlabel("Frequency (Hz)")
        axes[4].set_ylabel("Amplitude")
        axes[4].set_yscale("log")
        axes[4].set_xlim(-5000.0, 5000.0)
        axes[4].grid(True, alpha=0.3)
        axes[4].legend(loc="upper right")

        axes[5].plot(bin_index, unshifted_fft_magnitude, linewidth=1.0)
        axes[5].set_title(
            f"Unshifted FFT magnitude, SNR={analysis_result['sensor_to_noise_ratio']:.2f} dB"
        )
        axes[5].set_xlabel("FFT bin")
        axes[5].set_ylabel("Amplitude")
        axes[5].set_yscale("log")
        axes[5].grid(True, alpha=0.3)

        self.analysis_fig.tight_layout()
        self.analysis_fig.canvas.draw_idle()
        self.analysis_fig.canvas.flush_events()
        plt.pause(0.05)

    def _find_candidate_peaks(self, fft_magnitude: np.ndarray) -> list[int]:
        threshold = max(float(np.max(fft_magnitude)) * 0.05, 1.0e-12)
        try:
            peak_positions = find_local_maxima_manual(
                self.fft_points,
                fft_magnitude,
                th_val=threshold,
            )
        except IndexError:
            peak_positions = []

        if len(peak_positions) >= 1:
            return [int(pos) for pos in peak_positions[:2]]

        half_spectrum = fft_magnitude[1 : self.fft_points // 2]
        fallback_positions = np.argsort(half_spectrum)[-2:][::-1] + 1
        return [int(pos) for pos in fallback_positions]


def plot_static_measurement(result: StaticMeasurementResult, output: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 11))

    axes[0].plot(result.full_time, result.full_excitation, color="red", linewidth=1.0, label="Sweep excitation")
    axes[0].plot(result.full_time, result.full_sensor_signal, color="C0", linewidth=1.1, label="Sensor output")
    axes[0].set_title("Original sweep excitation and vibrating wire sensor output")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Amplitude")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(result.captured_time, result.measured_signal, linewidth=0.9, label="Captured signal with noise")
    axes[1].set_title("Captured signal after sweep excitation")
    axes[1].set_xlabel("Time after capture start (s)")
    axes[1].set_ylabel("Amplitude")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(result.fft_frequency, np.maximum(result.fft_magnitude, 1.0e-12), linewidth=1.1)
    axes[2].axvline(result.sensor_frequency, color="C2", linestyle="--", label=f"Sensor: {result.sensor_frequency:.6f} Hz")
    axes[2].axvline(-result.sensor_frequency, color="C2", linestyle="--")
    if result.noise_frequency > 0.0:
        axes[2].axvline(result.noise_frequency, color="red", linestyle="--", label=f"Noise: {result.noise_frequency:.6f} Hz")
        axes[2].axvline(-result.noise_frequency, color="red", linestyle="--")
    axes[2].set_title(
        "Algorithm analysis result "
        f"(SNR={result.sensor_to_noise_ratio:.2f} dB)"
    )
    axes[2].set_xlabel("Frequency (Hz)")
    axes[2].set_ylabel("Amplitude")
    axes[2].set_yscale("log")
    axes[2].set_xlim(-5000.0, 5000.0)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(output, dpi=220)
    plt.show()


def draw_dynamic_measurement(axes: np.ndarray, result: DynamicMeasurementResult) -> None:
    for axis in axes:
        axis.clear()
    axes[0].plot(result.full_time, result.full_excitation, color="red", linewidth=1.0, label="Excitation")
    axes[0].plot(result.full_time, result.full_sensor_signal, color="C0", linewidth=1.1, label="Sensor output")
    axes[0].set_title(
        "Dynamic vibrating wire simulation "
        f"(cycle {result.cycle_index}, excitation={result.excitation_frequency:.6f} Hz)"
    )
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Amplitude")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(result.captured_time, result.measured_signal, linewidth=0.9, label="Captured signal with noise")
    axes[1].set_title("Latest captured signal after phase-aligned excitation")
    axes[1].set_xlabel("Time after capture start (s)")
    axes[1].set_ylabel("Amplitude")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(result.fft_frequency, np.maximum(result.fft_magnitude, 1.0e-12), linewidth=1.1)
    axes[2].axvline(result.sensor_frequency, color="C2", linestyle="--", label=f"Sensor: {result.sensor_frequency:.6f} Hz")
    axes[2].axvline(-result.sensor_frequency, color="C2", linestyle="--")
    if result.noise_frequency > 0.0:
        axes[2].axvline(result.noise_frequency, color="red", linestyle="--", label=f"Noise: {result.noise_frequency:.6f} Hz")
        axes[2].axvline(-result.noise_frequency, color="red", linestyle="--")
    axes[2].set_title(
        "Latest algorithm analysis result "
        f"(SNR={result.sensor_to_noise_ratio:.2f} dB)"
    )
    axes[2].set_xlabel("Frequency (Hz)")
    axes[2].set_ylabel("Amplitude")
    axes[2].set_yscale("log")
    axes[2].set_xlim(-5000.0, 5000.0)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")


def plot_dynamic_measurement(result: DynamicMeasurementResult, output: Path, pause_seconds: float = 0.05) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 11))
    draw_dynamic_measurement(axes, result)
    fig.tight_layout()
    fig.savefig(output, dpi=220)
    plt.pause(pause_seconds)


class DynamicMeasurementPlotter:
    def __init__(self, output: Path, pause_seconds: float) -> None:
        self.output = output
        self.pause_seconds = pause_seconds
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 11))
        self._brought_to_front = False
        plt.show(block=False)

    def update(self, result: DynamicMeasurementResult) -> None:
        draw_dynamic_measurement(self.axes, result)
        self.fig.tight_layout()
        self.fig.savefig(self.output, dpi=220)
        if not self._brought_to_front:
            self._bring_to_front()
            self._brought_to_front = True
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(self.pause_seconds)

    def _bring_to_front(self) -> None:
        manager = getattr(self.fig.canvas, "manager", None)
        window = getattr(manager, "window", None)
        try:
            if hasattr(window, "lift"):
                window.lift()
            if hasattr(window, "attributes"):
                window.attributes("-topmost", True)
                window.attributes("-topmost", False)
            if hasattr(window, "raise_"):
                window.raise_()
            if hasattr(window, "activateWindow"):
                window.activateWindow()
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Datalogger vibrating wire measurement simulator.")

    # Simulation flow selection.
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--static", action="store_true", help="Run the static_vibsim flow.")
    mode_group.add_argument("--dynamic", action="store_true", help="Run the dynamic_vibsim flow.")

    # Vibrating wire mechanical model parameters.
    parser.add_argument("--q-value", type=float, default=1200.0)
    parser.add_argument("--resonant-frequency", type=float, default=2500.0)
    parser.add_argument("--static-resistance", type=float, default=150.0)
    parser.add_argument("--decay-coefficient", type=float, default=0.2)
    parser.add_argument(
        "--sensor-model",
        choices=("second_order", "electromagnetic", "ideal"),
        default="electromagnetic",
        help="Sensor simulation model.",
    )

    # Electromagnetic coupling model parameters.
    parser.add_argument("--equivalent-mass", type=float, default=1.0e-4)
    parser.add_argument("--coil-resistance", type=float, default=150.0)
    parser.add_argument("--coil-inductance", type=float, default=0.02)
    parser.add_argument("--force-constant", type=float, default=0.5)
    parser.add_argument("--back-emf-constant", type=float, default=0.5)
    parser.add_argument("--electromagnetic-output-gain", type=float, default=10.0)

    # Sampling and algorithm analysis parameters.
    parser.add_argument("--raw-sample-rate", type=float, default=6_500_000.0)
    parser.add_argument("--analysis-sample-rate", type=float, default=65_000.0)
    parser.add_argument("--fft-points", type=int, default=16_384)
    parser.add_argument("--capture-delay", type=float, default=0.005)

    # Startup sweep excitation parameters.
    parser.add_argument("--sweep-start", type=float, default=1400.0)
    parser.add_argument("--sweep-end", type=float, default=3500.0)
    parser.add_argument("--sweep-duration", type=float, default=0.02)
    parser.add_argument("--sweep-peak-to-peak-voltage", type=float, default=20.0)

    # Dynamic tracking and phase-aligned re-excitation parameters.
    parser.add_argument("--dynamic-cycles", type=int, default=4)
    parser.add_argument("--dynamic-excitation-cycles", type=float, default=3.0)
    parser.add_argument("--dynamic-excitation-peak-to-peak-voltage", type=float, default=20.0)
    parser.add_argument(
        "--dynamic-startup-output-duration",
        type=float,
        default=0.35,
        help="Free-decay interval after the startup sweep before the first phase-aligned excitation.",
    )
    parser.add_argument("--dynamic-cycle-period", type=float, default=0.35)
    parser.add_argument("--dynamic-plot-pause", type=float, default=0.8)

    # Measurement noise, interference, and output parameters.
    parser.add_argument("--gaussian-noise-std", type=float, default=0.01)
    parser.add_argument("--interference-frequency", type=float, default=50.0)
    parser.add_argument("--interference-amplitude", type=float, default=0.05)
    parser.add_argument("--output", type=Path, default=Path("static_vibsim.png"))
    return parser.parse_args()


def build_datalogger(args: argparse.Namespace) -> DataloggerSimulator:
    if args.sensor_model == "ideal":
        sensor = IdealVibratingWireSensorSimulator(
            q_value=args.q_value,
            resonant_frequency=args.resonant_frequency,
            static_resistance=args.static_resistance,
            decay_coefficient=args.decay_coefficient,
            output_gain=args.electromagnetic_output_gain,
        )
    elif args.sensor_model == "electromagnetic":
        sensor = ElectromagneticVibratingWireSensorSimulator(
            q_value=args.q_value,
            resonant_frequency=args.resonant_frequency,
            static_resistance=args.static_resistance,
            decay_coefficient=args.decay_coefficient,
            equivalent_mass=args.equivalent_mass,
            coil_resistance=args.coil_resistance,
            coil_inductance=args.coil_inductance,
            force_constant=args.force_constant,
            back_emf_constant=args.back_emf_constant,
            output_gain=args.electromagnetic_output_gain,
        )
    else:
        sensor = VibratingWireSensorSimulator(
            q_value=args.q_value,
            resonant_frequency=args.resonant_frequency,
            static_resistance=args.static_resistance,
            decay_coefficient=args.decay_coefficient,
        )
    datalogger = DataloggerSimulator(
        sensor=sensor,
        raw_sample_rate=args.raw_sample_rate,
        analysis_sample_rate=args.analysis_sample_rate,
        fft_points=args.fft_points,
        capture_delay=args.capture_delay,
        gaussian_noise_std=args.gaussian_noise_std,
        interference_frequency=args.interference_frequency,
        interference_amplitude=args.interference_amplitude,
    )
    return datalogger


def static_vibsim(args: argparse.Namespace) -> None:
    datalogger = build_datalogger(args)
    result = datalogger.run_static_full_measurement(
        sweep_start=args.sweep_start,
        sweep_end=args.sweep_end,
        sweep_duration=args.sweep_duration,
        sweep_peak_to_peak_voltage=args.sweep_peak_to_peak_voltage,
    )
    print(f"Estimated sensor frequency: {result.sensor_frequency:.6f} Hz")
    print(f"Estimated sensor amplitude: {result.sensor_amplitude:.6g}")
    print(f"Estimated noise frequency: {result.noise_frequency:.6f} Hz")
    print(f"Estimated noise amplitude: {result.noise_amplitude:.6g}")
    print(f"Sensor-to-noise ratio: {result.sensor_to_noise_ratio:.3f} dB")
    print(f"Estimated decay ratio: {result.decay_ratio:.6f}")

    plot_static_measurement(result, args.output)


def dynamic_vibsim(args: argparse.Namespace) -> None:
    datalogger = build_datalogger(args)
    output = args.output
    if output == Path("static_vibsim.png"):
        output = Path("dynamic_vibsim.png")

    plt.ion()
    plotter = DynamicMeasurementPlotter(output, pause_seconds=args.dynamic_plot_pause)
    last_result = None
    for result in datalogger.run_dynamic_measurements(
        sweep_start=args.sweep_start,
        sweep_end=args.sweep_end,
        sweep_duration=args.sweep_duration,
        sweep_peak_to_peak_voltage=args.sweep_peak_to_peak_voltage,
        excitation_peak_to_peak_voltage=args.dynamic_excitation_peak_to_peak_voltage,
        excitation_cycles=args.dynamic_excitation_cycles,
        cycle_count=args.dynamic_cycles,
        startup_output_duration=args.dynamic_startup_output_duration,
        cycle_period=args.dynamic_cycle_period,
    ):
        last_result = result
        print(
            f"[dynamic_vibsim cycle {result.cycle_index}] "
            f"excitation={result.excitation_frequency:.6f} Hz, "
            f"sensor={result.sensor_frequency:.6f} Hz, "
            f"sensor_amp={result.sensor_amplitude:.6g}, "
            f"noise={result.noise_frequency:.6f} Hz, "
            f"SNR={result.sensor_to_noise_ratio:.3f} dB, "
            f"decay={result.decay_ratio:.6f}"
        )
        plotter.update(result)

    plt.ioff()
    if last_result is not None:
        print(f"Dynamic plot updated: {output}")
        plt.show()


def main() -> None:
    args = parse_args()
    if args.dynamic:
        dynamic_vibsim(args)
    else:
        static_vibsim(args)


if __name__ == "__main__":
    main()
