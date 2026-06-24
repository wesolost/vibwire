import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


@dataclass
class SensorState:
    time: float = 0.0
    displacement: float = 0.0
    velocity: float = 0.0


@dataclass
class ElectromagneticSensorState:
    time: float = 0.0
    displacement: float = 0.0
    velocity: float = 0.0
    current: float = 0.0


@dataclass
class SimulationResult:
    time: np.ndarray
    excitation: np.ndarray
    sensor_signal: np.ndarray
    combined_signal: np.ndarray


class VibratingWireSensorSimulator:
    """Second-order vibrating wire sensor simulator with persistent state."""

    def __init__(
        self,
        q_value: float = 80.0,
        resonant_frequency: float = 1000.0,
        static_resistance: float = 480.0,
        decay_coefficient: float = 5.0,
        output_duration: float = 0.08,
        drive_gain: float = 2.0e6,
        output_gain: float = 1.0,
    ) -> None:
        self.q_value = q_value
        self.resonant_frequency = resonant_frequency
        self.static_resistance = static_resistance
        self.decay_coefficient = decay_coefficient
        self.output_duration = output_duration
        self.drive_gain = drive_gain
        self.output_gain = output_gain
        self.state = SensorState()

    @property
    def angular_frequency(self) -> float:
        return 2.0 * np.pi * self.resonant_frequency

    @property
    def damping(self) -> float:
        q_damping = self.angular_frequency / (2.0 * self.q_value)
        return q_damping + self.decay_coefficient

    def reset(self) -> None:
        self.state = SensorState()

    def _acceleration(self, displacement: float, velocity: float, excitation: float) -> float:
        return (
            self.drive_gain * excitation
            - 2.0 * self.damping * velocity
            - self.angular_frequency**2 * displacement
        )

    def _step(self, excitation: float, dt: float) -> float:
        x0 = self.state.displacement
        v0 = self.state.velocity

        # Fourth-order Runge-Kutta integration for stable oscillator simulation.
        k1_x = v0
        k1_v = self._acceleration(x0, v0, excitation)

        k2_x = v0 + 0.5 * dt * k1_v
        k2_v = self._acceleration(x0 + 0.5 * dt * k1_x, v0 + 0.5 * dt * k1_v, excitation)

        k3_x = v0 + 0.5 * dt * k2_v
        k3_v = self._acceleration(x0 + 0.5 * dt * k2_x, v0 + 0.5 * dt * k2_v, excitation)

        k4_x = v0 + dt * k3_v
        k4_v = self._acceleration(x0 + dt * k3_x, v0 + dt * k3_v, excitation)

        self.state.displacement += dt * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x) / 6.0
        self.state.velocity += dt * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v) / 6.0
        self.state.time += dt

        resistance_scale = 350.0 / max(self.static_resistance, 1.0)
        return self.output_gain * resistance_scale * self.state.displacement

    def simulate(
        self,
        excitation: np.ndarray,
        sample_rate: float,
        output_duration: float | None = None,
        decay_threshold: float = 1.0e-5,
        max_decay_duration: float = 2.0,
    ) -> SimulationResult:
        dt = 1.0 / sample_rate
        excitation = np.asarray(excitation, dtype=float)
        duration = self.output_duration if output_duration is None else output_duration

        excitation_series = excitation.copy()
        if duration > 0.0:
            post_excitation_samples = int(round(duration * sample_rate))
            excitation_series = np.pad(excitation_series, (0, post_excitation_samples))

        start_time = self.state.time
        sensor_values: list[float] = []
        excitation_values: list[float] = []
        time_values: list[float] = []

        for excitation_value in excitation_series:
            time_values.append(self.state.time)
            excitation_values.append(excitation_value)
            sensor_values.append(self._step(excitation_value, dt))

        # duration < 0 means "run only the provided excitation samples".
        if duration == 0.0:
            max_extra_samples = int(round(max_decay_duration * sample_rate))
            for _ in range(max_extra_samples):
                if abs(self.state.displacement) < decay_threshold and abs(self.state.velocity) < decay_threshold:
                    break
                time_values.append(self.state.time)
                excitation_values.append(0.0)
                sensor_values.append(self._step(0.0, dt))

        time = np.asarray(time_values) - start_time
        excitation_out = np.asarray(excitation_values)
        sensor_signal = np.asarray(sensor_values)
        combined_signal = excitation_out + sensor_signal
        return SimulationResult(time, excitation_out, sensor_signal, combined_signal)

    def simulate_sweep(
        self,
        start_frequency: float,
        end_frequency: float,
        sweep_duration: float,
        sample_rate: float,
        amplitude: float = 1.0,
        output_duration: float | None = None,
    ) -> SimulationResult:
        excitation = build_sweep_excitation(
            start_frequency=start_frequency,
            end_frequency=end_frequency,
            duration=sweep_duration,
            sample_rate=sample_rate,
            amplitude=amplitude,
        )
        return self.simulate(
            excitation=excitation,
            sample_rate=sample_rate,
            output_duration=output_duration,
        )

    def current_phase(self) -> float:
        return float(
            np.arctan2(
                self.state.displacement,
                self.state.velocity / max(self.angular_frequency, 1.0),
            )
        )

    def simulate_phase_aligned_single_frequency(
        self,
        frequency: float,
        duration: float,
        sample_rate: float,
        amplitude: float,
        output_duration: float | None = None,
    ) -> SimulationResult:
        excitation = build_single_frequency_excitation(
            frequency=frequency,
            duration=duration,
            sample_rate=sample_rate,
            amplitude=amplitude,
            phase=self.current_phase(),
        )
        return self.simulate(
            excitation=excitation,
            sample_rate=sample_rate,
            output_duration=output_duration,
        )


class IdealVibratingWireSensorSimulator:
    """Ideal decaying sine generator for frequency-estimation debugging."""

    def __init__(
        self,
        q_value: float = 1200.0,
        resonant_frequency: float = 2500.0,
        static_resistance: float = 150.0,
        decay_coefficient: float = 0.2,
        output_duration: float = 0.08,
        output_gain: float = 1.0,
    ) -> None:
        self.q_value = q_value
        self.resonant_frequency = resonant_frequency
        self.static_resistance = static_resistance
        self.decay_coefficient = decay_coefficient
        self.output_duration = output_duration
        self.output_gain = output_gain
        self.state = SensorState()
        self.decay_elapsed = 0.0
        self.decay_amplitude = output_gain

    @property
    def angular_frequency(self) -> float:
        return 2.0 * np.pi * self.resonant_frequency

    @property
    def damping(self) -> float:
        q_damping = self.angular_frequency / (2.0 * self.q_value)
        return q_damping + self.decay_coefficient

    def reset(self) -> None:
        self.state = SensorState()
        self.decay_elapsed = 0.0
        self.decay_amplitude = self.output_gain

    def _ideal_decay(self, sample_count: int, sample_rate: float, start_elapsed: float = 0.0) -> np.ndarray:
        decay_time = start_elapsed + np.arange(sample_count) / sample_rate
        return self.decay_amplitude * np.exp(-self.damping * decay_time) * np.sin(
            self.angular_frequency * decay_time
        )

    def simulate(
        self,
        excitation: np.ndarray,
        sample_rate: float,
        output_duration: float | None = None,
        decay_threshold: float = 1.0e-5,
        max_decay_duration: float = 2.0,
    ) -> SimulationResult:
        dt = 1.0 / sample_rate
        excitation = np.asarray(excitation, dtype=float)
        duration = self.output_duration if output_duration is None else output_duration

        if duration > 0.0:
            post_excitation_samples = int(round(duration * sample_rate))
        elif duration == 0.0:
            if self.damping <= 0.0:
                post_excitation_samples = int(round(max_decay_duration * sample_rate))
            else:
                decay_time = -np.log(decay_threshold / max(self.output_gain, 1.0e-12)) / self.damping
                post_excitation_samples = int(round(min(decay_time, max_decay_duration) * sample_rate))
        else:
            post_excitation_samples = 0

        excitation_out = np.pad(excitation, (0, post_excitation_samples))
        sensor_signal = np.zeros_like(excitation_out)
        has_excitation = bool(np.any(np.abs(excitation) > 0.0))
        if has_excitation and post_excitation_samples > 0:
            self.decay_amplitude = float(np.max(np.abs(excitation)))
            sensor_signal[len(excitation) :] = self._ideal_decay(post_excitation_samples, sample_rate)
            self.decay_elapsed = post_excitation_samples * dt
        elif not has_excitation and len(excitation) > 0:
            sensor_signal[: len(excitation)] = self._ideal_decay(len(excitation), sample_rate, self.decay_elapsed)
            self.decay_elapsed += len(excitation) * dt

        start_time = self.state.time
        time = np.arange(len(excitation_out)) * dt
        self.state.time += len(excitation_out) * dt
        combined_signal = excitation_out + sensor_signal
        return SimulationResult(time, excitation_out, sensor_signal, combined_signal)

    def simulate_sweep(
        self,
        start_frequency: float,
        end_frequency: float,
        sweep_duration: float,
        sample_rate: float,
        amplitude: float = 1.0,
        output_duration: float | None = None,
    ) -> SimulationResult:
        excitation = build_sweep_excitation(
            start_frequency=start_frequency,
            end_frequency=end_frequency,
            duration=sweep_duration,
            sample_rate=sample_rate,
            amplitude=amplitude,
        )
        return self.simulate(
            excitation=excitation,
            sample_rate=sample_rate,
            output_duration=output_duration,
        )

    def current_phase(self) -> float:
        return 0.0

    def simulate_phase_aligned_single_frequency(
        self,
        frequency: float,
        duration: float,
        sample_rate: float,
        amplitude: float,
        output_duration: float | None = None,
    ) -> SimulationResult:
        excitation = build_single_frequency_excitation(
            frequency=frequency,
            duration=duration,
            sample_rate=sample_rate,
            amplitude=amplitude,
            phase=0.0,
        )
        return self.simulate(
            excitation=excitation,
            sample_rate=sample_rate,
            output_duration=output_duration,
        )


class ElectromagneticVibratingWireSensorSimulator:
    """Vibrating wire simulator with coil current, force coupling, and back EMF."""

    def __init__(
        self,
        q_value: float = 1200.0,
        resonant_frequency: float = 2500.0,
        static_resistance: float = 480.0,
        decay_coefficient: float = 0.2,
        output_duration: float = 0.08,
        equivalent_mass: float = 1.0e-4,
        coil_resistance: float = 150.0,
        coil_inductance: float = 0.02,
        force_constant: float = 0.5,
        back_emf_constant: float = 0.05,
        output_gain: float = 5000.0,
    ) -> None:
        self.q_value = q_value
        self.resonant_frequency = resonant_frequency
        self.static_resistance = static_resistance
        self.decay_coefficient = decay_coefficient
        self.output_duration = output_duration
        self.equivalent_mass = equivalent_mass
        self.coil_resistance = coil_resistance
        self.coil_inductance = coil_inductance
        self.force_constant = force_constant
        self.back_emf_constant = back_emf_constant
        self.output_gain = output_gain
        self.state = ElectromagneticSensorState()

    @property
    def angular_frequency(self) -> float:
        return 2.0 * np.pi * self.resonant_frequency

    @property
    def stiffness(self) -> float:
        return self.equivalent_mass * self.angular_frequency**2

    @property
    def damping(self) -> float:
        q_damping = self.equivalent_mass * self.angular_frequency / self.q_value
        return q_damping + self.decay_coefficient * self.equivalent_mass

    def reset(self) -> None:
        self.state = ElectromagneticSensorState()

    def _derivatives(
        self,
        displacement: float,
        velocity: float,
        current: float,
        excitation_voltage: float,
    ) -> tuple[float, float, float]:
        dx_dt = velocity
        dv_dt = (
            self.force_constant * current
            - self.damping * velocity
            - self.stiffness * displacement
        ) / self.equivalent_mass
        di_dt = (
            excitation_voltage
            - self.coil_resistance * current
            - self.back_emf_constant * velocity
        ) / self.coil_inductance
        return dx_dt, dv_dt, di_dt

    def _step(self, excitation_voltage: float, dt: float) -> float:
        x0 = self.state.displacement
        v0 = self.state.velocity
        i0 = self.state.current

        k1_x, k1_v, k1_i = self._derivatives(x0, v0, i0, excitation_voltage)
        k2_x, k2_v, k2_i = self._derivatives(
            x0 + 0.5 * dt * k1_x,
            v0 + 0.5 * dt * k1_v,
            i0 + 0.5 * dt * k1_i,
            excitation_voltage,
        )
        k3_x, k3_v, k3_i = self._derivatives(
            x0 + 0.5 * dt * k2_x,
            v0 + 0.5 * dt * k2_v,
            i0 + 0.5 * dt * k2_i,
            excitation_voltage,
        )
        k4_x, k4_v, k4_i = self._derivatives(
            x0 + dt * k3_x,
            v0 + dt * k3_v,
            i0 + dt * k3_i,
            excitation_voltage,
        )

        self.state.displacement += dt * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x) / 6.0
        self.state.velocity += dt * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v) / 6.0
        self.state.current += dt * (k1_i + 2.0 * k2_i + 2.0 * k3_i + k4_i) / 6.0
        self.state.time += dt

        resistance_scale = 350.0 / max(self.static_resistance, 1.0)
        induced_voltage = self.back_emf_constant * self.state.velocity
        return self.output_gain * resistance_scale * induced_voltage

    def simulate(
        self,
        excitation: np.ndarray,
        sample_rate: float,
        output_duration: float | None = None,
        decay_threshold: float = 1.0e-5,
        max_decay_duration: float = 2.0,
    ) -> SimulationResult:
        dt = 1.0 / sample_rate
        excitation = np.asarray(excitation, dtype=float)
        duration = self.output_duration if output_duration is None else output_duration

        excitation_series = excitation.copy()
        if duration > 0.0:
            post_excitation_samples = int(round(duration * sample_rate))
            excitation_series = np.pad(excitation_series, (0, post_excitation_samples))

        start_time = self.state.time
        sensor_values: list[float] = []
        excitation_values: list[float] = []
        time_values: list[float] = []

        for excitation_value in excitation_series:
            time_values.append(self.state.time)
            excitation_values.append(excitation_value)
            sensor_values.append(self._step(excitation_value, dt))

        if duration == 0.0:
            max_extra_samples = int(round(max_decay_duration * sample_rate))
            for _ in range(max_extra_samples):
                if abs(self.state.displacement) < decay_threshold and abs(self.state.velocity) < decay_threshold:
                    break
                time_values.append(self.state.time)
                excitation_values.append(0.0)
                sensor_values.append(self._step(0.0, dt))

        time = np.asarray(time_values) - start_time
        excitation_out = np.asarray(excitation_values)
        sensor_signal = np.asarray(sensor_values)
        combined_signal = excitation_out + sensor_signal
        return SimulationResult(time, excitation_out, sensor_signal, combined_signal)

    def simulate_sweep(
        self,
        start_frequency: float,
        end_frequency: float,
        sweep_duration: float,
        sample_rate: float,
        amplitude: float = 1.0,
        output_duration: float | None = None,
    ) -> SimulationResult:
        excitation = build_sweep_excitation(
            start_frequency=start_frequency,
            end_frequency=end_frequency,
            duration=sweep_duration,
            sample_rate=sample_rate,
            amplitude=amplitude,
        )
        return self.simulate(
            excitation=excitation,
            sample_rate=sample_rate,
            output_duration=output_duration,
        )

    def current_phase(self) -> float:
        velocity_scale = max(
            np.hypot(self.angular_frequency * self.state.displacement, self.state.velocity),
            1.0e-12,
        )
        return float(np.arcsin(np.clip(self.state.velocity / velocity_scale, -1.0, 1.0)))

    def simulate_phase_aligned_single_frequency(
        self,
        frequency: float,
        duration: float,
        sample_rate: float,
        amplitude: float,
        output_duration: float | None = None,
    ) -> SimulationResult:
        excitation = build_single_frequency_excitation(
            frequency=frequency,
            duration=duration,
            sample_rate=sample_rate,
            amplitude=amplitude,
            phase=self.current_phase(),
        )
        return self.simulate(
            excitation=excitation,
            sample_rate=sample_rate,
            output_duration=output_duration,
        )


def build_sweep_excitation(
    start_frequency: float,
    end_frequency: float,
    duration: float,
    sample_rate: float,
    amplitude: float,
) -> np.ndarray:
    time = np.arange(0.0, duration, 1.0 / sample_rate)
    instantaneous_frequency = np.linspace(start_frequency, end_frequency, len(time))
    phase = 2.0 * np.pi * np.cumsum(instantaneous_frequency) / sample_rate
    return amplitude * np.sin(phase)


def build_single_frequency_excitation(
    frequency: float,
    duration: float,
    sample_rate: float,
    amplitude: float,
    phase: float,
) -> np.ndarray:
    time = np.arange(0.0, duration, 1.0 / sample_rate)
    return amplitude * np.sin(2.0 * np.pi * frequency * time + phase)


def plot_result(result: SimulationResult, output: Path, title: str) -> plt.Figure:
    fig, axis = plt.subplots(1, 1, figsize=(12, 5))

    axis.plot(result.time, result.excitation, color="red", linewidth=1.1, label="Excitation")
    axis.plot(result.time, result.sensor_signal, color="C0", linewidth=1.3, label="Sensor signal")
    axis.set_title(title)
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Amplitude")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(output, dpi=220)
    return fig


def plot_two_results(
    first_result: SimulationResult,
    second_result: SimulationResult,
    output: Path,
) -> None:
    first_dt = first_result.time[1] - first_result.time[0] if len(first_result.time) > 1 else 0.0
    second_time = second_result.time + first_result.time[-1] + first_dt
    time = np.concatenate((first_result.time, second_time))
    excitation = np.concatenate((first_result.excitation, second_result.excitation))
    sensor_signal = np.concatenate((first_result.sensor_signal, second_result.sensor_signal))

    fig, axis = plt.subplots(1, 1, figsize=(12, 5))

    axis.plot(time, excitation, color="red", linewidth=1.1, label="Excitation")
    axis.plot(time, sensor_signal, color="C0", linewidth=1.3, label="Sensor signal")
    axis.axvline(
        first_result.time[-1],
        color="#777777",
        linestyle="--",
        linewidth=1.0,
        label="Second excitation starts",
    )
    axis.set_title("Continuous vibrating wire simulation with two excitation stages")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Amplitude")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(output, dpi=220)
    plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vibrating wire sensor simulator.")

    # Vibrating wire mechanical model parameters.
    parser.add_argument("--q-value", type=float, default=1200.0)
    parser.add_argument("--resonant-frequency", type=float, default=2500.0)
    parser.add_argument("--static-resistance", type=float, default=480.0)
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
    parser.add_argument("--back-emf-constant", type=float, default=0.05)
    parser.add_argument("--electromagnetic-output-gain", type=float, default=10)

    # Sampling and sweep excitation parameters.
    parser.add_argument("--sample-rate", type=float, default=6_500_000.0)
    parser.add_argument("--sweep-start", type=float, default=1400.0)
    parser.add_argument("--sweep-end", type=float, default=3500.0)
    parser.add_argument("--sweep-duration", type=float, default=0.02)
    parser.add_argument(
        "--sweep-peak-to-peak-voltage",
        type=float,
        default=20.0,
        help="Sweep excitation peak-to-peak voltage. Default is 20 Vpp.",
    )

    # Post-excitation output and plot file parameters.
    parser.add_argument(
        "--output-duration",
        type=float,
        default=0.02,
        help="Output duration after excitation ends. Use 0 to let the signal naturally decay.",
    )
    parser.add_argument("--output", type=Path, default=Path("vibrating_wire_simulation.png"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.sensor_model == "ideal":
        simulator = IdealVibratingWireSensorSimulator(
            q_value=args.q_value,
            resonant_frequency=args.resonant_frequency,
            static_resistance=args.static_resistance,
            decay_coefficient=args.decay_coefficient,
            output_duration=args.output_duration,
            output_gain=args.electromagnetic_output_gain,
        )
    elif args.sensor_model == "electromagnetic":
        simulator = ElectromagneticVibratingWireSensorSimulator(
            q_value=args.q_value,
            resonant_frequency=args.resonant_frequency,
            static_resistance=args.static_resistance,
            decay_coefficient=args.decay_coefficient,
            output_duration=args.output_duration,
            equivalent_mass=args.equivalent_mass,
            coil_resistance=args.coil_resistance,
            coil_inductance=args.coil_inductance,
            force_constant=args.force_constant,
            back_emf_constant=args.back_emf_constant,
            output_gain=args.electromagnetic_output_gain,
        )
    else:
        simulator = VibratingWireSensorSimulator(
            q_value=args.q_value,
            resonant_frequency=args.resonant_frequency,
            static_resistance=args.static_resistance,
            decay_coefficient=args.decay_coefficient,
            output_duration=args.output_duration,
        )

    first_result = simulator.simulate_sweep(
        start_frequency=args.sweep_start,
        end_frequency=args.sweep_end,
        sweep_duration=args.sweep_duration,
        sample_rate=args.sample_rate,
        amplitude=args.sweep_peak_to_peak_voltage / 2.0,
    )
    second_result = simulator.simulate_phase_aligned_single_frequency(
        frequency=args.resonant_frequency,
        duration=1.0 / args.resonant_frequency,
        sample_rate=args.sample_rate,
        amplitude=args.sweep_peak_to_peak_voltage / 2.0,
    )
    plot_two_results(first_result, second_result, args.output)


if __name__ == "__main__":
    main()
