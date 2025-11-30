######################################################################################
# Description: Vibrating Wire Measurement Algorithm Simulation

# Author: Yuechao Guo
# Copyright (c) 2024 Yuechao Guo. All Rights Reserved.
# This code is protected by copyright law and intellectual property rights.
# No part of this code may be reproduced, distributed, modified, or used in any form
# without the explicit written permission of the author, Yuechao Guo.

# Contact: [wesolost@163.com]
# Created: [2025-11-18]
# Last Modified: [2025-11-128]
# Version: [v1]
#####################################################################################
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft
from scipy.signal import find_peaks
from scipy.optimize import curve_fit
from enum import Enum, auto
import warnings
warnings.filterwarnings('ignore')

### defination
class Window(Enum):
    RECT = auto()
    HANN = auto()
    HAMM = auto()
    BLACKMAN = auto()
    
class HyperParamAndSignal:
    def __init__(
        self,
        freq_sample = 44100,
        n_sample = 4096,
        decay_rate = 2.0,
        white_noise_level = 0.1,
    ):
        self.fs = freq_sample
        self.n_sample = n_sample
        self.decay_rate = decay_rate
        self.white_noise_level = white_noise_level
        self.duration = float(self.n_sample / self.fs)
        
    def FS(self) -> int:
        return self.fs
    def N(self) -> int:
        return self.n_sample
    def DR(self) -> float:
        return self.decay_rate
    def WNL(self) -> float:
        return self.white_noise_level
    def DT(self) -> float:
        return self.duration
        
    def generate_signal(
        self, 
        A0 : float = 1.0, 
        freq : float = 10.0, 
        phase : float = 0,
        nA0 : float = 0.1, 
        nfreq : float = 0, 
        nphase : float = 0,
    ):
        tp =  np.linspace(0, self.duration, self.n_sample, endpoint=False)
        signal = A0 * np.exp(-self.decay_rate * tp) * np.sin(2 * np.pi * freq * tp + phase)
        if nfreq != 0:
            signal = signal + nA0 * np.sin(2 * np.pi * nfreq * tp + phase)
        if self.white_noise_level > 0.0:
            signal = signal + self.white_noise_level * np.random.randn(self.n_sample)
        return signal
    
    def adc_quantize(
        self,
        signal_analog : np.ndarray,
        v_ref : float = 5.0, 
        num_bits : int = 12, 
        quantization_type : str ='mid-tread',
    ):
        max_abs_val = np.max(np.abs(signal_analog))
        signal = signal_analog / max_abs_val
        
        q = v_ref / (2 ** num_bits)
        num_levels = 2 ** num_bits
        
        if quantization_type == 'mid-tread':
            digital_min = -num_levels // 2
            digital_max = num_levels // 2 - 1
        else:
            digital_min = -num_levels // 2 + 1
            digital_max = num_levels // 2
        
        signal_clipped = np.clip(signal, -v_ref/2, v_ref/2)
        signal_digital = np.round(signal_clipped / q)
        signal_digital = np.clip(signal_digital, digital_min, digital_max)
        signal_quantized_voltage = signal_digital * q
        # quantization_error = signal_clipped - signal_quantized_voltage
        return signal_quantized_voltage * max_abs_val, signal_digital
    
    def apply_window(
        self, 
        signal : np.ndarray,
        win_type : Window = Window.RECT,
    ):
        if win_type == Window.RECT:
            return signal, np.ones(self.n_sample)
        elif win_type == Window.HANN:
            hanning_window = np.hanning(self.n_sample)
            return signal * hanning_window, hanning_window
        elif win_type == Window.HAMM:
            hamming_window = np.hamming(self.n_sample)
            return signal * hamming_window, hamming_window
        elif win_type == Window.BLACKMAN:
            blackman_window = np.blackman(self.n_sample)
            return signal * blackman_window, blackman_window
        else:
            raise NotImplementedError("Not support window type")

def find_local_maxima_manual(
    n_sampe : int,
    fft_vec : np.ndarray,
    win_type : Window = Window.RECT,
    th_val : float = 0.0,
):
    maxima = []
    peaks = []
    for i in range(1, n_sampe // 2 - 1):
        if fft_vec[i] > fft_vec[i-1] and fft_vec[i] >= fft_vec[i+1] and fft_vec[i] > th_val:
            peaks.append(i)
            maxima.append(fft_vec[i])
    
    sorted_indices = np.argsort(peaks)
    sorted_peaks = np.array(peaks)[sorted_indices]
    sorted_values = np.array(maxima)[sorted_indices]
    
    merged_peaks = []
    merged_values = []
    current_cluster = [sorted_peaks[0]]
    current_values = [sorted_values[0]]
    max_distance = 8
    
    for i in range(1, len(sorted_peaks)):
        if sorted_peaks[i] - current_cluster[-1] <= max_distance:
            current_cluster.append(sorted_peaks[i])
            current_values.append(sorted_values[i])
        else:
            max_idx = np.argmax(current_values)
            merged_peaks.append(current_cluster[max_idx])
            merged_values.append(current_values[max_idx])
            
            current_cluster = [sorted_peaks[i]]
            current_values = [sorted_values[i]]
    
    if current_cluster:
        max_idx = np.argmax(current_values)
        merged_peaks.append(current_cluster[max_idx])
        merged_values.append(current_values[max_idx])
    
    # print(f"{merged_peaks}, {merged_values}")
    if len(merged_peaks) > 2:
        top_two_indices = np.argsort(merged_values)[-2:][::-1]
        top_two_merged_peaks = [merged_peaks[i] for i in top_two_indices]
        return top_two_merged_peaks
    return merged_peaks

def candan_delta_est(
    n_sampe : int,
    pos : int,
    fft_real : np.ndarray,
    fft_imag : np.ndarray,
):
    up_factor = fft_real[pos - 1] - fft_real[pos + 1] + (fft_imag[pos - 1] - fft_imag[pos + 1]) * 1j
    down_factor = 2 * fft_real[pos] - fft_real[pos - 1] - fft_real[pos + 1] \
                + (2 * fft_imag[pos] - fft_imag[pos - 1] - fft_imag[pos + 1]) * 1j
    delta = np.tan(np.pi / n_sampe) / (np.pi / n_sampe) * np.real(up_factor / down_factor)
    return delta

def freq_estimate(
    fs :int,
    n_sampe : int,
    pos : int,
    fft_real : np.ndarray,
    fft_imag : np.ndarray,
    osignal : np.ndarray,
    win_type : Window = Window.RECT,
):  
    pos = max(1, pos)
    pos = min(n_sampe / 2 - 1, pos)
    
    # direct freq findding
    direct_detect_freq = pos * (float(fs) / n_sampe)
    
    # candan freq detection
    delta = 0.0
    amp_val = 0.0
    if win_type == Window.RECT:
        delta = candan_delta_est(n_sampe, pos, fft_real, fft_imag)
        amp_val = np.sqrt(fft_real[pos]**2 + fft_imag[pos]**2) * (2.0 / n_sampe) / np.abs(np.sinc(delta))
        
        # print(f"pos={pos}, delta={delta}, amp_val={amp_val}, dr={np.abs(np.sinc(delta))}")
        candan_detect_freq = (pos + delta) * (float(fs) / n_sampe)
        return direct_detect_freq, candan_detect_freq, amp_val
    
    # phase diff method
    shifft_m = n_sampe // 4 # less than n_sampe // 2
    window_val = np.ones(n_sampe - shifft_m)
    if win_type == Window.HANN:
        window_val = np.hanning(n_sampe - shifft_m)
    elif win_type == Window.HAMM:
        window_val = np.hamming(n_sampe - shifft_m)
    elif win_type == Window.BLACKMAN:
        window_val = np.blackman(n_sampe - shifft_m)
    else:
        raise NotImplementedError("Candan est not support window type")
    
    fft_left_n = fft(osignal[: n_sampe - shifft_m] * window_val, n_sampe)
    fft_right_n = fft(osignal[shifft_m : n_sampe] * window_val, n_sampe)
    ph1, ph2 = np.angle(fft_left_n[pos]), np.angle(fft_right_n[pos])
    dph = ph2 - ph1
    dph_unk = dph - ((2 * np.pi * pos * shifft_m / n_sampe) % (2 * np.pi))
    delta = dph_unk * n_sampe / (2 * np.pi * shifft_m)
    
    amp_factor = 1.0
    if win_type == Window.HANN:
        amp_factor = 0.5 * np.sinc(delta) + 0.25 * np.sinc(delta - 1) + 0.25 * np.sinc(delta + 1)
    elif win_type == Window.HAMM:
        amp_factor = 0.54 * np.sinc(delta) + 0.23 * np.sinc(delta - 1) + 0.23 * np.sinc(delta + 1)
    elif win_type == Window.BLACKMAN:
        amp_factor = 0.42 * np.sinc(delta) - 0.25 * np.sinc(delta - 1) - 0.25 * np.sinc(delta - 1) \
            + 0.04 * np.sinc(delta - 2) + 0.04 * np.sinc(delta + 2)
    else:
        raise NotImplementedError("Candan est not support window type")

    amp_val = np.sqrt(fft_real[pos]**2 + fft_imag[pos]**2) * (2.0 / n_sampe) / np.abs(amp_factor)
    print(f"pos={pos}, delta={delta}, amp_val={amp_val}")
    candan_detect_freq = (pos + delta) * (float(fs) / n_sampe)
    
    return direct_detect_freq, candan_detect_freq, amp_val
    
def sensor_noise_freq_metric(
    detect_result : dict,
):
    if len(detect_result) > 2:
        raise "Only support with sensor and noise analysis"
    
    if len(detect_result) == 1:
        sensor_freq = 0.0
        sensor_amplitude = 0.0
        noise_freq = 0.0
        noise_amplitude = 0.0
        sensor_to_noise_ratio = 0.0  
        for p in detect_result:
            sensor_freq = p["candan_detect_freq"]
            sensor_amplitude = p["val"]
        return sensor_freq, sensor_amplitude, noise_freq, noise_amplitude, sensor_to_noise_ratio

    last_val = 0.0
    senor_pos = 0
    for p in detect_result:
        if p["val"] > last_val:
            last_val = p["val"]
            senor_pos = p["pos"]
     
    sensor_freq = 0.0
    sensor_amplitude = 0.0
    noise_freq = 0.0
    noise_amplitude = 0.0
    sensor_to_noise_ratio = 0.0       
    for p in detect_result:
        if senor_pos == p["pos"]:
            sensor_freq = p["candan_detect_freq"]
            sensor_amplitude = p["val"]
        else:
            noise_freq = p["candan_detect_freq"]
            noise_amplitude = p["val"]
    
    sensor_to_noise_ratio = 20 * np.log10(sensor_amplitude / noise_amplitude)
    
    return sensor_freq, sensor_amplitude, noise_freq, noise_amplitude, sensor_to_noise_ratio

def freq_hilbert_filter(
    n_sampe : int,
    fft_real : np.ndarray,
    fft_imag : np.ndarray,
):
    hilbert_arr = np.zeros(n_sampe, dtype=float)
    if n_sampe % 2 == 0:
        hilbert_arr[0] = 1.0
        hilbert_arr[1 : n_sampe // 2] = 2.0
        hilbert_arr[n_sampe // 2] = 1.0
    else:
        hilbert_arr[0] = 1.0
        hilbert_arr[1 : n_sampe // 2] = 2.0
    
    return fft_real * hilbert_arr, fft_imag * hilbert_arr

def calc_decay_ratio(
    n_sampe : int,
    y : np.ndarray,
    x : np.ndarray,
):
    f1 = n_sampe * np.sum(x * y) - np.sum(x) * np.sum(y)
    f2 = n_sampe * np.sum(x**2) - np.sum(x)**2
    return  f1 / f2

# ============================================================================
# algo sim show
# ============================================================================
if __name__ == "__main__":

    FS = 4096 * 8
    NS = 4096
    # orign_freq = [580, 850, 1016.660156, 2000.1275, 3468.976, 4765.1679, 5832.4198]
    orign_freq = [1036.002]
    orign_freq_amp = 4.5
    signal_decay_rate = 3
    noise_freq = 50
    noise_freq_amp = 0.1
    wnl = 0.000001
    win_type = Window.HANN
    V_REF = 5
    ADC_BITS = 12
    TRUC = 1
    
    # picture four
    fig, axes = plt.subplots(2, 3, figsize=(10, 8))
    hps = HyperParamAndSignal(freq_sample=FS, n_sample=NS, decay_rate=signal_decay_rate, white_noise_level=wnl)
    for fq in orign_freq:
        # generate signal and window
        signal_analog = hps.generate_signal(A0=orign_freq_amp, freq=fq, nA0=noise_freq_amp, nfreq=noise_freq)
        osignal_quant, s_d = hps.adc_quantize(signal_analog, V_REF, ADC_BITS)
        signal, window_val = hps.apply_window(osignal_quant, win_type)

        # fft 
        fft_res = fft(signal, NS)
        fft_real = fft_res.real
        fft_imag = fft_res.imag
        fft_magnitude = np.sqrt(fft_real**2 + fft_imag**2) * (2.0 / hps.N())

        # hilbert change for envelope
        hilbert_real, hilbert_imag = freq_hilbert_filter(hps.N(), fft_real, fft_imag)
        hilbert_complex_signal = hilbert_real + 1j * hilbert_imag
        hilbert_res = ifft(hilbert_complex_signal)
        signal_envelope = np.abs(hilbert_res / np.abs(window_val))
        
        # calc for decay_ratio
        ln_result = np.log(signal_envelope)
        tp = np.linspace(0, hps.DT(), hps.N(), endpoint=False)
        decay_ratio = calc_decay_ratio(hps.N() - 2 * TRUC, ln_result[TRUC:-TRUC], tp[TRUC:-TRUC]) * (-1.0)
        print(f"decay_ratio={decay_ratio:4.6f}")
        
        # find local max and candan detection for every peak
        vec_pos = find_local_maxima_manual(hps.N(), fft_magnitude, th_val=0.001)
        detect_result = []
        for pos in vec_pos:
            if pos == 0 or pos == hps.N() / 2:
                continue
            direct_detect_freq, candan_detect_freq, amp_val = freq_estimate(hps.FS(), hps.N(), pos, fft_real, fft_imag, osignal_quant, win_type)
            one_peak = {"pos": pos, "val":amp_val, \
                        "direct_detect_freq":direct_detect_freq, "candan_detect_freq":candan_detect_freq}
            detect_result.append(one_peak)
        
        # get metric
        sensor_freq, sensor_amplitude, noise_freq, noise_amplitude, sensor_to_noise_ratio = sensor_noise_freq_metric(detect_result)
        
        # correct the amp
        drsample = np.abs(np.exp(-decay_ratio * tp))
        sensor_amplitude = np.sum(sensor_amplitude / drsample) / hps.N()
        noise_amplitude = np.sum(noise_amplitude / drsample) / hps.N()
        
        print(f"sensor_freq={sensor_freq:4.6f}Hz")
        print(f"sensor_amplitude={sensor_amplitude:4.6f}")
        print(f"noise_freq={noise_freq:4.6f}Hz")
        print(f"noise_amplitude={noise_amplitude:4.6f}")
        print(f"sensor_to_noise_ratio={sensor_to_noise_ratio:4.6f}dB")
    
    # sys.exit(1)
    axes[0, 0].plot(signal, 'b-', linewidth=2)
    axes[0, 0].set_title('Signal')
    axes[0, 0].set_ylabel('AMP')
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(window_val, 'r-', linewidth=2)
    axes[0, 1].set_title('Window')
    axes[0, 1].set_ylabel('AMP')
    axes[0, 1].grid(True)
    
    axes[0, 2].plot(s_d, 'b-', linewidth=2)
    axes[0, 2].set_title('Signal-ADC-Quantize')
    axes[0, 2].set_ylabel('AMP')
    axes[0, 2].grid(True)

    axes[1, 0].semilogy(fft_magnitude, linewidth=2)
    # axes[1, 0].plot(fft_magnitude, linewidth=2)
    axes[1, 0].set_title('Spectral')
    axes[1, 0].set_ylabel('AMP')
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(signal_envelope, linewidth=2)
    axes[1, 1].set_title('Envelope')
    axes[1, 1].set_ylabel('AMP')
    axes[1, 1].grid(True)
    plt.tight_layout()
    plt.show()