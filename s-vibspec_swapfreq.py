######################################################################################
# Description: DDS Algorithm Simulation

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

class HyperParamAndSignalTable:
    def __init__(
        self,
        freq_sample = 44100,
        n_sample = 441000,
    ):
        assert n_sample % freq_sample == 0, "sampe number must be integral multiple of freq_sample"
        self.fs = freq_sample
        self.n_sample = n_sample
        self.duration = n_sample / freq_sample
    
    def FS(self) -> int:
        return self.fs
    def N(self) -> int:
        return self.n_sample
    def DT(self) -> int:
        return self.duration
    
    def generate_signal_table(
        self,
    ):
        tp =  np.linspace(0, self.duration, self.n_sample, endpoint=False)
        signal = np.sin(2 * np.pi * tp)
        return np.array(signal)
    
def genernate_swap_freq(
    fs : int,
    ns : int,
    signal_table : np.array,
    swap_freq_points : np.array,
    s_duation : int = 1000, # ms
    swap_step_duation : int = 1, # ms
):
    sigle_step_point = int(fs / (s_duation / swap_step_duation))
    tsp = np.zeros(sigle_step_point * len(swap_freq_points))
    
    current = 0
    idx = 0
    for freq in swap_freq_points:
        p_accum = int(ns / (fs / freq))
        phase_seq = np.zeros(sigle_step_point, dtype=int)
        for i in range(sigle_step_point):
            phase_seq[i] = current
            current = (current + p_accum) % ns
        idx += 1
        tsp[(idx -1) * sigle_step_point : idx * sigle_step_point] = signal_table[phase_seq]

    return tsp
# ============================================================================
# algo sim show
# ============================================================================
if __name__ == "__main__":
    
    FS = 1000 * 100
    NS = 1000 * 100
    
    swap_freq_begin = 1000
    swap_freq_end = 4000
    swap_steps = 50
    swap_step_duation = 1  # 1 ms for every swap freq point
    
    hps = HyperParamAndSignalTable(freq_sample=FS, n_sample=NS)
    signal_table = hps.generate_signal_table()
    
    # 在对数空间中等间距分割
    log_start = np.log10(swap_freq_begin)
    log_end = np.log10(swap_freq_end)
    log_values = np.linspace(log_start, log_end, swap_steps)
    swap_freq_points = np.round(10**log_values)
    
    
    print(f"{swap_freq_points}")
    swap_signal = genernate_swap_freq(hps.FS(), hps.N(), signal_table, swap_freq_points, hps.DT() * 1000, swap_step_duation)
    
    plt.plot(swap_signal, 'b-', linewidth=2)
    plt.tight_layout()
    plt.show()