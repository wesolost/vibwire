######################################################################################
# Description: Serial Server with Modbus Protocol Simulation
# Author: AI Assistant
# Created: 2026-01-12
# Version: v1
######################################################################################
import serial
import serial.tools.list_ports
from pymodbus.server.startstop import StartSerialServer
from pymodbus import ModbusDeviceIdentification, FramerType
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusDeviceContext, ModbusServerContext
import logging
import struct
import time
import random
from s_vibspec_sim import serial_get_result
from s_vibspec_sim import VibwireResult
from collections import deque

# Configure logging
logging.basicConfig()
log = logging.getLogger()
# Create a queue to store VibwireResult objects with max size 10
vibwire_queue = deque(maxlen=10)
log.setLevel(logging.INFO)

class SerialModbusServer:
    def __init__(self):
        self.port = 'COM11'
        self.baudrate = 961200
        self.bytesize = serial.EIGHTBITS
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.timeout = 1
        self.slave_id = 0x01
        
    def configure_serial(self, port='COM11', baudrate=9600, bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1):
        """配置串口参数"""
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
    
    def list_available_ports(self):
        """列出可用的串口"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def init_modbus_context(self):
        """初始化Modbus数据上下文"""
        # 创建数据块
        # 离散输入 (只读)
        di = ModbusSequentialDataBlock(0, [0]*100)
        # 线圈 (读写)
        co = ModbusSequentialDataBlock(0, [0]*100)
        # 输入寄存器 (只读)
        ir = ModbusSequentialDataBlock(0, [0]*100)
        # 保持寄存器 (读写)
        hr = ModbusSequentialDataBlock(0, [0]*100)
        
        # 初始化一些示例数据
        hr.setValues(0, [1234])  # 保持寄存器地址0的值为1234
        hr.setValues(1, [5678])  # 保持寄存器地址1的值为5678
        ir.setValues(0, [9876])  # 输入寄存器地址0的值为9876
        
        # 创建从机上下文
        slave_context = ModbusDeviceContext(
            di=di, co=co, ir=ir, hr=hr
        )
        
        # 创建服务器上下文
        context = ModbusServerContext(devices=slave_context, single=True)
        
        return context
    
    def init_device_identity(self):
        """初始化设备标识"""
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'VibWire'
        identity.ProductCode = 'VW'
        identity.VendorUrl = 'http://www.example.com'
        identity.ProductName = 'Modbus Simulation Server'
        identity.ModelName = 'Serial Server'
        identity.MajorMinorRevision = '1.0'
        
        return identity
    
    def trace_received_packets(self, is_master: bool, data: bytes) -> bytes:
        """跟踪并打印接收的数据包"""
        if not is_master:  # 只打印从串口接收的数据（不是主设备发送的）
            print(f"[Serial RX] {data.hex(' ')}")
        return data
    
    def crc16(self, data):
        """计算CRC16校验码"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    
    def add_crc(self, data):
        """为数据添加CRC16校验码"""
        crc = self.crc16(data)
        return data + struct.pack('<H', crc)  # CRC以小端序添加
    
    def verify_crc(self, data):
        """验证数据的CRC16校验码"""
        if len(data) < 2:
            return False
        received_crc = struct.unpack('<H', data[-2:])[0]
        calculated_crc = self.crc16(data[:-2])
        return received_crc == calculated_crc
    
    def handle_0x65_command(self, request_frame):
        """处理0x65指令：通道结果查询与响应"""
        try:
            # 解析请求帧：[地址][0x65][通道编号][CRC*2]
            if len(request_frame) < 5:  # 地址(1) + 功能码(1) + 通道编号(1) + CRC(2)
                # 异常响应：[地址][0xE5][通道编号][CRC*2]
                channel = request_frame[2] if len(request_frame) > 2 else 0x00
                error_response = struct.pack('>BBB', self.slave_id, 0xE5, channel)
                return self.add_crc(error_response)
            
            channel = request_frame[2]
            
            # 验证通道编号（1-8）
            if not (1 <= channel <= 8):
                error_response = struct.pack('>BBB', self.slave_id, 0xE5, channel)
                return self.add_crc(error_response)
            
            print(f"Executing one-time sampling on channel {channel}...")
            # 生成200到6500之间的随机浮点数作为orign_freq
            orign_freq = random.uniform(200, 6500)
            print(f"Generated random origin frequency: {orign_freq:.4f} Hz")
            vibwire_result = serial_get_result(orign_freq=orign_freq)
            vibwire_queue.append(vibwire_result)
            
            # 从vibwire_queue获取数据，如果队列不为空
            if len(vibwire_queue) > 0:
                # 获取队列中第一个数据
                vibwire_res = vibwire_queue[0]
                
                # 从VibwireResult对象中提取参数
                signal_frequency = vibwire_res.sensor_freq
                signal_amplitude = vibwire_res.sensor_amplitude
                noise_frequency = vibwire_res.est_noise_freq
                noise_amplitude = vibwire_res.est_noise_amplitude
                signal_to_noise_ratio = vibwire_res.sensor_to_noise_ratio
                decay_ratio = vibwire_res.decay_ratio
                
                # 使用其他可用属性或模拟值
                temperature = 20.0 + channel * 1.5  # 保留模拟温度
                voltage_excitation_amplitude = 5.0  # 使用原始频率幅度转换
                system_sample_rate = vibwire_res.FS  # 使用实际采样频率
                one_process_time = 0.5  # 根据采样点数和采样频率计算处理时间
                
                print(f"Using data from queue for channel {channel}:")
                print(f"  Signal Frequency: {signal_frequency:.2f} Hz")
                print(f"  Signal Amplitude: {signal_amplitude:.2f}")
                print(f"  Noise Frequency: {noise_frequency:.2f} Hz")
                print(f"  Sampling Frequency: {system_sample_rate} Hz")
            else:
                # 队列空时返回N bytes为0的响应
                print(f"Queue empty, returning empty response for channel {channel}")
                
                # 构建空响应帧：[地址][0x65][通道编号][N bytes=0][data=空][CRC*2]
                n_bytes = 0
                data = b""
                response = struct.pack('>BBBB', self.slave_id, 0x65, channel, n_bytes)
                response += data
                return self.add_crc(response)
            
            # 构建数据部分（仅当队列有数据时执行）
            data = struct.pack('>10f', 
                              signal_frequency,
                              signal_amplitude,
                              noise_frequency,
                              noise_amplitude,
                              signal_to_noise_ratio,
                              decay_ratio,
                              temperature,
                              voltage_excitation_amplitude,
                              system_sample_rate,
                              one_process_time)
            
            # 构建响应帧：[地址][0x65][通道编号][N bytes][data][CRC*2]
            n_bytes = len(data)  # 40 bytes
            response = struct.pack('>BBBB', self.slave_id, 0x65, channel, n_bytes)
            response += data
            
            return self.add_crc(response)
            
        except Exception as e:
            log.error(f"Error handling 0x65 command: {e}")
            # 异常响应
            channel = request_frame[2] if len(request_frame) > 2 else 0x00
            error_response = struct.pack('>BBB', self.slave_id, 0xE5, channel)
            return self.add_crc(error_response)
    
    def handle_0x66_command(self, request_frame):
        """处理0x66指令：通道启动Program"""
        try:
            # 解析请求帧：[地址][0x66][通道编号][N bytes][Parameters][CRC*2]
            if len(request_frame) < 7:  # 地址(1) + 功能码(1) + 通道编号(1) + N bytes(1) + CRC(2)
                # 异常响应：[地址][0xE6][通道编号][CRC*2]
                channel = request_frame[2] if len(request_frame) > 2 else 0x00
                error_response = struct.pack('>BBB', self.slave_id, 0xE6, channel)
                return self.add_crc(error_response)
            
            channel = request_frame[2]
            n_bytes = request_frame[3]
            
            # 验证通道编号（1-8）
            if not (1 <= channel <= 8):
                error_response = struct.pack('>BBB', self.slave_id, 0xE6, channel)
                return self.add_crc(error_response)
            
            # 验证参数长度
            if n_bytes != 17:
                error_response = struct.pack('>BBB', self.slave_id, 0xE6, channel)
                return self.add_crc(error_response)
            
            # 验证整个帧的长度
            expected_length = 1 + 1 + 1 + 1 + n_bytes + 2  # 地址 + 功能码 + 通道 + N bytes + Parameters + CRC
            if len(request_frame) != expected_length:
                error_response = struct.pack('>BBB', self.slave_id, 0xE6, channel)
                return self.add_crc(error_response)
            
            # 解析Parameters字段
            parameters = request_frame[4:4+n_bytes]
            
            # 解析Thermistor-Setting|Swap-Type
            thermistor_swap_byte = parameters[0]
            thermistor_setting = (thermistor_swap_byte & 0x02) >> 1  # 第二bit
            swap_type = thermistor_swap_byte & 0x01  # 第一bit
            
            # 解析Thermistor-Coeff-A/B/C
            thermistor_coeff_a = struct.unpack('>f', parameters[1:5])[0]
            thermistor_coeff_b = struct.unpack('>f', parameters[5:9])[0]
            thermistor_coeff_c = struct.unpack('>f', parameters[9:13])[0]
            
            # 解析Swap-Start/End-Frequency
            swap_start_frequency = struct.unpack('>H', parameters[13:15])[0]
            swap_end_frequency = struct.unpack('>H', parameters[15:17])[0]
            
            # 打印解析后的参数
            print(f"Channel {channel} Program Parameters:")
            print(f"  Thermistor Setting: {'ON' if thermistor_setting else 'OFF'}")
            print(f"  Swap Type: {'Linear' if swap_type else 'Logarithmic'}")
            print(f"  Thermistor Coeff A: {thermistor_coeff_a}")
            print(f"  Thermistor Coeff B: {thermistor_coeff_b}")
            print(f"  Thermistor Coeff C: {thermistor_coeff_c}")
            print(f"  Swap Start Frequency: {swap_start_frequency}")
            print(f"  Swap End Frequency: {swap_end_frequency}")
            
            # 模拟处理参数（在实际应用中可能需要保存到数据结构中）
            # 这里假设处理成功
            return_code = 0
            
            # 构建响应帧：[地址][0x66][通道编号][ReturnCode][CRC*2]
            response = struct.pack('>BBBB', self.slave_id, 0x66, channel, return_code)
            return self.add_crc(response)
            
        except Exception as e:
            log.error(f"Error handling 0x66 command: {e}")
            # 异常响应
            channel = request_frame[2] if len(request_frame) > 2 else 0x00
            error_response = struct.pack('>BBB', self.slave_id, 0xE6, channel)
            return self.add_crc(error_response)
    
    def handle_0x67_command(self, request_frame):
        """处理0x67指令：通道一次性采样"""
        try:
            # 解析请求帧：[地址][0x67][通道编号][N bytes][Parameters][CRC*2]
            if len(request_frame) < 7:  # 地址(1) + 功能码(1) + 通道编号(1) + N bytes(1) + CRC(2)
                # 异常响应：[地址][0xE7][通道编号][CRC*2]
                channel = request_frame[2] if len(request_frame) > 2 else 0x00
                error_response = struct.pack('>BBB', self.slave_id, 0xE7, channel)
                return self.add_crc(error_response)
            
            channel = request_frame[2]
            n_bytes = request_frame[3]
            
            # 验证通道编号（1-8）
            if not (1 <= channel <= 8):
                error_response = struct.pack('>BBB', self.slave_id, 0xE7, channel)
                return self.add_crc(error_response)
            
            # 验证参数长度
            if n_bytes != 17:
                error_response = struct.pack('>BBB', self.slave_id, 0xE7, channel)
                return self.add_crc(error_response)
            
            # 验证整个帧的长度
            expected_length = 1 + 1 + 1 + 1 + n_bytes + 2  # 地址 + 功能码 + 通道 + N bytes + Parameters + CRC
            if len(request_frame) != expected_length:
                error_response = struct.pack('>BBB', self.slave_id, 0xE7, channel)
                return self.add_crc(error_response)
            
            # 解析Parameters字段
            parameters = request_frame[4:4+n_bytes]
            
            # 解析Thermistor-Setting|Swap-Type
            thermistor_swap_byte = parameters[0]
            thermistor_setting = (thermistor_swap_byte & 0x02) >> 1  # 第二bit
            swap_type = thermistor_swap_byte & 0x01  # 第一bit
            
            # 解析Thermistor-Coeff-A/B/C
            thermistor_coeff_a = struct.unpack('>f', parameters[1:5])[0]
            thermistor_coeff_b = struct.unpack('>f', parameters[5:9])[0]
            thermistor_coeff_c = struct.unpack('>f', parameters[9:13])[0]
            
            # 解析Swap-Start/End-Frequency
            swap_start_frequency = struct.unpack('>H', parameters[13:15])[0]
            swap_end_frequency = struct.unpack('>H', parameters[15:17])[0]
            
            # 打印解析后的参数
            print(f"Channel {channel} One-time Sampling Parameters:")
            print(f"  Thermistor Setting: {'ON' if thermistor_setting else 'OFF'}")
            print(f"  Swap Type: {'Linear' if swap_type else 'Logarithmic'}")
            print(f"  Thermistor Coeff A: {thermistor_coeff_a}")
            print(f"  Thermistor Coeff B: {thermistor_coeff_b}")
            print(f"  Thermistor Coeff C: {thermistor_coeff_c}")
            print(f"  Swap Start Frequency: {swap_start_frequency}")
            print(f"  Swap End Frequency: {swap_end_frequency}")
            
            # 模拟执行一次性采样操作
            print(f"Executing one-time sampling on channel {channel}...")
            # 生成200到6500之间的随机浮点数作为orign_freq
            orign_freq = random.uniform(200, 6500)
            print(f"Generated random origin frequency: {orign_freq:.4f} Hz")
            vibwire_result = serial_get_result(orign_freq=orign_freq)
            vibwire_queue.append(vibwire_result)
            time.sleep(0.1)  # 模拟采样延迟
            
            # 模拟返回结果
            return_code = 0  # 成功
            fft_code = 0x00  # 8192 points
            
            # 构建响应帧：[地址][0x67][通道编号][ReturnCode][FFT Code][CRC*2]
            response = struct.pack('>BBBBB', self.slave_id, 0x67, channel, return_code, fft_code)
            return self.add_crc(response)
            
        except Exception as e:
            log.error(f"Error handling 0x67 command: {e}")
            # 异常响应
            channel = request_frame[2] if len(request_frame) > 2 else 0x00
            error_response = struct.pack('>BBB', self.slave_id, 0xE7, channel)
            return self.add_crc(error_response)
    
    def handle_0x68_command(self, request_frame):
        """处理0x68指令：通道一次性采样结果查询"""
        try:
            # 解析请求帧：[地址][0x68][通道编号][CRC*2]
            if len(request_frame) < 5:  # 地址(1) + 功能码(1) + 通道编号(1) + CRC(2)
                # 异常响应：[地址][0xE8][通道编号][CRC*2]
                channel = request_frame[2] if len(request_frame) > 2 else 0x00
                error_response = struct.pack('>BBB', self.slave_id, 0xE8, channel)
                return self.add_crc(error_response)
            
            channel = request_frame[2]
            
            # 验证通道编号（1-8）
            if not (1 <= channel <= 8):
                error_response = struct.pack('>BBB', self.slave_id, 0xE8, channel)
                return self.add_crc(error_response)
            
            # 生成模拟数据（10个浮点数） - 这里可以模拟一次性采样的结果
            if len(vibwire_queue) == 0:
                # 构建响应帧：[地址][0x68][通道编号][N bytes][CRC*2]
                n_bytes = 0
                response = struct.pack('>BBBB', self.slave_id, 0x68, channel, n_bytes)
                return self.add_crc(response)
            
            # 从队列获取最新的VibwireResult
            vibwire_res = vibwire_queue[len(vibwire_queue)-1]
            signal_frequency = vibwire_res.sensor_freq
            signal_amplitude = vibwire_res.sensor_amplitude
            est_noise_frequency = vibwire_res.est_noise_freq
            est_noise_amplitude = vibwire_res.est_noise_amplitude
            signal_to_noise_ratio = vibwire_res.sensor_to_noise_ratio
            decay_ratio = vibwire_res.decay_ratio
            temperature = 20.0 + channel * 1.5  # 21.5, 23.0, ..., 32.5
            voltage_excitation_amplitude = 3.0 + channel * 0.4  # 3.4, 3.8, ..., 6.2
            system_sample_rate = 20000.0 + channel * 2000  # 22000.0, 24000.0, ..., 36000.0
            one_process_time = 0.2 + channel * 0.02  # 0.22, 0.24, ..., 0.36
            
            # 构建数据部分：10个浮点数（40字节）
            data = struct.pack('>10f', 
                              signal_frequency,
                              signal_amplitude,
                              est_noise_frequency,
                              est_noise_amplitude,
                              signal_to_noise_ratio,
                              decay_ratio,
                              temperature,
                              voltage_excitation_amplitude,
                              system_sample_rate,
                              one_process_time)
            
            # 构建响应帧：[地址][0x68][通道编号][N bytes][data][CRC*2]
            n_bytes = len(data)  # 40 bytes
            response = struct.pack('>BBBB', self.slave_id, 0x68, channel, n_bytes)
            response += data
            
            return self.add_crc(response)
            
        except Exception as e:
            log.error(f"Error handling 0x68 command: {e}")
            # 异常响应
            channel = request_frame[2] if len(request_frame) > 2 else 0x00
            error_response = struct.pack('>BBB', self.slave_id, 0xE8, channel)
            return self.add_crc(error_response)
    
    def handle_0x69_command(self, request_frame):
        """处理0x69指令：通道一次性采样结果的原始数据查询"""
        try:
            # 解析请求帧：[地址][0x69][通道编号][data_type][offset][CRC*2]
            if len(request_frame) < 7:  # 地址(1) + 功能码(1) + 通道(1) + data_type(1) + offset(2) + CRC(2)
                # 异常响应：[地址][0xE9][通道编号][CRC*2]
                channel = request_frame[2] if len(request_frame) > 2 else 0x00
                error_response = struct.pack('>BBB', self.slave_id, 0xE9, channel)
                return self.add_crc(error_response)
            
            channel = request_frame[2]
            data_type = request_frame[3]
            offset = struct.unpack('>H', request_frame[4:6])[0]
            
            # 验证通道编号（1-8）
            if not (1 <= channel <= 8):
                error_response = struct.pack('>BBB', self.slave_id, 0xE9, channel)
                return self.add_crc(error_response)
            
            # 验证数据类型（0或1）
            if data_type not in [0, 1]:
                error_response = struct.pack('>BBB', self.slave_id, 0xE9, channel)
                return self.add_crc(error_response)
            
            
            print(f"Querying raw data - Channel: {channel}, Data Type: {data_type}, Offset: {offset}")
            
            # 模拟生成原始数据
            # 每一个offset对应248个字节的数据块，但实际数据最大240字节
            # 这里模拟根据offset返回不同的数据
            data_type_str = "Raw Signal" if data_type == 0 else "FFT Data"
            
            if len(vibwire_queue) == 0:
                # 构建响应帧：[地址][0x68][通道编号][N bytes][CRC*2]
                n_bytes = 0
                response = struct.pack('>BBBBHB', self.slave_id, 0x69, channel, data_type, offset, n_bytes)
                return self.add_crc(response)
            
            vibwire_res = vibwire_queue[len(vibwire_queue)-1]
            if data_type == 0:
                left_bytes = len(vibwire_res.signal) * 4 - offset * 240
                if left_bytes < 240:
                    n_bytes = left_bytes
                else:
                    n_bytes = 240
                float_count = n_bytes // 4
                float_data = vibwire_res.signal[offset*60 :offset*60 + float_count]
            else:
                left_bytes = len(vibwire_res.fft_magnitude) * 4 - offset * 240
                if left_bytes < 240:
                    n_bytes = left_bytes
                else:
                    n_bytes = 240
                float_count = n_bytes // 4
                float_data = vibwire_res.fft_magnitude[offset*60 :offset*60 + float_count]

            # 构建数据部分，确保data是二进制字符串
            if float_count > 0:
                data = struct.pack(f'>{float_count}f', *float_data)
            else:
                data = b""
            
            # 构建响应帧：[地址][0x69][通道编号][data_type][offset][N bytes][data][CRC*2]
            response = struct.pack('>BBBBHB', self.slave_id, 0x69, channel, data_type, offset, n_bytes)
            response += data
            
            return self.add_crc(response)
            
        except Exception as e:
            log.error(f"Error handling 0x69 command: {e}")
            # 异常响应
            channel = request_frame[2] if len(request_frame) > 2 else 0x00
            error_response = struct.pack('>BBB', self.slave_id, 0xE9, channel)
            return self.add_crc(error_response)
    
    def handle_serial_data(self, serial_port):
        """处理串口数据的主循环"""
        import time
        
        buffer = b""
        last_char_time = time.time()
        
        # 计算3.5个字符时间的超时值（单位：秒）
        # 每个字符包括：1个起始位 + 8个数据位 + 1个校验位 + 1个停止位 = 11位
        bits_per_char = 11
        char_time = bits_per_char / self.baudrate
        inter_char_timeout = 3.5 * char_time
        
        print(f"Modbus RTU inter-character timeout: {inter_char_timeout:.6f} seconds")
        
        while True:
            try:
                # 设置串口读取超时为inter_char_timeout的1/10，以便更频繁地检查超时
                serial_port.timeout = inter_char_timeout / 10
                
                # 读取一个字节
                data = serial_port.read(1)
                current_time = time.time()
                
                if data:
                    # 收到数据，更新最后字符时间
                    buffer += data
                    last_char_time = current_time
                    # print(f"[Serial RX] Byte: {data.hex(' ')}, Buffer: {buffer.hex(' ')}")
                else:
                    # 没有收到数据，检查是否超时
                    if len(buffer) > 0 and (current_time - last_char_time) > inter_char_timeout:
                        print(f"[Serial RX] Frame timeout, processing buffer: {buffer.hex(' ')}")
                        
                        # 处理完整的帧
                        if len(buffer) >= 4:  # 最小帧长度
                            # 尝试从缓冲区末尾向前查找完整的帧
                            for i in range(max(0, len(buffer) - 200), len(buffer) - 3):
                                possible_frame = buffer[i:]
                                
                                if self.verify_crc(possible_frame):
                                    print(f"Found complete frame: {possible_frame.hex(' ')}")
                                    
                                    address = possible_frame[0]
                                    function_code = possible_frame[1]
                                    
                                    if address == self.slave_id:
                                        print(f"Received command: Function Code 0x{function_code:02X}")
                                        
                                        response = None
                                        if function_code == 0x65:
                                            response = self.handle_0x65_command(possible_frame)
                                        elif function_code == 0x66:
                                            response = self.handle_0x66_command(possible_frame)
                                        elif function_code == 0x67:
                                            response = self.handle_0x67_command(possible_frame)
                                        elif function_code == 0x68:
                                            response = self.handle_0x68_command(possible_frame)
                                        elif function_code == 0x69:
                                            response = self.handle_0x69_command(possible_frame)
                                        
                                        if response:
                                            # print(f"[Serial TX] {response.hex(' ')}")
                                            print(f"[Serial TX] {len(response)}")
                                            serial_port.write(response)
                                            serial_port.flush()
                                    
                                    # 清空缓冲区，保留可能的下一个帧的开头
                                    buffer = buffer[i+len(possible_frame):]
                                    break
                            else:
                                # 没有找到有效的CRC，清空缓冲区
                                print("No valid frame found, clearing buffer")
                                buffer = b""
                        else:
                            # 帧太短，清空缓冲区
                            print("Frame too short, clearing buffer")
                            buffer = b""
            
            except Exception as e:
                log.error(f"Error handling serial data: {e}")
                buffer = b""
    
    def start_server(self):
        """启动Modbus服务器"""
        try:
            # 打印串口配置信息
            log.info(f"Starting Custom Modbus Server on port {self.port}")
            log.info(f"Baudrate: {self.baudrate}, Parity: {self.parity}, Stopbits: {self.stopbits}")
            log.info(f"Device Address: {self.slave_id}")
            
            # 打开串口
            serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=self.timeout
            )
            
            print(f"Serial port {self.port} opened successfully")
            
            try:
                # 启动数据处理主循环
                self.handle_serial_data(serial_port)
            finally:
                serial_port.close()
                print(f"Serial port {self.port} closed")
            
        except Exception as e:
            log.error(f"Error starting server: {e}")
            raise

def main():
    """主函数"""
    print("=== Serial Modbus Server ===")
    
    # 创建服务器实例
    server = SerialModbusServer()
    
    # 启动服务器
    try:
        print("Starting Modbus server...")
        print("Press Ctrl+C to stop.")
        server.start_server()
    except KeyboardInterrupt:
        print("\nServer stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
        print("Please check your serial port configuration and try again.")

if __name__ == "__main__":
    main()
