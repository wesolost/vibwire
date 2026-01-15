######################################################################################
# Description: Serial Modbus Client
# Author: AI Assistant
# Created: 2026-01-13
# Version: v1
######################################################################################
import serial
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType
import logging
import struct
import time

# Configure logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.INFO)

class SerialModbusClient:
    def __init__(self):
        self.port = 'COM10'
        self.baudrate = 115200
        self.bytesize = serial.EIGHTBITS
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.timeout = 1
        self.slave_id = 0x01
        self.client = None
        
    def crc16(self, data):
        """计算CRC16校验码"""
        crc = 0xFFFF
        polynomial = 0xA001  # CRC-16-CCITT
        
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ polynomial
                else:
                    crc >>= 1
        
        # 返回小端格式的CRC
        return crc & 0xFFFF
        
    def add_crc(self, data):
        """为数据添加CRC校验码"""
        crc = self.crc16(data)
        return data + struct.pack('<H', crc)
        
    def verify_crc(self, data):
        """验证CRC校验码"""
        if len(data) < 2:
            return False
        
        received_crc = struct.unpack('<H', data[-2:])[0]
        calculated_crc = self.crc16(data[:-2])
        
        return received_crc == calculated_crc
        
    def configure_serial(self, port='COM10', baudrate=9600, bytesize=serial.EIGHTBITS,
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
    
    def connect(self):
        """连接到Modbus服务器"""
        try:
            self.client = ModbusSerialClient(
                port=self.port,
                framer=FramerType.RTU,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=self.timeout,
            )
            
            if self.client.connect():
                print(f"Connected to {self.port} successfully")
                return True
            else:
                print(f"Failed to connect to {self.port}")
                return False
        except Exception as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.client and self.client.is_socket_open():
            self.client.close()
            print("Disconnected from server")
    

    
    def read_input_registers(self, address, count):
        """读取输入寄存器"""
        if not self.client or not self.client.is_socket_open():
            print("Not connected to server")
            return None
        
        try:
            result = self.client.read_input_registers(
                address=address,
                count=count,
                device_id=self.slave_id
            )
            
            if result.isError():
                print(f"Error reading input registers: {result}")
                return None
            else:
                print(f"Read input registers {address}-{address+count-1}: {result.registers}")
                return result.registers
        except Exception as e:
            print(f"Exception reading input registers: {e}")
            return None
    
    def read_coils(self, address, count):
        """读取线圈"""
        if not self.client or not self.client.is_socket_open():
            print("Not connected to server")
            return None
        
        try:
            result = self.client.read_coils(
                address=address,
                count=count,
                device_id=self.slave_id
            )
            
            if result.isError():
                print(f"Error reading coils: {result}")
                return None
            else:
                print(f"Read coils {address}-{address+count-1}: {result.bits}")
                return result.bits
        except Exception as e:
            print(f"Exception reading coils: {e}")
            return None
    
    def write_coil(self, address, value):
        """写入线圈"""
        if not self.client or not self.client.is_socket_open():
            print("Not connected to server")
            return False
        
        try:
            result = self.client.write_coil(
                address=address,
                value=value,
                device_id=self.slave_id
            )
            
            if result.isError():
                print(f"Error writing coil: {result}")
                return False
            else:
                print(f"Written value {value} to coil {address}")
                return True
        except Exception as e:
            print(f"Exception writing coil: {e}")
            return False
    
    def read_discrete_inputs(self, address, count):
        """读取离散输入"""
        if not self.client or not self.client.is_socket_open():
            print("Not connected to server")
            return None
        
        try:
            result = self.client.read_discrete_inputs(
                address=address,
                count=count,
                device_id=self.slave_id
            )
            
            if result.isError():
                print(f"Error reading discrete inputs: {result}")
                return None
            else:
                print(f"Read discrete inputs {address}-{address+count-1}: {result.bits}")
                return result.bits
        except Exception as e:
            print(f"Exception reading discrete inputs: {e}")
            return None
    
    def _get_serial_port(self):
        """获取底层串口对象"""
        if not self.client or not self.client.is_socket_open():
            return None
        # 在pymodbus 3.x中，底层串口对象通过socket属性访问
        if hasattr(self.client, 'socket'):
            return self.client.socket
        else:
            print("Error: Could not access serial port through client.socket")
            return None
    
    def query_channel_result(self, channel_number):
        """通道结果查询与响应指令(0x65)"""
        serial_port = self._get_serial_port()
        if not serial_port:
            print("Not connected to server")
            return None
        
        try:
            # 构建请求帧: [地址][0x65][通道编号][CRC*2]
            request_data = struct.pack('>BBB', self.slave_id, 0x65, channel_number)
            request_frame = self.add_crc(request_data)
            
            print(f"Sending query channel result command: {request_frame.hex(' ')}")
            
            # 清空接收缓冲区
            serial_port.flushInput()
            
            # 发送请求
            serial_port.write(request_frame)
            serial_port.flush()
            
            # 等待响应
            time.sleep(0.1)
            response_frame = serial_port.read(200)  # 足够大的缓冲区
            
            if not response_frame:
                print("No response received")
                return None
            
            print(f"Received response: {response_frame.hex(' ')}")
            
            # 验证CRC
            if not self.verify_crc(response_frame):
                print("CRC verification failed")
                return None
            
            # 解析响应帧
            if len(response_frame) < 4:  # 地址 + 功能码 + 通道编号 + CRC
                print("Invalid response length")
                return None
            
            address = response_frame[0]
            function_code = response_frame[1]
            response_channel = response_frame[2]
            
            if address != self.slave_id:
                print(f"Invalid response address: {address}")
                return None
            
            if response_channel != channel_number:
                print(f"Channel mismatch: requested {channel_number}, got {response_channel}")
                return None
            
            # 处理异常响应
            if function_code == 0xE5:
                print("Error response received: Invalid channel or operation failed")
                return None
            
            # 处理正常响应
            if function_code == 0x65:
                if len(response_frame) < 5:  # 地址 + 功能码 + 通道编号 + N bytes + CRC
                    print("Invalid response length")
                    return None
                
                n_bytes = response_frame[3]
                
                if n_bytes == 0:
                    print("No data available")
                    return {"channel": channel_number, "has_data": False}
                
                # 检查数据长度
                if len(response_frame) < 4 + n_bytes + 2:  # 地址 + 功能码 + 通道编号 + N bytes + data + CRC
                    print("Invalid data length")
                    return None
                
                # 解析数据
                data_bytes = response_frame[4:4+n_bytes]
                
                # 检查数据是否符合预期格式（40字节）
                if n_bytes == 40:
                    # 解析10个浮点数
                    data_values = struct.unpack('>10f', data_bytes)
                    
                    result = {
                        "channel": channel_number,
                        "has_data": True,
                        "data": {
                            "signal_frequency": data_values[0],
                            "signal_amplitude": data_values[1],
                            "noise_frequency": data_values[2],
                            "noise_amplitude": data_values[3],
                            "signal_to_noise_ratio": data_values[4],
                            "decay_ratio": data_values[5],
                            "temperature": data_values[6],
                            "voltage_excitation_amplitude": data_values[7],
                            "system_sample_rate": data_values[8],
                            "one_process_time": data_values[9]
                        }
                    }
                    
                    print("\nChannel Result Data:")
                    print(f"Signal Frequency: {result['data']['signal_frequency']:.2f}")
                    print(f"Signal Amplitude: {result['data']['signal_amplitude']:.2f}")
                    print(f"Noise Frequency: {result['data']['noise_frequency']:.2f}")
                    print(f"Noise Amplitude: {result['data']['noise_amplitude']:.2f}")
                    print(f"Signal-to-Noise Ratio: {result['data']['signal_to_noise_ratio']:.2f} dB")
                    print(f"Decay Ratio: {result['data']['decay_ratio']:.6f}")
                    print(f"Temperature: {result['data']['temperature']:.2f} °C")
                    print(f"Voltage Excitation Amplitude: {result['data']['voltage_excitation_amplitude']:.2f} V")
                    print(f"System Sample Rate: {result['data']['system_sample_rate']:.0f} Hz")
                    print(f"One Process Time: {result['data']['one_process_time']:.2f} ms")
                    
                    return result
                else:
                    print(f"Unexpected data length: {n_bytes} bytes")
                    return {"channel": channel_number, "has_data": True, "raw_data": data_bytes.hex(' ')}
            
            print(f"Unexpected function code: {function_code}")
            return None
            
        except Exception as e:
            print(f"Exception querying channel result: {e}")
            return None
    
    def start_channel_program(self, channel_number, thermistor_setting, swap_type,
                            thermistor_coeff_a, thermistor_coeff_b, thermistor_coeff_c,
                            swap_start_freq, swap_end_freq):
        """通道启动Program指令(0x66)"""
        serial_port = self._get_serial_port()
        if not serial_port:
            print("Not connected to server")
            return False
        
        try:
            # 构建Thermistor-Setting|Swap-Type字节
            # Bit 1: Thermistor-Setting (0=OFF, 1=ON)
            # Bit 0: Swap-Type (0=logarithmic, 1=linear)
            thermistor_swap_byte = 0x00
            if thermistor_setting:
                thermistor_swap_byte |= 0x02  # Bit 1
            if swap_type:
                thermistor_swap_byte |= 0x01  # Bit 0
            
            # 构建Parameters
            # [Thermistor-Setting|Swap-Type][Thermistor-Coeff-A][Thermistor-Coeff-B][Thermistor-Coeff-C][Swap-Start-Frequency][Swap-End-Frequency]
            parameters = struct.pack('>BfffHH', 
                                   thermistor_swap_byte,
                                   thermistor_coeff_a,
                                   thermistor_coeff_b,
                                   thermistor_coeff_c,
                                   int(swap_start_freq),
                                   int(swap_end_freq))
            
            # 计算N bytes (Parameters的长度)
            n_bytes = len(parameters)
            
            # 构建请求帧: [地址][0x66][通道编号][N bytes][Parameters][CRC*2]
            request_data = struct.pack('>BBBB', self.slave_id, 0x66, channel_number, n_bytes)
            request_data += parameters
            request_frame = self.add_crc(request_data)
            
            print(f"Sending start channel program command: {request_frame.hex(' ')}")
            
            # 清空接收缓冲区
            serial_port.flushInput()
            
            # 发送请求
            serial_port.write(request_frame)
            serial_port.flush()
            
            # 等待响应
            time.sleep(0.1)
            response_frame = serial_port.read(200)  # 足够大的缓冲区
            
            if not response_frame:
                print("No response received")
                return False
            
            print(f"Received response: {response_frame.hex(' ')}")
            
            # 验证CRC
            if not self.verify_crc(response_frame):
                print("CRC verification failed")
                return False
            
            # 解析响应帧
            if len(response_frame) < 5:  # 地址 + 功能码 + 通道编号 + ReturnCode + CRC
                print("Invalid response length")
                return False
            
            address = response_frame[0]
            function_code = response_frame[1]
            response_channel = response_frame[2]
            return_code = response_frame[3]
            
            if address != self.slave_id:
                print(f"Invalid response address: {address}")
                return False
            
            if response_channel != channel_number:
                print(f"Channel mismatch: requested {channel_number}, got {response_channel}")
                return False
            
            # 处理异常响应
            if function_code == 0xE6:
                print("Error response received: Invalid parameters or operation failed")
                return False
            
            # 处理正常响应
            if function_code == 0x66:
                if return_code == 0:
                    print("Channel program started successfully")
                    return True
                else:
                    print(f"Channel program failed with return code: {return_code}")
                    return False
            
            print(f"Unexpected function code: {function_code}")
            return False
            
        except Exception as e:
            print(f"Exception starting channel program: {e}")
            return False
    
    def one_time_sampling(self, channel_number, thermistor_setting, swap_type,
                         thermistor_coeff_a, thermistor_coeff_b, thermistor_coeff_c,
                         swap_start_freq, swap_end_freq):
        """通道一次性采样指令(0x67)"""
        serial_port = self._get_serial_port()
        if not serial_port:
            print("Not connected to server")
            return None
        
        try:
            # 构建Thermistor-Setting|Swap-Type字节
            # Bit 1: Thermistor-Setting (0=OFF, 1=ON)
            # Bit 0: Swap-Type (0=logarithmic, 1=linear)
            thermistor_swap_byte = 0x00
            if thermistor_setting:
                thermistor_swap_byte |= 0x02  # Bit 1
            if swap_type:
                thermistor_swap_byte |= 0x01  # Bit 0
            
            # 构建Parameters
            # [Thermistor-Setting|Swap-Type][Thermistor-Coeff-A][Thermistor-Coeff-B][Thermistor-Coeff-C][Swap-Start-Frequency][Swap-End-Frequency]
            parameters = struct.pack('>BfffHH', 
                                   thermistor_swap_byte,
                                   thermistor_coeff_a,
                                   thermistor_coeff_b,
                                   thermistor_coeff_c,
                                   int(swap_start_freq),
                                   int(swap_end_freq))
            
            # 计算N bytes (Parameters的长度)
            n_bytes = len(parameters)
            
            # 构建请求帧: [地址][0x67][通道编号][N bytes][Parameters][CRC*2]
            request_data = struct.pack('>BBBB', self.slave_id, 0x67, channel_number, n_bytes)
            request_data += parameters
            request_frame = self.add_crc(request_data)
            
            print(f"Sending one time sampling command: {request_frame.hex(' ')}")
            
            # 清空接收缓冲区
            serial_port.flushInput()
            
            # 发送请求
            serial_port.write(request_frame)
            serial_port.flush()
            
            # 等待响应
            time.sleep(0.1)
            response_frame = serial_port.read(200)  # 足够大的缓冲区
            
            if not response_frame:
                print("No response received")
                return None
            
            print(f"Received response: {response_frame.hex(' ')}")
            
            # 验证CRC
            if not self.verify_crc(response_frame):
                print("CRC verification failed")
                return None
            
            # 解析响应帧
            if len(response_frame) < 6:  # 地址 + 功能码 + 通道编号 + ReturnCode + FFT Code + CRC
                print("Invalid response length")
                return None
            
            address = response_frame[0]
            function_code = response_frame[1]
            response_channel = response_frame[2]
            
            if address != self.slave_id:
                print(f"Invalid response address: {address}")
                return None
            
            if response_channel != channel_number:
                print(f"Channel mismatch: requested {channel_number}, got {response_channel}")
                return None
            
            # 处理异常响应
            if function_code == 0xE7:
                print("Error response received: Invalid parameters or operation failed")
                return None
            
            # 处理正常响应
            if function_code == 0x67:
                return_code = response_frame[3]
                fft_code = response_frame[4]
                
                # 映射FFT Code到点数
                fft_points = 4096 if fft_code == 0x00 else 8192 if fft_code == 0x01 else fft_code
                
                if return_code == 0:
                    print(f"One-time sampling started successfully with FFT points: {fft_points}")
                    return {
                        "success": True,
                        "fft_points": fft_points,
                        "fft_code": fft_code
                    }
                else:
                    print(f"One-time sampling failed with return code: {return_code}")
                    return {
                        "success": False,
                        "return_code": return_code
                    }
            
            print(f"Unexpected function code: {function_code}")
            return None
            
        except Exception as e:
            print(f"Exception performing one-time sampling: {e}")
            return None
    
    def query_sampling_result(self, channel_number):
        """通道一次性采样结果查询指令(0x68)"""
        serial_port = self._get_serial_port()
        if not serial_port:
            print("Not connected to server")
            return None
        
        try:
            # 构建请求帧: [地址][0x68][通道编号][CRC*2]
            request_data = struct.pack('>BBB', self.slave_id, 0x68, channel_number)
            request_frame = self.add_crc(request_data)
            
            print(f"Sending query sampling result command: {request_frame.hex(' ')}")
            
            # 清空接收缓冲区
            serial_port.flushInput()
            
            # 发送请求
            serial_port.write(request_frame)
            serial_port.flush()
            
            # 等待响应
            time.sleep(0.1)
            response_frame = serial_port.read(200)  # 足够大的缓冲区
            
            if not response_frame:
                print("No response received")
                return None
            
            print(f"Received response: {response_frame.hex(' ')}")
            
            # 验证CRC
            if not self.verify_crc(response_frame):
                print("CRC verification failed")
                return None
            
            # 解析响应帧
            if len(response_frame) < 4:  # 地址 + 功能码 + 通道编号 + CRC
                print("Invalid response length")
                return None
            
            address = response_frame[0]
            function_code = response_frame[1]
            response_channel = response_frame[2]
            
            if address != self.slave_id:
                print(f"Invalid response address: {address}")
                return None
            
            if response_channel != channel_number:
                print(f"Channel mismatch: requested {channel_number}, got {response_channel}")
                return None
            
            # 处理异常响应
            if function_code == 0xE8:
                print("Error response received: Invalid channel or operation failed")
                return None
            
            # 处理正常响应
            if function_code == 0x68:
                if len(response_frame) < 5:  # 地址 + 功能码 + 通道编号 + N bytes + CRC
                    print("Invalid response length")
                    return None
                
                n_bytes = response_frame[3]
                print(f"N bytes: {n_bytes}")
                if n_bytes == 0:
                    print("No sampling data available")
                    return {"channel": channel_number, "has_data": False}
                
                # 检查数据长度
                if len(response_frame) < 4 + n_bytes + 2:  # 地址 + 功能码 + 通道编号 + N bytes + data + CRC
                    print("Invalid data length")
                    return None
                
                # 解析数据
                data_bytes = response_frame[4:4+n_bytes]
                
                # 检查数据是否符合预期格式（40字节）
                if n_bytes == 40:
                    # 解析10个浮点数
                    data_values = struct.unpack('>10f', data_bytes)
                    
                    result = {
                        "channel": channel_number,
                        "has_data": True,
                        "data": {
                            "signal_frequency": data_values[0],
                            "signal_amplitude": data_values[1],
                            "noise_frequency": data_values[2],
                            "noise_amplitude": data_values[3],
                            "signal_to_noise_ratio": data_values[4],
                            "decay_ratio": data_values[5],
                            "temperature": data_values[6],
                            "voltage_excitation_amplitude": data_values[7],
                            "system_sample_rate": data_values[8],
                            "one_process_time": data_values[9]
                        }
                    }
                    
                    print("\nSampling Result Data:")
                    print(f"Signal Frequency: {result['data']['signal_frequency']:.6f}")
                    print(f"Signal Amplitude: {result['data']['signal_amplitude']:.6f}")
                    print(f"Noise Frequency: {result['data']['noise_frequency']:.6f}")
                    print(f"Noise Amplitude: {result['data']['noise_amplitude']:.6f}")
                    print(f"Signal-to-Noise Ratio: {result['data']['signal_to_noise_ratio']:.6f} dB")
                    print(f"Decay Ratio: {result['data']['decay_ratio']:.6f}")
                    print(f"Temperature: {result['data']['temperature']:.2f} °C")
                    print(f"Voltage Excitation Amplitude: {result['data']['voltage_excitation_amplitude']:.2f} V")
                    print(f"System Sample Rate: {result['data']['system_sample_rate']:.0f} Hz")
                    print(f"One Process Time: {result['data']['one_process_time']:.2f} ms")
                    
                    return result
                else:
                    print(f"Unexpected data length: {n_bytes} bytes")
                    return {"channel": channel_number, "has_data": True, "raw_data": data_bytes.hex(' ')}
            
            print(f"Unexpected function code: {function_code}")
            return None
            
        except Exception as e:
            print(f"Exception querying sampling result: {e}")
            return None
    
    def query_raw_data(self, channel_number, data_type, offset):
        """通道一次性采样结果的原始数据查询指令(0x69)"""
        serial_port = self._get_serial_port()
        if not serial_port:
            print("Not connected to server")
            return None
        
        try:
            data_type_str = "Raw Signal Data" if data_type == 0 else "FFT Data"
            
            # 初始化结果
            all_raw_data = []
            all_raw_bytes = b""
            current_offset = 0
            total_bytes = 0
            total_floats = 0
            
            print(f"\n=== Starting to query {data_type_str} (Channel {channel_number}, Offset {offset}) ===")
            
            while True:
                # 构建请求帧: [地址][0x69][通道编号][data_type][offset][CRC*2]
                request_data = struct.pack('>BBBBH', self.slave_id, 0x69, channel_number, data_type, current_offset)
                request_frame = self.add_crc(request_data)
                
                print(f"\nSending query (Offset: {current_offset}): {request_frame.hex(' ')}")
                
                # 清空接收缓冲区
                serial_port.flushInput()
                
                # 发送请求
                serial_port.write(request_frame)
                serial_port.flush()
                
                # 等待响应
                time.sleep(0.1)
                response_frame = serial_port.read(300)  # 足够大的缓冲区
                
                if not response_frame:
                    print("No response received")
                    break
                
                print(f"Received response: {response_frame.hex(' ')}")
                
                # 验证CRC
                if not self.verify_crc(response_frame):
                    print("CRC verification failed")
                    break
                
                # 解析响应帧
                if len(response_frame) < 7:  # 地址 + 功能码 + 通道编号 + data_type + offset + CRC
                    print("Invalid response length")
                    break
                
                address = response_frame[0]
                function_code = response_frame[1]
                response_channel = response_frame[2]
                response_data_type = response_frame[3]
                response_offset = struct.unpack('>H', response_frame[4:6])[0]
                
                if address != self.slave_id:
                    print(f"Invalid response address: {address}")
                    break
                
                if response_channel != channel_number:
                    print(f"Channel mismatch: requested {channel_number}, got {response_channel}")
                    break
                
                if response_data_type != data_type:
                    print(f"Data type mismatch: requested {data_type}, got {response_data_type}")
                    break
                
                # 处理异常响应
                if function_code == 0xE9:
                    print("Error response received: Invalid parameters or operation failed")
                    break
                
                # 处理正常响应
                if function_code == 0x69:
                    if len(response_frame) < 8:  # 地址 + 功能码 + 通道编号 + data_type + offset + N bytes + CRC
                        print("Invalid response length")
                        break
                    
                    n_bytes = response_frame[6]
                    
                    if n_bytes == 0:
                        print("No raw data available")
                        break
                    
                    # 检查数据长度
                    if len(response_frame) < 7 + n_bytes + 2:  # 地址 + 功能码 + 通道编号 + data_type + offset + N bytes + data + CRC
                        print("Invalid data length")
                        break
                    
                    # 解析数据
                    data_bytes = response_frame[7:7+n_bytes]
                    
                    # 尝试解析为浮点数
                    raw_data = []
                    if n_bytes % 4 == 0:  # 检查是否可以解析为浮点数
                        num_floats = n_bytes // 4
                        try:
                            raw_data = struct.unpack(f'>{num_floats}f', data_bytes)
                            print(f"Parsed {num_floats} floating point values from {data_type_str} (Offset {current_offset})")
                            
                            # 累加数据
                            all_raw_data.extend(raw_data)
                            all_raw_bytes += data_bytes
                            total_bytes += n_bytes
                            total_floats += num_floats
                            
                        except Exception as e:
                            print(f"Failed to parse data as floats: {e}")
                            print(f"Raw data (hex): {data_bytes.hex(' ')}")
                            break
                    else:
                        print(f"Data length ({n_bytes}) is not a multiple of 4, cannot parse as floats")
                        print(f"Raw data (hex): {data_bytes.hex(' ')}")
                        break
                    
                    # 判断是否还有更多数据
                    if n_bytes < 240:
                        print(f"All data has been read! Total bytes: {total_bytes}, Total floats: {total_floats}")
                        break
                    else:
                        print(f"More data available, continuing to read...")
                        current_offset += 1  # 增加偏移量继续读取
                
                else:
                    print(f"Unexpected function code: {function_code}")
                    break
            
            # 返回整合后的结果之前，打印all_raw_bytes的float形式
            if all_raw_bytes:
                print(f"\n=== Final Data Summary ===")
                print(f"Total raw bytes length: {len(all_raw_bytes)} bytes")
                
                if len(all_raw_bytes) % 4 == 0:
                    num_floats = len(all_raw_bytes) // 4
                    print(f"Total float values: {num_floats}")
                    
                    try:
                        # 转换为float数组
                        final_floats = struct.unpack(f'>{num_floats}f', all_raw_bytes)
                        
                        # 打印每个float值，每行10个，保留3位小数
                        print("\nAll float values:")
                        for i, value in enumerate(final_floats):
                            print(f"{value:8.3f}", end=" ")
                            if (i + 1) % 10 == 0:
                                print()
                        if num_floats % 10 != 0:
                            print()  # 确保最后一行换行
                            
                    except Exception as e:
                        print(f"Failed to parse final raw bytes as floats: {e}")
                else:
                    print(f"Raw bytes length ({len(all_raw_bytes)}) is not a multiple of 4, cannot parse as floats")
            
            # 返回整合后的结果
            if all_raw_data:
                return {
                    "channel": channel_number,
                    "data_type": data_type,
                    "start_offset": offset,
                    "end_offset": current_offset,
                    "has_data": True,
                    "total_bytes": total_bytes,
                    "total_floats": total_floats,
                    "data": all_raw_data,
                    "data_type_str": data_type_str,
                    "raw_bytes": all_raw_bytes,
                    "more_data_available": False  # 所有数据都已读取完成
                }
            else:
                return {
                    "channel": channel_number,
                    "data_type": data_type,
                    "start_offset": offset,
                    "has_data": False
                }
            
        except Exception as e:
            print(f"Exception querying raw data: {e}")
            return None

def main():
    """主函数"""
    print("=== Serial Modbus Client ===")
    
    # 创建客户端实例
    client = SerialModbusClient()
    
    if not client.connect():
        return
    
    try:
        while True:
            print("\nMenu:")
            print("1. Query Channel Result (0x65)")
            print("2. Start Channel Program (0x66)")
            print("3. One-time Sampling (0x67)")
            print("4. Query Sampling Result (0x68)")
            print("5. Query Raw Data (0x69)")
            print("6. Exit")
            
            choice = input("Enter your choice (1-6): ")
            
            if choice == '1':
                # Query Channel Result (0x65)
                print("Channel mapping:")
                print("A1-0x01, A2-0x02, A3-0x03, A4-0x04")
                print("B1-0x05, B2-0x06, B3-0x07, B4-0x08")
                try:
                    channel_input = input("Enter channel number (1-8) [default: 1]: ")
                    channel = int(channel_input) if channel_input else 1
                    if 1 <= channel <= 8:
                        client.query_channel_result(channel)
                    else:
                        print("Invalid channel number")
                except ValueError:
                    print("Invalid input, please enter a number")
            
            elif choice == '2':
                # Start Channel Program (0x66)
                print("Channel mapping:")
                print("A1-0x01, A2-0x02, A3-0x03, A4-0x04")
                print("B1-0x05, B2-0x06, B3-0x07, B4-0x08")
                try:
                    channel_input = input("Enter channel number (1-8) [default: 1]: ")
                    channel = int(channel_input) if channel_input else 1
                    if not (1 <= channel <= 8):
                        print("Invalid channel number")
                        continue
                    
                    thermistor_input = input("Thermistor Setting (0=OFF, 1=ON) [default: 1]: ")
                    thermistor_setting = thermistor_input == '1' if thermistor_input else True
                    
                    swap_input = input("Swap Type (0=logarithmic, 1=linear) [default: 0]: ")
                    swap_type = swap_input == '1' if swap_input else False
                    
                    coeff_a_input = input("Thermistor Coeff A [default: 0.0]: ")
                    coeff_a = float(coeff_a_input) if coeff_a_input else 0.0
                    
                    coeff_b_input = input("Thermistor Coeff B [default: 0.0]: ")
                    coeff_b = float(coeff_b_input) if coeff_b_input else 0.0
                    
                    coeff_c_input = input("Thermistor Coeff C [default: 0.0]: ")
                    coeff_c = float(coeff_c_input) if coeff_c_input else 0.0
                    
                    start_freq_input = input("Swap Start Frequency (0-65535) [default: 100]: ")
                    start_freq = int(start_freq_input) if start_freq_input else 100
                    if not (0 <= start_freq <= 65535):
                        print("Invalid start frequency, must be between 0 and 65535")
                        continue
                    
                    end_freq_input = input("Swap End Frequency (0-65535) [default: 1000]: ")
                    end_freq = int(end_freq_input) if end_freq_input else 1000
                    if not (0 <= end_freq <= 65535):
                        print("Invalid end frequency, must be between 0 and 65535")
                        continue
                    
                    client.start_channel_program(
                        channel, thermistor_setting, swap_type,
                        coeff_a, coeff_b, coeff_c,
                        start_freq, end_freq
                    )
                except ValueError:
                    print("Invalid input, please check your values")
            
            elif choice == '3':
                # One-time Sampling (0x67)
                print("Channel mapping:")
                print("A1-0x01, A2-0x02, A3-0x03, A4-0x04")
                print("B1-0x05, B2-0x06, B3-0x07, B4-0x08")
                try:
                    channel_input = input("Enter channel number (1-8) [default: 1]: ")
                    channel = int(channel_input) if channel_input else 1
                    if not (1 <= channel <= 8):
                        print("Invalid channel number")
                        continue
                    
                    thermistor_input = input("Thermistor Setting (0=OFF, 1=ON) [default: 1]: ")
                    thermistor_setting = thermistor_input == '1' if thermistor_input else True
                    
                    swap_input = input("Swap Type (0=logarithmic, 1=linear) [default: 0]: ")
                    swap_type = swap_input == '1' if swap_input else False
                    
                    coeff_a_input = input("Thermistor Coeff A [default: 0.0]: ")
                    coeff_a = float(coeff_a_input) if coeff_a_input else 0.0
                    
                    coeff_b_input = input("Thermistor Coeff B [default: 0.0]: ")
                    coeff_b = float(coeff_b_input) if coeff_b_input else 0.0
                    
                    coeff_c_input = input("Thermistor Coeff C [default: 0.0]: ")
                    coeff_c = float(coeff_c_input) if coeff_c_input else 0.0
                    
                    start_freq_input = input("Swap Start Frequency (0-65535) [default: 100]: ")
                    start_freq = int(start_freq_input) if start_freq_input else 100
                    if not (0 <= start_freq <= 65535):
                        print("Invalid start frequency, must be between 0 and 65535")
                        continue
                    
                    end_freq_input = input("Swap End Frequency (0-65535) [default: 1000]: ")
                    end_freq = int(end_freq_input) if end_freq_input else 1000
                    if not (0 <= end_freq <= 65535):
                        print("Invalid end frequency, must be between 0 and 65535")
                        continue
                    
                    client.one_time_sampling(
                        channel, thermistor_setting, swap_type,
                        coeff_a, coeff_b, coeff_c,
                        start_freq, end_freq
                    )
                except ValueError:
                    print("Invalid input, please check your values")
            
            elif choice == '4':
                # Query Sampling Result (0x68)
                print("Channel mapping:")
                print("A1-0x01, A2-0x02, A3-0x03, A4-0x04")
                print("B1-0x05, B2-0x06, B3-0x07, B4-0x08")
                try:
                    channel_input = input("Enter channel number (1-8) [default: 1]: ")
                    channel = int(channel_input) if channel_input else 1
                    if 1 <= channel <= 8:
                        client.query_sampling_result(channel)
                    else:
                        print("Invalid channel number")
                except ValueError:
                    print("Invalid input, please enter a number")
            
            elif choice == '5':
                # Query Raw Data (0x69)
                print("Channel mapping:")
                print("A1-0x01, A2-0x02, A3-0x03, A4-0x04")
                print("B1-0x05, B2-0x06, B3-0x07, B4-0x08")
                try:
                    channel_input = input("Enter channel number (1-8) [default: 1]: ")
                    channel = int(channel_input) if channel_input else 1
                    if not (1 <= channel <= 8):
                        print("Invalid channel number")
                        continue
                    
                    data_type_input = input("Data Type (0=Raw Signal, 1=FFT Data) [default: 0]: ")
                    data_type = int(data_type_input) if data_type_input else 0
                    if data_type not in [0, 1]:
                        print("Invalid data type")
                        continue
                    
                    offset_input = input("Offset (0, 1, 2, ...) [default: 0]: ")
                    offset = int(offset_input) if offset_input else 0
                    
                    client.query_raw_data(channel, data_type, offset)
                except ValueError:
                    print("Invalid input, please check your values")
            
            elif choice == '6':
                print("Exiting program")
                break
            
            else:
                print("Invalid choice, please try again")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        client.disconnect()

if __name__ == "__main__":
    main()
