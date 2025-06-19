#!/usr/bin/env python3
"""
Log Decoder for Foot Sensor and BHI360 Log Files

This script decodes protobuf-based log files and displays their contents.
It requires the generated Python protobuf files.

Usage:
    python decode_log.py <log_file_path>
    python decode_log.py --list <directory_path>
"""

import sys
import os
import struct
import argparse
from pathlib import Path

# Try to import the generated protobuf modules
try:
    sys.path.append(os.path.join(os.path.dirname(__file__), '../build/sensing_fw'))
    import foot_sensor_messages_pb2 as foot_pb
    import bhi360_sensor_messages_pb2 as bhi360_pb
except ImportError:
    print("Error: Cannot import protobuf modules.")
    print("Please build the project first: cd .. && west build")
    sys.exit(1)


class LogDecoder:
    def __init__(self):
        self.absolute_time_ms = 0
        self.packet_count = 0
        self.sampling_frequency = 0
        self.firmware_version = ""
        
        # Statistics
        self.min_delta = float('inf')
        self.max_delta = 0
        self.total_delta = 0
        self.delta_count = 0
        
        # BHI360 specific
        self.min_quat_accuracy = float('inf')
        self.max_quat_accuracy = float('-inf')
        self.min_step_count = float('inf')
        self.max_step_count = 0

    def read_protobuf_message(self, file):
        """Read a length-delimited protobuf message"""
        # Try to read size (varint)
        size_bytes = []
        while True:
            byte = file.read(1)
            if not byte:
                return None
            b = ord(byte)
            size_bytes.append(b & 0x7F)
            if (b & 0x80) == 0:
                break
        
        # Decode varint
        size = 0
        for i, b in enumerate(size_bytes):
            size |= b << (7 * i)
        
        # Read message data
        data = file.read(size)
        if len(data) != size:
            return None
        
        return data

    def decode_foot_sensor_log(self, file_path):
        """Decode a foot sensor log file"""
        print(f"\n=== FOOT SENSOR LOG DECODER ===")
        print(f"File: {file_path}")
        print(f"Size: {os.path.getsize(file_path)} bytes")
        
        with open(file_path, 'rb') as f:
            # Try different reading methods
            file_content = f.read()
            f.seek(0)
            
            print("\n--- DECODING LOG CONTENTS ---")
            
            # Try to parse as a stream of messages
            offset = 0
            while offset < len(file_content):
                try:
                    # Try to parse from current position
                    msg = foot_pb.FootSensorLogMessage()
                    
                    # Try to find message boundary
                    found = False
                    for length in range(1, min(256, len(file_content) - offset)):
                        try:
                            msg.ParseFromString(file_content[offset:offset+length])
                            found = True
                            offset += length
                            break
                        except:
                            continue
                    
                    if not found:
                        offset += 1
                        continue
                    
                    # Process message based on type
                    if msg.HasField('sensing_data'):
                        print("\nHEADER:")
                        print(f"  Firmware: {msg.sensing_data.firmware_version}")
                        print(f"  Frequency: {msg.sensing_data.sampling_frequency} Hz")
                        self.sampling_frequency = msg.sensing_data.sampling_frequency
                        
                    elif msg.HasField('foot_sensor'):
                        delta = msg.foot_sensor.delta_ms
                        
                        if self.packet_count > 0:
                            self.min_delta = min(self.min_delta, delta)
                            self.max_delta = max(self.max_delta, delta)
                            self.total_delta += delta
                            self.delta_count += 1
                        
                        if self.packet_count < 5:
                            print(f"\nPacket {self.packet_count}:")
                            print(f"  Delta: {delta} ms")
                            print(f"  Time: {self.absolute_time_ms} ms ({self.absolute_time_ms/1000:.2f} s)")
                            print(f"  Readings: {list(msg.foot_sensor.readings)}")
                        
                        if self.packet_count == 0:
                            self.absolute_time_ms = 0
                        else:
                            self.absolute_time_ms += delta
                        
                        self.packet_count += 1
                        
                    elif msg.HasField('session_end'):
                        print(f"\nSESSION END:")
                        print(f"  Uptime: {msg.session_end.uptime_ms} ms")
                        print(f"  Total Packets: {self.packet_count}")
                        print(f"  Duration: {self.absolute_time_ms/1000:.2f} seconds")
                        break
                        
                except Exception as e:
                    continue
            
            self._print_statistics()

    def decode_bhi360_log(self, file_path):
        """Decode a BHI360 log file"""
        print(f"\n=== BHI360 LOG DECODER ===")
        print(f"File: {file_path}")
        print(f"Size: {os.path.getsize(file_path)} bytes")
        
        with open(file_path, 'rb') as f:
            file_content = f.read()
            
            print("\n--- DECODING LOG CONTENTS ---")
            
            offset = 0
            while offset < len(file_content):
                try:
                    msg = bhi360_pb.BHI360LogMessage()
                    
                    # Try to find message boundary
                    found = False
                    for length in range(1, min(256, len(file_content) - offset)):
                        try:
                            msg.ParseFromString(file_content[offset:offset+length])
                            found = True
                            offset += length
                            break
                        except:
                            continue
                    
                    if not found:
                        offset += 1
                        continue
                    
                    # Process message based on type
                    if msg.HasField('sensing_data'):
                        print("\nHEADER:")
                        print(f"  Firmware: {msg.sensing_data.firmware_version}")
                        print(f"  Frequency: {msg.sensing_data.sampling_frequency} Hz")
                        self.sampling_frequency = msg.sensing_data.sampling_frequency
                        
                    elif msg.HasField('bhi360_log_record'):
                        rec = msg.bhi360_log_record
                        delta = rec.delta_ms
                        
                        if self.packet_count > 0:
                            self.min_delta = min(self.min_delta, delta)
                            self.max_delta = max(self.max_delta, delta)
                            self.total_delta += delta
                            self.delta_count += 1
                        
                        self.min_quat_accuracy = min(self.min_quat_accuracy, rec.quat_accuracy)
                        self.max_quat_accuracy = max(self.max_quat_accuracy, rec.quat_accuracy)
                        self.min_step_count = min(self.min_step_count, rec.step_count)
                        self.max_step_count = max(self.max_step_count, rec.step_count)
                        
                        if self.packet_count < 5:
                            print(f"\nPacket {self.packet_count}:")
                            print(f"  Delta: {delta} ms")
                            print(f"  Time: {self.absolute_time_ms} ms ({self.absolute_time_ms/1000:.2f} s)")
                            print(f"  Quaternion: [{rec.quat_x:.3f}, {rec.quat_y:.3f}, {rec.quat_z:.3f}, {rec.quat_w:.3f}]")
                            print(f"  Accuracy: {rec.quat_accuracy}")
                            print(f"  Linear Accel: [{rec.lacc_x:.3f}, {rec.lacc_y:.3f}, {rec.lacc_z:.3f}] m/s²")
                            # Check if gyroscope fields exist (for newer log files)
                            if hasattr(rec, 'gyro_x'):
                                print(f"  Gyroscope: [{rec.gyro_x:.3f}, {rec.gyro_y:.3f}, {rec.gyro_z:.3f}] rad/s")
                            print(f"  Step Count: {rec.step_count}")
                        
                        if self.packet_count == 0:
                            self.absolute_time_ms = 0
                        else:
                            self.absolute_time_ms += delta
                        
                        self.packet_count += 1
                        
                    elif msg.HasField('session_end'):
                        print(f"\nSESSION END:")
                        print(f"  Uptime: {msg.session_end.uptime_ms} ms")
                        print(f"  Total Packets: {self.packet_count}")
                        print(f"  Duration: {self.absolute_time_ms/1000:.2f} seconds")
                        break
                        
                except Exception as e:
                    continue
            
            self._print_statistics()
            self._print_bhi360_statistics()

    def _print_statistics(self):
        """Print timing statistics"""
        if self.delta_count > 0:
            avg_delta = self.total_delta / self.delta_count
            print(f"\nTIMING STATISTICS:")
            print(f"  Min Delta: {self.min_delta} ms")
            print(f"  Max Delta: {self.max_delta} ms")
            print(f"  Avg Delta: {avg_delta:.2f} ms")
            print(f"  Jitter: {self.max_delta - self.min_delta} ms")
            
            if self.sampling_frequency > 0:
                expected = 1000.0 / self.sampling_frequency
                deviation = ((avg_delta - expected) / expected) * 100
                print(f"  Expected: {expected:.2f} ms")
                print(f"  Deviation: {deviation:.2f}%")
                
                expected_packets = (self.absolute_time_ms / 1000.0) * self.sampling_frequency
                packet_loss = 100.0 * (1.0 - (self.packet_count / expected_packets))
                print(f"  Packet Loss: {packet_loss:.2f}%")

    def _print_bhi360_statistics(self):
        """Print BHI360-specific statistics"""
        print(f"\nBHI360 STATISTICS:")
        print(f"  Quaternion Accuracy: min={self.min_quat_accuracy:.1f}, max={self.max_quat_accuracy:.1f}")
        print(f"  Steps: start={self.min_step_count}, end={self.max_step_count}, total={self.max_step_count - self.min_step_count}")

    def list_logs(self, directory):
        """List all log files in a directory"""
        print(f"\n=== LOG FILE INVENTORY ===")
        print(f"Directory: {directory}")
        
        foot_files = []
        bhi360_files = []
        
        for file in Path(directory).glob("*.pb"):
            if "foot_" in file.name:
                foot_files.append(file)
            elif "bhi360_" in file.name:
                bhi360_files.append(file)
        
        print(f"\n{'FILENAME':<30} {'SIZE':<10} {'TYPE'}")
        print(f"{'-'*30} {'-'*10} {'-'*20}")
        
        total_size = 0
        for f in sorted(foot_files):
            size = f.stat().st_size
            total_size += size
            print(f"{f.name:<30} {size:<10} Foot Sensor")
        
        for f in sorted(bhi360_files):
            size = f.stat().st_size
            total_size += size
            print(f"{f.name:<30} {size:<10} BHI360")
        
        print(f"\nSUMMARY:")
        print(f"  Foot Sensor Logs: {len(foot_files)}")
        print(f"  BHI360 Logs: {len(bhi360_files)}")
        print(f"  Total Files: {len(foot_files) + len(bhi360_files)}")
        print(f"  Total Size: {total_size} bytes ({total_size/1024:.2f} KB)")


def main():
    parser = argparse.ArgumentParser(description='Decode sensor log files')
    parser.add_argument('path', help='Log file path or directory (with --list)')
    parser.add_argument('--list', action='store_true', help='List all log files in directory')
    
    args = parser.parse_args()
    
    decoder = LogDecoder()
    
    if args.list:
        decoder.list_logs(args.path)
    else:
        if not os.path.exists(args.path):
            print(f"Error: File not found: {args.path}")
            sys.exit(1)
        
        # Determine file type
        if "foot_" in args.path:
            decoder.decode_foot_sensor_log(args.path)
        elif "bhi360_" in args.path:
            decoder.decode_bhi360_log(args.path)
        else:
            print("Error: Cannot determine file type from filename")
            print("File should contain 'foot_' or 'bhi360_' in the name")
            sys.exit(1)


if __name__ == "__main__":
    main()