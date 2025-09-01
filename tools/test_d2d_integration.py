#!/usr/bin/env python3
"""
D2D Integration Test Script
Tests device-to-device communication between primary and secondary devices

Usage:
    python3 test_d2d_integration.py --primary /dev/ttyACM0 --secondary /dev/ttyACM1
"""

import argparse
import serial
import time
import re
import sys
from datetime import datetime
from typing import Optional, Tuple, List

class Colors:
    """ANSI color codes for terminal output"""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

class D2DTestRunner:
    def __init__(self, primary_port: str, secondary_port: str, baudrate: int = 115200):
        self.primary_port = primary_port
        self.secondary_port = secondary_port
        self.baudrate = baudrate
        self.primary_serial = None
        self.secondary_serial = None
        self.test_results = []
        
    def connect(self) -> bool:
        """Connect to both devices"""
        try:
            print(f"{Colors.CYAN}Connecting to devices...{Colors.RESET}")
            self.primary_serial = serial.Serial(self.primary_port, self.baudrate, timeout=1)
            self.secondary_serial = serial.Serial(self.secondary_port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for devices to stabilize
            
            # Clear buffers
            self.primary_serial.reset_input_buffer()
            self.secondary_serial.reset_input_buffer()
            
            print(f"{Colors.GREEN}✓ Connected to primary: {self.primary_port}{Colors.RESET}")
            print(f"{Colors.GREEN}✓ Connected to secondary: {self.secondary_port}{Colors.RESET}")
            return True
            
        except Exception as e:
            print(f"{Colors.RED}✗ Connection failed: {e}{Colors.RESET}")
            return False
    
    def disconnect(self):
        """Disconnect from devices"""
        if self.primary_serial:
            self.primary_serial.close()
        if self.secondary_serial:
            self.secondary_serial.close()
    
    def send_command(self, device: serial.Serial, command: str) -> bool:
        """Send a shell command to a device"""
        try:
            device.write(f"{command}\n".encode())
            time.sleep(0.1)
            return True
        except Exception as e:
            print(f"{Colors.RED}Failed to send command: {e}{Colors.RESET}")
            return False
    
    def read_output(self, device: serial.Serial, timeout: float = 2.0) -> List[str]:
        """Read output from device until timeout"""
        lines = []
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if device.in_waiting:
                try:
                    line = device.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        lines.append(line)
                except:
                    pass
            else:
                time.sleep(0.01)
        
        return lines
    
    def wait_for_pattern(self, device: serial.Serial, pattern: str, timeout: float = 10.0) -> Optional[str]:
        """Wait for a specific pattern in device output"""
        regex = re.compile(pattern)
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            lines = self.read_output(device, 0.1)
            for line in lines:
                match = regex.search(line)
                if match:
                    return line
        
        return None
    
    def log_test_result(self, test_name: str, passed: bool, details: str = ""):
        """Log test result"""
        self.test_results.append({
            'name': test_name,
            'passed': passed,
            'details': details,
            'timestamp': datetime.now()
        })
        
        status = f"{Colors.GREEN}PASS{Colors.RESET}" if passed else f"{Colors.RED}FAIL{Colors.RESET}"
        print(f"\n{Colors.BOLD}[{test_name}]{Colors.RESET} {status}")
        if details:
            print(f"  {details}")
    
    def test_d2d_connection(self) -> bool:
        """Test 1: Verify D2D connection establishment"""
        print(f"\n{Colors.YELLOW}=== Test 1: D2D Connection ==={Colors.RESET}")
        
        # Check primary device for secondary connection
        primary_output = self.read_output(self.primary_serial, 5.0)
        secondary_connected = any("Secondary device connected" in line for line in primary_output)
        
        # Check for service discovery
        discovery_complete = any("D2D service discovery complete" in line for line in primary_output)
        
        passed = secondary_connected and discovery_complete
        details = f"Secondary connected: {secondary_connected}, Discovery complete: {discovery_complete}"
        
        self.log_test_result("D2D Connection", passed, details)
        return passed
    
    def test_foot_sensor_data_transfer(self) -> bool:
        """Test 2: Verify foot sensor data transfer from secondary to primary"""
        print(f"\n{Colors.YELLOW}=== Test 2: Foot Sensor Data Transfer ==={Colors.RESET}")
        
        # Trigger foot sensor data on secondary
        print("  Triggering foot sensor data on secondary...")
        self.send_command(self.secondary_serial, "foot_sensor test")
        time.sleep(1)
        
        # Check primary for received data
        primary_output = self.read_output(self.primary_serial, 3.0)
        
        # Look for the specific log messages
        received_data = False
        handler_called = False
        
        for line in primary_output:
            if "=== RECEIVED FOOT SENSOR DATA FROM SECONDARY ===" in line:
                received_data = True
                print(f"  {Colors.GREEN}✓ Primary received foot sensor data{Colors.RESET}")
            if "Secondary foot samples:" in line:
                handler_called = True
                print(f"  {Colors.GREEN}✓ Data handler processed the data{Colors.RESET}")
                print(f"    {line}")
        
        passed = received_data and handler_called
        self.log_test_result("Foot Sensor Data Transfer", passed, 
                           f"Received: {received_data}, Handler called: {handler_called}")
        return passed
    
    def test_bhi360_data_transfer(self) -> bool:
        """Test 3: Verify BHI360 data transfer from secondary to primary"""
        print(f"\n{Colors.YELLOW}=== Test 3: BHI360 Data Transfer ==={Colors.RESET}")
        
        # Trigger BHI360 data on secondary
        print("  Triggering BHI360 data on secondary...")
        self.send_command(self.secondary_serial, "motion_sensor test")
        time.sleep(1)
        
        # Check primary for received data
        primary_output = self.read_output(self.primary_serial, 3.0)
        
        # Look for different BHI360 data types
        received_3d_mapping = False
        received_step_count = False
        received_linear_accel = False
        
        for line in primary_output:
            if "=== RECEIVED BHI360 3D MAPPING FROM SECONDARY ===" in line:
                received_3d_mapping = True
                print(f"  {Colors.GREEN}✓ Received 3D mapping data{Colors.RESET}")
            elif "=== RECEIVED BHI360 STEP COUNT FROM SECONDARY ===" in line:
                received_step_count = True
                print(f"  {Colors.GREEN}✓ Received step count data{Colors.RESET}")
            elif "=== RECEIVED BHI360 LINEAR ACCEL FROM SECONDARY ===" in line:
                received_linear_accel = True
                print(f"  {Colors.GREEN}✓ Received linear acceleration data{Colors.RESET}")
        
        passed = received_3d_mapping or received_step_count or received_linear_accel
        details = f"3D: {received_3d_mapping}, Steps: {received_step_count}, Accel: {received_linear_accel}"
        
        self.log_test_result("BHI360 Data Transfer", passed, details)
        return passed
    
    def test_command_forwarding(self) -> bool:
        """Test 4: Verify command forwarding from primary to secondary"""
        print(f"\n{Colors.YELLOW}=== Test 4: Command Forwarding ==={Colors.RESET}")
        
        # Send set time command on primary
        test_time = int(time.time())
        print(f"  Sending set time command: {test_time}")
        self.send_command(self.primary_serial, f"control set_time {test_time}")
        time.sleep(1)
        
        # Check both devices for command processing
        primary_output = self.read_output(self.primary_serial, 2.0)
        secondary_output = self.read_output(self.secondary_serial, 2.0)
        
        # Look for forwarding on primary
        forwarded = any("D2D TX: Forwarding set time command" in line for line in primary_output)
        
        # Look for reception on secondary
        received = any("D2D RX: Set Time Command" in line for line in secondary_output)
        
        passed = forwarded and received
        details = f"Forwarded: {forwarded}, Received: {received}"
        
        if passed:
            print(f"  {Colors.GREEN}✓ Command successfully forwarded{Colors.RESET}")
        
        self.log_test_result("Command Forwarding", passed, details)
        return passed
    
    def test_status_updates(self) -> bool:
        """Test 5: Verify status updates from secondary to primary"""
        print(f"\n{Colors.YELLOW}=== Test 5: Status Updates ==={Colors.RESET}")
        
        # Trigger status update on secondary
        print("  Triggering status update on secondary...")
        self.send_command(self.secondary_serial, "status set 0x100")  # Set logging active bit
        time.sleep(1)
        
        # Check primary for received status
        primary_output = self.read_output(self.primary_serial, 2.0)
        
        received_status = False
        for line in primary_output:
            if "=== RECEIVED STATUS FROM SECONDARY:" in line and "0x" in line:
                received_status = True
                print(f"  {Colors.GREEN}✓ Received status update: {line}{Colors.RESET}")
                break
        
        self.log_test_result("Status Updates", received_status)
        return received_status
    
    def test_log_availability(self) -> bool:
        """Test 6: Verify log availability notifications"""
        print(f"\n{Colors.YELLOW}=== Test 6: Log Availability Notifications ==={Colors.RESET}")
        
        # Trigger log availability on secondary
        print("  Triggering log availability on secondary...")
        self.send_command(self.secondary_serial, "data notify_log foot 5")
        time.sleep(1)
        
        # Check primary for notification
        primary_output = self.read_output(self.primary_serial, 2.0)
        
        received_notification = False
        for line in primary_output:
            if "=== RECEIVED FOOT LOG AVAILABLE FROM SECONDARY:" in line:
                received_notification = True
                print(f"  {Colors.GREEN}✓ Received log notification: {line}{Colors.RESET}")
                break
        
        self.log_test_result("Log Availability", received_notification)
        return received_notification
    
    def test_connection_recovery(self) -> bool:
        """Test 7: Verify connection recovery after disconnect"""
        print(f"\n{Colors.YELLOW}=== Test 7: Connection Recovery ==={Colors.RESET}")
        
        # Simulate disconnect
        print("  Simulating disconnect...")
        self.send_command(self.primary_serial, "bt disconnect")
        time.sleep(3)
        
        # Wait for reconnection
        print("  Waiting for reconnection...")
        reconnected = self.wait_for_pattern(self.primary_serial, "Secondary device connected", 15.0)
        
        passed = reconnected is not None
        self.log_test_result("Connection Recovery", passed, 
                           "Reconnected" if passed else "Failed to reconnect")
        return passed
    
    def run_all_tests(self):
        """Run all D2D integration tests"""
        print(f"\n{Colors.BOLD}{Colors.CYAN}D2D Integration Test Suite{Colors.RESET}")
        print(f"{Colors.CYAN}{'='*50}{Colors.RESET}")
        
        if not self.connect():
            print(f"{Colors.RED}Failed to connect to devices. Exiting.{Colors.RESET}")
            return
        
        try:
            # Run tests
            tests = [
                self.test_d2d_connection,
                self.test_foot_sensor_data_transfer,
                self.test_bhi360_data_transfer,
                self.test_command_forwarding,
                self.test_status_updates,
                self.test_log_availability,
                # self.test_connection_recovery,  # Optional: can disrupt other tests
            ]
            
            for test in tests:
                try:
                    test()
                except Exception as e:
                    print(f"{Colors.RED}Test failed with exception: {e}{Colors.RESET}")
                    self.log_test_result(test.__name__, False, str(e))
            
            # Print summary
            self.print_summary()
            
        finally:
            self.disconnect()
    
    def print_summary(self):
        """Print test summary"""
        print(f"\n{Colors.BOLD}{Colors.CYAN}Test Summary{Colors.RESET}")
        print(f"{Colors.CYAN}{'='*50}{Colors.RESET}")
        
        total = len(self.test_results)
        passed = sum(1 for r in self.test_results if r['passed'])
        failed = total - passed
        
        print(f"Total Tests: {total}")
        print(f"{Colors.GREEN}Passed: {passed}{Colors.RESET}")
        print(f"{Colors.RED}Failed: {failed}{Colors.RESET}")
        
        if failed > 0:
            print(f"\n{Colors.RED}Failed Tests:{Colors.RESET}")
            for result in self.test_results:
                if not result['passed']:
                    print(f"  - {result['name']}: {result['details']}")
        
        # Return exit code
        sys.exit(0 if failed == 0 else 1)

def main():
    parser = argparse.ArgumentParser(description='D2D Integration Test Runner')
    parser.add_argument('--primary', '-p', required=True, help='Primary device serial port (e.g., /dev/ttyACM0)')
    parser.add_argument('--secondary', '-s', required=True, help='Secondary device serial port (e.g., /dev/ttyACM1)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='Serial baudrate (default: 115200)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Create and run test runner
    runner = D2DTestRunner(args.primary, args.secondary, args.baudrate)
    runner.run_all_tests()

if __name__ == '__main__':
    main()