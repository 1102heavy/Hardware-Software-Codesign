import serial
import time
import csv

# Configuration
SERIAL_PORT = '/dev/serial/by-id/usb-Xilinx_ML_Carrier_Card_XFL12WVSPQXJ-if01-port0' 
# BAUD_RATE = 115200
# TIMEOUT = 1

# --- Configuration ---
# SERIAL_PORT = 'COM3'  # Update to your port
BAUD_RATE = 115200
A_ROWS,A_COLS = 64, 8


def run_validated_test():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
        ser.reset_input_buffer()

        # Define Test Data
        matrix_a = [10, 2, 3, 4, 
                    5, 6, 7, 8]
        vector_b = [1, 1, 1, 1]
        payload = bytearray(matrix_a + vector_b)

        # 1. Calculate Expected Result locally
        expected = []
        for r in range(A_ROWS):
            row_sum = 0
            for c in range(A_COLS):
                row_sum += matrix_a[r * A_COLS + c] * vector_b[c]
            # Account for the 8-bit truncation in C (u8)
            expected.append(row_sum % 256)

        # 2. Send to Zynq
        print(f"Sending payload: {list(payload)}")
        ser.write(payload)

        # 3. Read and Validate
        raw_response = ser.read(A_ROWS)
        if len(raw_response) == A_ROWS:
            actual = [int(b) for b in raw_response]
            
            print("-" * 30)
            print(f"Expected: {expected}")
            print(f"Actual:   {actual}")
            print("-" * 30)

            if actual == expected:
                print("✅ SUCCESS: Results match!")
            else:
                print("❌ ERROR: Mismatch detected.")
        else:
            print(f"Expected: {expected}")
            print(f"❌ ERROR: Timeout. Expected {A_ROWS} bytes, got {len(raw_response)}")

        ser.close()

    except Exception as e:
        print(f"System Error: {e}")



def read_csv(filename):
    data = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            # Convert string values to integers
            data.extend([int(cell) for cell in row if cell.strip()])
    return data

def run_csv_test():
    try:
        # 1. Load Data from CSVs
        matrix_a = read_csv('A.csv')
        vector_b = read_csv('B.csv')
        # print(matrix_a)
        # print(vector_b)
        
        # Validation: Check if sizes match expectations
        if len(matrix_a) != (A_ROWS * A_COLS) or len(vector_b) != A_COLS:
            print(f"Expected A: {(A_ROWS * A_COLS)}")
            print(f"Actual A:   {len(matrix_a)}")
            print(f"Expected B: {(A_COLS)}")
            print(f"Actual B:   {len(vector_b)}")
            print("❌ Error: CSV data dimensions do not match A_ROWS/A_COLS")
            return

        # 2. Local "Golden" Calculation for Validation
        expected = []
        for r in range(A_ROWS):
            row_sum = sum(matrix_a[r*A_COLS + c] * vector_b[c] for c in range(A_COLS))
            expected.append((row_sum)) # Match Zynq's 8-bit truncation

        # 3. Serial Communication
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.reset_input_buffer()

        payload = bytearray(matrix_a + vector_b)
        print(f"Sending {len(payload)} bytes from CSV...")
        ser.write(payload)

        # 4. Read Results
        raw_response = ser.read(A_ROWS)
        if len(raw_response) == A_ROWS:
            actual = [int(b) for b in raw_response]
            print(f"Expected: {expected}")
            print(f"Actual:   {actual}")
            print("✅ Match!" if actual == expected else "❌ Mismatch!")
        else:
            print("❌ Timeout: Device did not return enough bytes.")

        ser.close()

    except FileNotFoundError as e:
        print(f"❌ Error: Could not find file - {e}")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    run_csv_test()
