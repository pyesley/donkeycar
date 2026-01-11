# dtr_polarity_probe.py
import sys, time, serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
ser = serial.Serial(port, 115200, timeout=0)
print(f"Opened {port}. Toggling DTR every 3s. Watch the can.")
try:
    ser.setDTR(True)
    print("DTR=TRUE (asserted) -> Does motor STOP?")
    time.sleep(3)
    ser.setDTR(False)
    print("DTR=FALSE (cleared) -> Does motor RUN?")
    time.sleep(3)
    ser.setDTR(True)
    print("DTR=TRUE (asserted) -> STOP again?")
    time.sleep(3)
    ser.setDTR(False)
    print("DTR=FALSE (cleared) -> RUN again?")
    time.sleep(3)
finally:
    ser.setDTR(True)   # leave it stopped
    ser.close()
