import cv2

# Check build info for CUDA
build_info = cv2.getBuildInformation()
if "CUDA" in build_info:
    print("OpenCV built with CUDA support.")
    # Check detected devices
    try:
        count = cv2.cuda.getCudaEnabledDeviceCount()
        print(f"CUDA Devices detected: {count}")
        if count > 0:
            cv2.cuda.printCudaDeviceInfo(0) # Print info about the first device
    except Exception as e:
        print(f"Error accessing CUDA device: {e}")
else:
    print("OpenCV *NOT* built with CUDA support.")

print(f"OpenCV version: {cv2.__version__}")
exit() # Exit python