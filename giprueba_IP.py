import sys
import time

import cv2
"""import gi
#import picamera2
gi.require_version('GLib','2.0')
gi.require_version('GObject','2.0')
gi.require_version('Gst','1.0')

from gi.repository import Gst,GObject,GLib

Gst.init(sys.argv[1:])
print(sys.argv[0:])

print(Gst.version_string())"""
pipeline = (
    "rtspsrc location='rtsp://admin:Hik1245%40@192.168.1.64:554/Streaming/Channels/101 latency=0 ! '+ 'rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! appsink"
)

#cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
#cap = cv2.VideoCapture(r'rtspsrc location="rtsp://admin:Hik1245%40@192.168.1.62:554/Streaming/Channels/101" latency=0 ! rtph265depay ! avdec_h265 ! autovideosink')

pipeline = (
    'rtspsrc location=rtsp://admin:Hik1245%40@192.168.1.64:554/Streaming/Channels/101 latency=0 ! '
    'rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! appsink'
)

pipeline = (
    'rtspsrc location=rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101 latency=0 ! '
    'rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! appsink'
)

pipeline = (
    'rtspsrc location=rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101 latency=0 ! '
    'rtph265depay ! h265parse ! nvv4l2decoder ! videoconvert ! video/x-raw, format=BGR ! appsink'
)

pipeline = (
    'gst-launch-1.0 rtspsrc location=rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101 latency=0 !' 
    'rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink'

)
"""pipeline = (
    'gst-launch-1.0 rtspsrc location=rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101 latency=0 !' 
    'rtph265depay ! h265parse ! nvv4l2decoder ! videoconvert ! appsink'
)"""
pipeline = (
    "gst-launch-1.0 rtspsrc location=rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101 latency=0 !"
    "rtph265depay ! h265parse ! nvv4l2decoder ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "appsink"
)
pipeline = (
    "gst-launch-1.0 rtspsrc location=rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101 latency=0 ! "
    "rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate ! "
    "video/x-raw, format=BGRx, framerate=10/1 ! appsink"
)

location="rtsp://admin:admin2025@192.168.18.227:554/cam/realmonitor?channel=1&subtype=0"
pipeline = (
    f"rtspsrc location=rtsp://admin:admin2025@192.168.18.227:554/cam/realmonitor?channel=1&subtype=0 latency=0 ! "
    "rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate ! "
    "video/x-raw, format=BGRx, framerate=3/1 ! appsink"
    
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
#cap.set(cv2.CAP_PROP_FPS, 30)   

if not cap.isOpened():
    print("Error: No se pudo abrir el stream RTSP.")
    exit()
    
while True:
    try:
        ret, frame = cap.read()
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        frame = cv2.resize(frame, (640, 480))
        
    except:
        print("ERROR")
        break
    
    if not ret:
        print("Error: No se pudo leer el frame del stream.")
        break
    
    # Mostrar el frame
    #frame = cv2.resize(frame,None,fx = 0.8,fy=0.8,interpolation=cv2.INTER_CUBIC) 
    #time.sleep(2)
    #RTSP
    cv2.imshow("Camara 2K", frame)
    
    # Salir al presionar 'q'
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()

"""
rtspsrc location='rtsp://admin:Hik1245%40@192.168.1.66:554/Streaming/Channels/101' ! rtph265depay ! h265parse ! omxh264dec ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! fakesink


FUNCIONAL:
gst-launch-1.0 rtspsrc location="rtsp://admin:Hik1245%40@192.168.1.61:554/Streaming/Channels/101" latency=0 ! rtph265depay ! avdec_h265 ! autovideosink
gst-launch-1.0 rtspsrc location="rtsp://admin:Hik1245%40@192.168.1.62:554/Streaming/Channels/101" latency=0 ! rtph265depay ! avdec_h265 ! autovideosink

gst-launch-1.0 rtspsrc location="rtsp://admin:Hik1245%40@192.168.1.63:554/Streaming/Channels/101" latency=0 ! rtph265depay ! avdec_h265 ! autovideosink
gst-launch-1.0 rtspsrc location="rtsp://admin:Hik1245%40@192.168.1.64:554/Streaming/Channels/101" latency=0 ! rtph265depay ! avdec_h265 ! autovideosink

gst-launch-1.0 rtspsrc location="rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101" latency=0 ! rtph265depay ! avdec_h265 ! autovideosink
gst-launch-1.0 rtspsrc location="rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101" latency=0 ! rtph265depay ! v4l2slh265dec ! autovideosink


H264:
gst-launch-1.0 rtspsrc location="rtsp://admin:Hik1245%40@192.168.1.64:554/Streaming/Channels/101" latency=0 ! rtph264depay ! avdec_h264 ! autovideosink

CONDOMINIO H265:
gst-launch-1.0 rtspsrc location="rtsp://admin:admin2025@192.168.18.227:554/cam/realmonitor?channel=1&subtype=0?buffer_size=102400000" latency=0 ! rtph265depay ! avdec_h265 ! autovideosink
CONDOMINIO H264:
gst-launch-1.0 rtspsrc location="rtsp://admin:admin2025@192.168.18.227:554/cam/realmonitor?channel=1&subtype=0?buffer_size=102400000" latency=0 ! rtph264depay ! avdec_h264 ! autovideosink

sudo apt install -y     build-essential cmake git pkg-config     libjpeg-dev libpng-dev libtiff-dev     libavcodec-dev libavformat-dev libswscale-dev     libv4l-dev libxvidcore-dev libx264-dev     libgtk-3-dev libatlas-base-dev gfortran     python3-dev python3-venv     libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt install -y   cmake build-essential libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev   libgtk-3-dev libjpeg-dev libpng-dev libtiff-dev libavcodec-dev   libavformat-dev libswscale-dev libv4l-dev
pip install --upgrade pip setuptools
pip install wheel cython

cd ~/Templates/REPOS/GSTREAM
OPENCV_VER="48"
git clone --branch ${OPENCV_VER} --depth 1 --recurse-submodules --shallow-submodules https://github.com/opencv/opencv-python.git opencv-python-${OPENCV_VER}
/////git clone --depth 1 --branch 74 --recurse-submodules https://github.com/opencv/opencv-python.git opencv-python-74
cd opencv-python-48

# Asegura que numpy 2 esté activado en el entorno
pip install "numpy<2"

# Compila el wheel con soporte para GStreamer
export CMAKE_ARGS="-DWITH_GSTREAMER=ON"
export ENABLE_HEADLESS=0

python3 -m pip wheel . --verbose

pip uninstall opencv-python opencv-python-headless opencv-contrib-python -y

pip install opencv_python*.whl
python -c "import cv2; print(cv2.getBuildInformation())"

"""
#gst-launch-1.0 rtspsrc location=rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101 latency=0 ! rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! nvegltransform ! nveglglessink
#gst-launch-1.0 rtspsrc location="rtsp://admin:admin2025@192.168.18.227:554/cam/realmonitor?channel=1&subtype=0" latency=0 ! rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! nvegltransform ! nveglglessink