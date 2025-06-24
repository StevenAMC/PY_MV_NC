import cv2
import threading
import time
import serial
import queue
import numpy as np
from paddleocr import PaddleOCR
from datetime import datetime
from collections import Counter
from pynput import keyboard
import serial.tools.list_ports
import sys


# Variable global de control
exit_event = threading.Event()

_window_name = "CAPTURA"
__UART_JETSON__ = '/dev/ttyTHS1'
__SALIDA__ = 0
__ENTRADA__ = 1
__UMBRA_TAM__ = 1000
__SCALA__ = 0.15
_tiempo_ventana = 0
IP_camera1 = "192.168.18.225"
IP_camera2 = "192.168.18.226"
IP_camera3 = "192.168.18.227"
IP_camera4 = "192.168.18.228"   
IP_camera_l = "192.168.18.180"
#rtsp_url1 = f"rtsp://admin:admin2025@{IP_camera_l}:554/cam/realmonitor?channel=1&subtype=0?buffer_size=102400000"
#rtsp_url2 = f"rtsp://admin:admin2025@{IP_camera4}:554/cam/realmonitor?channel=1&subtype=0?buffer_size=102400000"
#rtsp_url1 = f"rtsp://admin:admin2025@{IP_camera_l}:554/cam/realmonitor?channel=1&subtype=0?buffer_size=102400000"
#rtsp_url2 = f"rtsp://admin:admin2025@{IP_camera_l}:554/cam/realmonitor?channel=1&subtype=0?buffer_size=102400000"
"""rtsp_url1 = (
    f"rtspsrc rtsp://admin:admin2025@{IP_camera_l}:554/cam/realmonitor?channel=1&subtype=0?buffer_size=102400000 latency=0 ! "
    "rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate ! "
    "video/x-raw, format=BGRx, framerate=3/1 ! appsink"
)"""
"""rtsp_url1 = (
    "rtspsrc location=rtsp://admin:Admin2025@192.168.18.180:554/Streaming/Channels/101 latency=0 ! "
    "rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate ! "
    "video/x-raw, format=BGRx, framerate=3/1 ! appsink"
)"""
rtsp_url1 = (
    f"rtspsrc location=rtsp://admin:admin2025@{IP_camera1}:554/cam/realmonitor?channel=1&subtype=0?buffer_size=102400000 latency=0 ! "
    "rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate ! "
    "video/x-raw, format=BGRx, framerate=3/1 ! appsink"
)
rtsp_url2 = (
    f"rtspsrc location=rtsp://admin:admin2025@{IP_camera2}:554/cam/realmonitor?channel=1&subtype=0?buffer_size=102400000 latency=0 ! "
    "rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate ! "
    "video/x-raw, format=BGRx, framerate=3/1 ! appsink"
)

class RTSP_movement:
    def __init__(self, rtsp_url, coladatos, direccion):
        self.rtsp_url = rtsp_url
        self.cola = coladatos
        self.ultima_placa = ""
        self.ultimo_tiempo = ""
        self.plaquitas = queue.Queue()
        self.colaimages = queue.Queue(maxsize=10)
        self.cola_imagenes_a_detectar = queue.Queue(maxsize=1)#15
        self.aLlenar = True
        
        self.texto_actual = ""
        self.direccion = direccion
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_GSTREAMER)
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.strings_recibidos = []
        self.tiempo_inicio = time.time()
        self.start_time = 0
        #self.fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
        #self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(30,20))
        
        self.repeticiones_de_placas = 0
        
        self.hilito = threading.Thread(target=self.update, daemon=True)
        self.hilito.start()
        #threading.Thread(target=self.detect_movement, daemon=True).start()
        
    def get_running(self):
        return self.running

    def update(self):
        print("CORRIENDO")
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                """frame = cv2.resize(
                    frame, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC
                )"""
                if self.direccion == __ENTRADA__:
                    alto, ancho, canales = frame.shape
                    frame = frame[300:alto, 400:ancho-200]
                    alto, ancho, canales = frame.shape
                    points = [
                        [[0,0],[0, alto/4], [ancho/1.1, alto/2.8], [ancho, alto/3],[ancho,0]],
                        # [[160, 30], [50, 30], [80, 130], [100, 130]],
                    ]
                    for poly in points:
                        cv2.fillPoly(frame, np.array([poly], dtype=np.int32), (0, 0, 0))
                    #       RECORTE:
                    frame = frame[::, 0:ancho-500]
                    
                    
                if self.direccion == __SALIDA__:
                    alto, ancho, canales = frame.shape
                    frame = frame[300:alto, 700:ancho-100]
                    alto, ancho, canales = frame.shape
                    points = [
                        [[ancho/6, 0], [ancho, 0], [ancho, alto/1.8]],
                        # [[160, 30], [50, 30], [80, 130], [100, 130]],
                    ]
                    for poly in points:
                        cv2.fillPoly(frame, np.array([poly], dtype=np.int32), (0, 0, 0))
                    
                else:
                    pass
                
                """with self.lock:
                    self.frame = frame"""
                try:
                    #self.colaimages.put(frame,block=False)
                    if self.colaimages.full():
                        self.colaimages.get()  # Elimina el más antiguo
                    self.colaimages.put(frame,block=False)
                    
                except:
                    print("¡La cola colaimages está llena!")       
                    
            else:
                
                self.stop()
                break
        
    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None
        
    def detect_movement(self):
        
        if self.colaimages.empty():    
            return None,None
        
        frame_org = self.colaimages.get()
        frame_show = frame_org.copy()

        try:
            #if self.aLlenar == True:#self.cola_imagenes_a_detectar.full():
            if self.cola_imagenes_a_detectar.full():
                self.cola_imagenes_a_detectar.get()  # Elimina el más antiguo
            self.cola_imagenes_a_detectar.put(frame_org,block=False)
            #self.cola_imagenes_a_detectar.put(frame_org,block=False)       
        except: #queue.Full:
            #self.aLlenar = False
            print("¡La cola cola_imagenes_a_detectar está llena!")
                
        if not self.plaquitas.empty():
            self.ultima_placa = self.plaquitas.get()
            self.ultimo_tiempo = datetime.now().strftime("%H:%M:%S")
            cv2.putText(frame_show,"Placa Vehicular:",(10, 80),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 255, 0),2,)
        
        cv2.putText(frame_show, self.ultimo_tiempo+" > "+self.ultima_placa,(10, 150),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0, 255, 0),3,)    
        
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame_show, current_time, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        #cv2.putText(frame, texto_estado , (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color,2)
        
        return frame_org,frame_show


    def process_video(self, ocr):
    #def process_video(self):
        if not self.cola_imagenes_a_detectar.empty():
            #print("PROCESANDO...")
            frame = self.cola_imagenes_a_detectar.get()
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = ocr.ocr(rgb_frame, cls=False)

            detected_text = ""

            if results:
                for line in results:
                    if line:
                        for word_info in line:
                            # print(word_info)
                            if word_info and len(word_info) > 1:
                                text, acc, box = (
                                    word_info[1][0],
                                    word_info[1][1],
                                    word_info[0],
                                )
                                detected_text += text + ""
                                if (
                                    acc > 0.80 and (len(text) >= 6 and len(text) <= 7)# and ("-" in text)
                                ):  # len(text)>6 and len(text)<8 and acc>0.96 :
                                    #print("PLACA:", text, "Precision:", acc)
                                    self.strings_recibidos.append(text)
                                    
        if time.time() - self.tiempo_inicio < _tiempo_ventana:
            pass
        else:
            if self.strings_recibidos:  # and len(self.strings_recibidos)>2:
                print(self.strings_recibidos, ">", len(self.strings_recibidos))
                contador = Counter(self.strings_recibidos)
                mas_comun, cantidad = contador.most_common(
                    1)[0]  # (string, cantidad)
                self.strings_recibidos.clear()
                
                if mas_comun:
                    print(f"String más repetido en 5s: '{mas_comun}' con {cantidad} repeticiones")
                    mas_comun = mas_comun.upper()
                    self.plaquitas.put(mas_comun)
                    print("PLACA mas_comun",mas_comun)
                    
                    #*********************
                    #self.texto_actual = mas_comun
                    
                    self.cola.put(f"#P:{mas_comun},D:{self.direccion}")
                    
                    """if mas_comun not in self.texto_actual:
                        self.texto_actual = mas_comun
                        print(mas_comun)
                        self.cola.put(f"#P:{mas_comun},D:{self.direccion}")
                    elif mas_comun in self.texto_actual:
                        self.repeticiones_de_placas +=0"""
                        
                    #print(f"String más repetido en 5s: '{mas_comun}' con {cantidad} repeticiones")
                    """cv2.polylines(
                        frame,
                        [np.int32(box)],
                        isClosed=True,
                        color=(0, 255, 0),
                        thickness=2,
                    )
                    cv2.putText(
                        frame,
                        mas_comun,
                        (int(box[0][0]), int(box[0][1]) - 10),
                        cv2.FONT_HERSHEY_COMPLEX,
                        0.5,
                        (255, 0, 0),
                        2,
                    )"""
                    
                    # cv2.putText(frame,detected_text.strip(),(10,150),cv2.FONT_HERSHEY_SIMPLEX,2.5,(0,255,0),8)
            #self.tiempo_inicio = time.time()
            
            #return frame
            self.tiempo_inicio = time.time()

    def stop(self):
        self.running = False
        time.sleep(0.5)
        self.cap.release()


class scannerUSB:
    def __init__(self, cola, port='/dev/ttyS0', baudrate=9600):
        self.window_name = 'Detector_Fatiga'
        self.cola = cola
        self.capturing = False
        self.data = ""
        self.data_validada = ""
        self.listener_running = True

        # Inicializa el listener de teclado
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Hilo para la comunicación serial
        self.serial_thread = threading.Thread(
            target=self.serial_communication, daemon=True)
        self.serial_thread.start()

    # *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # * LECTURA DE TECLADO

    def on_press(self, key):
        try:
            if key.char:
                self.data += key.char
                self.capturing = True
        except AttributeError:
            if key == keyboard.Key.enter and self.capturing:
                self.data_validada = self.data
                self.data = ""
                self.capturing = False

    def on_release(self, key):
        if key == keyboard.Key.esc:
            return False

    def close_program(self):
        self.listener_running = False
        time.sleep(0.8)
        # self.ser.close()

    # *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # * COMUNICACIÓN SERIAL

    def serial_communication(self):
        while True:
            try:
                if len(self.data_validada) > 0 and not self.capturing:
                    envio_QBAR = f"#Q:{self.data_validada},D:1"  # .encode()
                    self.data_validada = ""
                    self.cola.put(envio_QBAR)
                    print(envio_QBAR)
                    # self.ser.write(envio_QBAR)
                    # self.ser.flush()
                if not self.listener_running:
                    raise Exception("Cerrando el programa")
            except Exception as e:
                print(e)
                self.close_program()
                break
            time.sleep(0.1)

class SerialScanner:
    def __init__(self, cola, port, baudrate=9600):
        self.cola = cola
        self.port = port
        self.ser = serial.Serial(self.port, baudrate, timeout=0)
        self.running = True
        self.ser.read_all()
        self.serial_thread = threading.Thread(
            target=self.serial_scanner, daemon=True)
        self.serial_thread.start()
        

    def serial_scanner(self):
        terminadores = [b'\r\n', b'\t']
        buffer = b''
        try:
            while self.running:
                if self.ser.in_waiting > 0:
                    buffer += self.ser.read(self.ser.in_waiting)
                    for t in terminadores:
                        if t in buffer:
                            mensaje, _, buffer = buffer.partition(t)
                            try:
                                # Filtrar caracteres inválidos antes de decodificar
                                mensaje = bytes([b for b in mensaje if 32 <= b <= 126])
                                mensaje_str = mensaje.decode(errors='ignore').strip()
                                if t == b"\r\n":
                                    print(f"Mensaje recibido <CR><LN>: {mensaje_str}")
                                    self.cola.put(f"#Q:{mensaje_str},D:1")
                                else:
                                    print(f"Mensaje recibido <TAB>: {mensaje_str}")
                                    self.cola.put(f"#Q:{mensaje_str},D:0")
                            except Exception as e:
                                print(e, "ERROR en decode SerialScanner")
                else:
                    time.sleep(0.05)
        except Exception as e:#(serial.SerialException, OSError) as e:
            print(f"[ERROR] Puerto {self.port} desconectado: {e}")
            self.running = False
            exit_event.set()  # Señalamos que se debe salir
        finally:
            try:
                self.ser.close()
            except:
                pass

    def stop(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        self.serial_thread.join()
    
    
class SerialSender:
    def __init__(self, cola, port='/dev/ttyS0', baudrate=115200, intervalo_ms=400):
        self.cola = cola
        self.intervalo = intervalo_ms / 1000.0  # Convertir a segundos
        self.running = True

        # Inicializar puerto serial
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        # Iniciar hilo
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                if not self.cola.empty():
                    dato = self.cola.get()
                    print(f"[SerialSender] Enviando: {dato}")
                    self.ser.write(f"{dato}\r\n".encode())
                    self.ser.flush()
            except Exception as e:
                print(f"[SerialSender] Error: {e}")
            time.sleep(self.intervalo)

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()

print("INICIO ocr 1...........")
ocr = PaddleOCR(use_angle_cls=True, lang="en", show_log=False)


# Initialize threaded RTSP stream
cola_datos = queue.Queue()

stream = RTSP_movement(rtsp_url1, cola_datos,__ENTRADA__)
stream2 = RTSP_movement(rtsp_url2, cola_datos, __SALIDA__)

# '/dev/ttyUSB0'
# '/dev/ttyAMA0'
serial_app = SerialSender(cola=cola_datos, port=__UART_JETSON__)
app_scanner = scannerUSB(cola_datos)


conexiones = []
puertos = serial.tools.list_ports.comports()
"""puertos_filtrados = [p.device for p in puertos if 'ttyUSB' in p.device]

for puerto in puertos_filtrados:
    print(f"Abriendo puerto {puerto}")
    try:
        scan_ser = SerialScanner(cola=cola_datos,port=puerto,baudrate=9600)
        print(f"Puerto {puerto} abierto correctamente a 9600 baudios.")
        conexiones.append(scan_ser)
        
    except serial.SerialException as e:
        print(f"No se pudo abrir {puerto}: {e}")
"""


def ocr_callbacks1():
    print("INICIO ocr")
    
    while True:
        #time.sleep(0.001)
        stream.process_video(ocr)
        stream2.process_video(ocr)
    print("FIN ocr")
    
"""def ocr_callbacks2():
    print("INICIO ocr")
    ocr = PaddleOCR(use_angle_cls=True, lang="en", show_log=False)

    while True:
        time.sleep(0.01)
        #stream.process_video(ocr)
        stream2.process_video(ocr)
    print("FIN ocr")"""
threading.Thread(target=ocr_callbacks1, daemon=True).start()
#threading.Thread(target=ocr_callbacks2, daemon=True).start()


cv2.namedWindow(_window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(_window_name, cv2.WND_PROP_FULLSCREEN, 1)

frame,frame_show = stream.detect_movement()
frame2, frame2_show = stream2.detect_movement()

frame_actual = None
frame2_actual = None

#attemps = 0

"""while frame is None or frame2 is None:
    attemps +=1
    if attemps == 60:
        break
    time.sleep(0.2)
"""

while True:
    
    #time.sleep(0.5)
    #time.sleep(0.01)
    
    #frame = stream.get_frame()
    #frame2 = stream2.get_frame()
    
    
    if not stream.get_running() or exit_event.is_set() or not stream2.get_running() :
        print("ERROR")
        for c in conexiones:
            try:
                c.stop()
            except:
                pass
        break
    frame,frame_show = stream.detect_movement()
    frame2, frame2_show = stream2.detect_movement()

    if frame_show is None:
        frame_actual = frame_actual
    else:
        frame_actual = frame_show
        
    if frame2_show is None:
        frame2_actual = frame2_actual
    else:
        frame2_actual = frame2_show
    
        
    #if frame is not None and frame2 is not None:
    if frame_show is not None:
        #combined = cv2.hconcat([frame_show, frame2_show])
        #cv2.imshow(_window_name, combined)
        cv2.imshow(_window_name, frame_show)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break



#ventanahilo = threading.Thread(target=opencv_ventana, daemon=True)
#ventanahilo.start()
#threading.Thread(target=ocr_callbacks, daemon=True).start()

stream.stop()    
#stream2.stop()
cv2.destroyAllWindows()


