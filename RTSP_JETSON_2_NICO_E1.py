import cv2
import threading
import time

import queue
import numpy as np
from paddleocr import PaddleOCR
from datetime import datetime
from collections import Counter


#import sys
import socket
import atexit

# Variable global de control
exit_event = threading.Event()

_window_name = "CAPTURA"

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



# rtsp_url2 = (
#     #'rtspsrc location=rtsp://admin:admin2025@192.168.18.227:554/cam/realmonitor?channel=1&subtype=0?buffer_size=0 latency=100 ! '
#     'rtspsrc location=rtsp://admin:admin2025@192.168.18.227:554/cam/realmonitor?channel=1&subtype=0 latency=100 ! '
#     'rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate ! '
#     'video/x-raw, format=BGRx, framerate=4/1 ! appsink'
# )

rtsp_url2 = (
    #'rtspsrc location=rtsp://admin:admin2025@192.168.18.227:554/cam/realmonitor?channel=1&subtype=0?buffer_size=0 latency=100 ! '
    'rtspsrc location=rtsp://admin:admin2025@192.168.18.227:554/cam/realmonitor?channel=1&subtype=0 ! '
    'rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate ! '
    'video/x-raw, format=BGRx ! appsink'
)

class RTSP_movement:
    def __init__(self, rtsp_url, coladatos, direccion):
        self.rtsp_url = rtsp_url
        self.cola = coladatos
        self.ultima_placa = ""
        self.ultimo_tiempo = ""
        self.plaquitas = queue.Queue()
        self.colaimages = queue.Queue(maxsize=10)
        self.cola_imagenes_a_detectar = queue.Queue(maxsize=5)#15
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
        points_salida = [np.array([(604, 265), (866, 269), (835, 339), (677, 362)], dtype=np.int32)]
        print("CORRIENDO")
        
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                if self.direccion == 3:
                    cv2.fillPoly(frame, [np.array(points_salida, dtype=np.int32)], (0, 0, 0))
                try:
                    #self.colaimages.put(frame,block=False)
                    if self.colaimages.full():
                        self.colaimages.get()  # Elimina el más antiguo
                    self.colaimages.put(frame,block=False)
                    
                except:
                    print("¡La cola colaimages está llena!")
            else:
                exit_event.set()
                self.stop()
                break
        
    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None
        
    def detect_movement(self):
        
        if self.colaimages.empty():    
            return None
        
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
        
        return frame_show


    def process_video(self, ocr):
        # def process_video(self):
        if not self.cola_imagenes_a_detectar.empty():
            # print("PROCESANDO...")
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
                                    acc > 0.80 and (len(text) >= 6 and len(
                                        text) <= 7)  # and ("-" in text)
                                ):  # len(text)>6 and len(text)<8 and acc>0.96 :
                                    # print("PLACA:", text, "Precision:", acc)
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
                    #print(f"String más repetido en 5s: '{mas_comun}' con {cantidad} repeticiones")
                    mas_comun = mas_comun.upper()
                    
                    if mas_comun not in self.texto_actual and mas_comun[-3:].isdigit():
                        self.texto_actual = mas_comun
                        self.cola.put(f"#P:{mas_comun},D:{self.direccion}")
                        self.plaquitas.put(mas_comun)

            self.tiempo_inicio = time.time()

    def stop(self):
        self.cola.put(f"#C1:0")
        self.running = False
        time.sleep(0.5)
        self.cap.release()
class ClienteTCP_Sender:
    def __init__(self,cola, host='192.168.18.162', port=65432):
        self.host = host
        self.port = port
        self.mensajes = cola
        self.running = False
        self.hilo = None

    def iniciar(self):
        self.running = True
        self.hilo = threading.Thread(target=self._enviar_mensajes, daemon=True)
        self.hilo.start()

    def detener(self):
        self.running = False
        if self.hilo:
            self.hilo.join()
        print("Cliente detenido.")

    def _enviar_mensajes(self):
        while self.running == True:
            try:
                if not self.mensajes.empty():
                    mensaje = self.mensajes.get_nowait()
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.settimeout(3)
                        s.connect((self.host, self.port))
                        print(f"Conectado a {self.host}:{self.port}")
                        try:
                            s.sendall(mensaje.encode())
                            print("Enviado:", mensaje)
                        except Exception as e:
                            print("Error envio:",e)
                time.sleep(0.02)
            except Exception as e:
                print("Error en el cliente:", e)
            
class Scheduler:
    def __init__(self, interval_hours):
        self.interval_seconds = interval_hours * 3600
        self.next_run = time.time() + self.interval_seconds
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run_loop)
        self.thread.daemon = True

    def task(self):
        print("Tarea ejecutada a las", time.ctime())

    def _run_loop(self):
        while not self.stop_event.is_set():
            now = time.time()
            if now >= self.next_run:
                self.task()
                self.next_run = now + self.interval_seconds
            # Espera hasta 60 segundos, pero puede ser interrumpido antes
            self.stop_event.wait(timeout=60)

    def start(self):
        self.stop_event.clear()
        if not self.thread.is_alive():
            self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join()

                
        
class Baliza:
    def __init__(self, cola, intervalo=295):
        self.intervalo = intervalo  # segundos
        self.cola = cola
        self.running = False
        self.thread = None

    def iniciar(self):
        self.running = True
        self.thread = threading.Thread(target=self._ejecutar, daemon=True)
        self.thread.start()

    def detener(self):
        self.running = False
        if self.thread:
            self.thread.join()
        print("Tarea detenida.")

    def _ejecutar(self):
        while self.running:
            print("[Tarea] Ejecutando tarea periódica...")
            # Aquí va tu lógica:
            self.mi_tarea()

            for _ in range(self.intervalo):
                if not self.running:
                    break
                time.sleep(1)

    def mi_tarea(self):
        # 👉 Aquí pones lo que quieras ejecutar cada 5 minutos
        self.cola.put("#C1:1")

print("INICIO ocr 1...........")
ocr = PaddleOCR(use_angle_cls=True, lang="en", show_log=False)

cola_datos = queue.Queue()

stream = RTSP_movement(rtsp_url2, cola_datos, __ENTRADA__)#__ENTRADA__)
cliente = ClienteTCP_Sender(cola_datos)
cliente.iniciar()
baliza = Baliza(cola_datos,intervalo=120)  # 5 minutos
baliza.iniciar()
scheduler = Scheduler(interval_hours=12)
scheduler.start()

def ocr_callbacks1():
    print("INICIO ocr")
    
    while True:
        time.sleep(0.001)
        stream.process_video(ocr)
        #stream2.process_video(ocr)
    print("FIN ocr")
    

threading.Thread(target=ocr_callbacks1, daemon=True).start()


cv2.namedWindow(_window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(_window_name, cv2.WND_PROP_FULLSCREEN, 1)

frame,frame_show = stream.detect_movement()



while True:
    
    if not stream.get_running() or exit_event.is_set() or not cliente.running:
        print("ERROR GENERAL")
        break
    
    frame_show = stream.detect_movement()

    if frame_show is not None:
        #combined = cv2.hconcat([frame_show, frame2_show])
        #cv2.imshow(_window_name, combined)
        frame_show = cv2.resize(frame_show, None, fx=0.35, fy=0.35, interpolation=cv2.INTER_AREA)
        cv2.imshow(_window_name, frame_show)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        
        break


def cleanup():
    stream.stop()
    baliza.detener()
    cliente.detener()
    scheduler.stop()
    cv2.destroyAllWindows()

atexit.register(cleanup)



