import cv2
import numpy as np
import math
import serial
import time
from ultralytics import YOLO

# ---------------------------------------------------------
# 1. KONFIGURACJA GŁÓWNA
# ---------------------------------------------------------
PORT_COM = 'COM5'
BAUD_RATE = 9600
MODEL_YOLO = 'best.pt'

# ---------------------------------------------------------
# 2. INICJALIZACJA SYSTEMÓW
# ---------------------------------------------------------
# Połączenie z Arduino
try:
    arduino = serial.Serial(PORT_COM, BAUD_RATE, timeout=1)
    time.sleep(2) # Czas na reset płytki
    print(f"[SYSTEM] Polaczono z Arduino na porcie {PORT_COM}")
except Exception as e:
    print(f"[BLAD] Nie mozna polaczyc z Arduino: {e}")
    exit()

# Załadowanie Mózgu Wizyjnego (Homografii)
try:
    macierz_H = np.load("homografia.npy")
    print("[SYSTEM] Macierz homografii zaladowana.")
except FileNotFoundError:
    print("[BLAD] Brak pliku 'homografia.npy'!")
    exit()

# Załadowanie Sztucznej Inteligencji (YOLO)
model = YOLO(MODEL_YOLO)
cap = cv2.VideoCapture(1)

print("[SYSTEM] Wszystkie moduly gotowe. Rozpoczynam nasluch...")

# Zmienne blokujące "spamowanie" Arduino
ostatni_kat_baza = -1
ostatni_kat_lokiec = -1
czas_ostatniej_wysylki = 0

poprzedni_x_mm = -999.0
poprzedni_y_mm = -999.0
czas_rozpoczecia_postoju = 0
obiekt_ustabilizowany = False
TOLERANCJA_RUCHU_MM = 3.0
CZAS_OCZEKIWANIA = 1.0

# ---------------------------------------------------------
# 3. KINEMATYKA (Twój dostrojony algorytm)
# ---------------------------------------------------------
def policz_kinematyke(x_mm, y_mm):
    L1 = 64.0 
    L2 = 64.0 
    
    r = math.sqrt(x_mm**2 + y_mm**2)
    max_zasieg = L1 + L2
    if r >= max_zasieg: r = max_zasieg - 0.1 

    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    theta2_rad = math.acos(cos_theta2)
    kat_lokiec_stopnie = math.degrees(theta2_rad)

    kat_do_celu = math.atan2(-y_mm, x_mm)
    kat_odchylenia = math.atan2(L2 * math.sin(theta2_rad), L1 + L2 * math.cos(theta2_rad))
    
    theta1_rad = kat_do_celu + kat_odchylenia
    kat_baza_stopnie = math.degrees(theta1_rad)

    OFFSET_BAZA = 19
    OFFSET_LOKIEC = 9
    
    finalny_baza = int(kat_baza_stopnie + OFFSET_BAZA)
    finalny_lokiec = int(160 - kat_lokiec_stopnie)
    
    finalny_baza = max(10, min(150, finalny_baza))
    finalny_lokiec = max(10, min(170, finalny_lokiec))
    
    return finalny_baza, finalny_lokiec

# ---------------------------------------------------------
# 4. GŁÓWNA PĘTLA ROBOCZA
# ---------------------------------------------------------
while True:
    ret, frame = cap.read()
    if not ret: break
    
    results = model(frame, conf=0.75)

    for result in results:
        if len(result.boxes) > 0:
            box = result.boxes[0]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            id_klasy = int(box.cls[0].item()) 
            
            obj_cx = (x1 + x2) // 2
            obj_cy = (y1 + y2) // 2

            # Transformacja
            punkt_piksele = np.array([[[obj_cx, obj_cy]]], dtype="float32")
            punkt_mm = cv2.perspectiveTransform(punkt_piksele, macierz_H)
            
            robot_x_mm = punkt_mm[0][0][0]
            robot_y_mm = punkt_mm[0][0][1]
            
            if robot_x_mm < 150: 
                robot_y_mm += 15.0
                robot_x_mm += 15.0 

            dystans_ruchu = math.sqrt((robot_x_mm - poprzedni_x_mm)**2 + (robot_y_mm - poprzedni_y_mm)**2)

            if dystans_ruchu > TOLERANCJA_RUCHU_MM:
                poprzedni_x_mm = robot_x_mm
                poprzedni_y_mm = robot_y_mm
                czas_rozpoczecia_postoju = time.time()
                obiekt_ustabilizowany = False
                cv2.putText(frame, "RUCH - Czekam...", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:
                czas_postoju = time.time() - czas_rozpoczecia_postoju
                if czas_postoju >= CZAS_OCZEKIWANIA:
                    obiekt_ustabilizowany = True
                    nazwa = "SRUBA" if id_klasy == 0 else "WKRET"
                    cv2.putText(frame, f"STABILNE: {nazwa}", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, f"Stabilizacja: {czas_postoju:.1f}s", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

            if obiekt_ustabilizowany:
                kat_baza, kat_lokiec = policz_kinematyke(robot_x_mm, robot_y_mm)

                obecny_czas = time.time()
                roznica_baza = abs(kat_baza - ostatni_kat_baza)
                roznica_lokiec = abs(kat_lokiec - ostatni_kat_lokiec)
                
                if (roznica_baza > 2 or roznica_lokiec > 2) and (obecny_czas - czas_ostatniej_wysylki > 1.5):
                    komenda = f"{kat_baza},{kat_lokiec},{id_klasy}\n"
                    arduino.write(komenda.encode('utf-8'))
                    
                    ostatni_kat_baza = kat_baza
                    ostatni_kat_lokiec = kat_lokiec
                    czas_ostatniej_wysylki = obecny_czas
                    
                    print(f"[RUCH] Baza: {kat_baza}, Lokiec: {kat_lokiec} | Typ: {id_klasy}")

            # Wizualizacja
            kolor = (255, 0, 0) if id_klasy == 0 else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), kolor, 2)
            cv2.circle(frame, (obj_cx, obj_cy), 5, kolor, -1)
            
            tekst_info = f"X:{robot_x_mm:.0f} Y:{robot_y_mm:.0f} | B:{kat_baza if obiekt_ustabilizowany else '-'} L:{kat_lokiec if obiekt_ustabilizowany else '-'}"
            cv2.putText(frame, tekst_info, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, kolor, 2)
            
            break
        else:
            ostatni_kat_baza = -1
            ostatni_kat_lokiec = -1
            poprzedni_x_mm = -999.0
            poprzedni_y_mm = -999.0
            
    cv2.imshow("Wizja SCARA - System Produkcyjny", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("\n[ZAMYKANIE] Zlecono wylaczenie. Parkuje ramie...")
        komenda_parkowania = "P\n"
        arduino.write(komenda_parkowania.encode('utf-8'))
        
        print("Czekam 4 sekundy na zaparkowanie...")
        time.sleep(4) 
        print("Ramie zaparkowane. Zamykam system.")
        break

# Sprzątanie
cap.release()
cv2.destroyAllWindows()
arduino.close()