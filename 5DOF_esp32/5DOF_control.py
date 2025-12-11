import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter.filedialog import asksaveasfilename, askopenfilename
import json
import serial
import serial.tools.list_ports
import time
import threading
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import math
import numpy as np

# =====================================================================
# KOMUNIKACJA SERIAL Z ESP-32
# =====================================================================

class RobotSerial:
    """
    Klasa odpowiedzialna za komunikację szeregową z modułem ESP-32.
    Obejmuje otwieranie portu, nasłuchiwanie danych w tle oraz wysyłanie komend sterujących.
    Zaprojektowana tak, aby reszta aplikacji mogła korzystać z aktualnych danych bez blokowania GUI.
    """

    def __init__(self, port=None, baudrate=115200):
        self.port_name = port
        self.baudrate = baudrate

        self.ser = None             # Obiekt Serial (utworzony dopiero w connect()).
        self.running = False        # Flaga kontrolująca pracę wątku odczytu.
        self.reader_thread = None   # Wątek odpowiedzialny za odbiór danych w tle.

        # Aktualne wartości odczytane z kontrolera [X, Y, Z, E, A, S] w stopniach.
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Lock zabezpiecza dane przy odczycie/zapisie z różnych wątków (mutex w c++)
        self.lock = threading.Lock()

        # Miejsce na funkcję, która zaktualizuje liczby i wykresy w oknie aplikacji.
        # Dzięki temu robot może poinformować GUI, że ma nowe dane do wyświetlenia.
        self.gui_callback = None

    def find_esp32_port(self):
        """
        Przeszukaj wszystkie dostępne porty COM, starając się wykryć ESP32.
        """
        # W naszym przypadku to "CP210x USB to UART Bridge (COM9)". 
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "CP210" in p.description: 
                return p.device

        return None # None — jeśli urządzenie nie zostało znalezione

    def connect(self):
        """
        Nawiązuje połączenie z urządzeniem ESP-32 przez port szeregowy.
        Zamyka poprzednie połączenie, wykrywa odpowiedni port, otwiera go,
        inicjalizuje mikrokontroler i uruchamia wątek odczytu.
        Zwraca: (bool, str) - informacja o powodzeniu operacji.
        """

        # Jeśli istnieje wcześniejsze połączenie
        # Zamknij, aby nie blokować portu lub wątku w tle.
        if self.ser and self.ser.is_open:
            self.close()

        # Wybór portu do użycia. Dopiero po udanej inicjalizacji trafia do self.port_name.
        target_port = self.port_name or self.find_esp32_port()
        if target_port is None:
            return False, "Nie znaleziono urządzenia ESP-32"

        try:
            # Otwarcie portu. Timeout zapobiega zawieszeniu odczytu.
            self.ser = serial.Serial(target_port, self.baudrate, timeout=1)

            # ESP32 wykonuje reset po zestawieniu połączenia (sygnał DTR).
            # Krótki odstęp pozwala mu się ponownie uruchomić.
            time.sleep(2)

            # Czyścimy bufor, aby uniknąć pracy na śmieciowych danych po resecie.
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # Start wątku odpowiedzialnego za odbiór ramek z kontrolera
            self.running = True
            self.reader_thread = threading.Thread(
                target=self._continuous_read,
                daemon=True # Zabija wątek przy zamknięciu aplikacji
            )
            self.reader_thread.start()

            self.port_name = target_port # Port uznajemy za działający.
            return True, f"Połączono z {self.port_name}"

        except Exception as e:
            # Sprzątanie w razie błędu – metoda close() zabezpiecza wszystkie zasoby
            self.close()
            return False, f"Błąd połączenia: {e}"

    def _continuous_read(self):
        """
        Wątek odbierający dane z portu szeregowego.
        Odczyt działa blokująco, dzięki czemu w spoczynku nie obciąża procesora.
        Metoda parsuje ramki z danymi kątów i aktualizuje stan robota.
        """

        AXIS_MAP = {
            "X": 0, 
            "Y": 1, 
            "Z": 2, 
            "E": 3, 
            "S": 4,
            "A": 5  # Slave Y
        }

        while self.running and self.ser and self.ser.is_open:
            try:
                # Blokujący odczyt z UART — czeka na pełną linię lub timeout.
                raw_line = self.ser.readline()
                if not raw_line: continue

                # Dekodowanie odebranych danych.
                line = raw_line.decode("utf-8", errors="ignore").strip()
                if not line: continue

                # Obsługa ramek danych (<X...,Y...,Z...,E...,A...>)
                if line.startswith("<") and line.endswith(">"):
                    updates = []
                    items = line[1:-1].split(',')

                    # Parsowanie
                    for item in items:
                        if len(item) >= 2 and item[0] in AXIS_MAP:
                            try:
                                updates.append((AXIS_MAP[item[0]], float(item[1:])))
                            except ValueError: pass

                    # Aktualizacja współdzielonego stanu z blokadą
                    if updates: # Czy mamy nowe dane do zapisania?
                        with self.lock: # Blokada na czas zapisu i kopiowania
                            for idx, val in updates: # Zapisywanie nowych wartości
                                self.current_angles[idx] = val
                            # Zrobiona kopia do wysłania do GUI
                            data_to_send = self.current_angles.copy() if self.gui_callback else None

                        # Callback poza blokadą, żeby nie wieszać wątku, jeżeli GUI byłoby wolne
                        if self.gui_callback and data_to_send:
                            self.gui_callback(data_to_send)
                            

                else:
                    if self.gui_callback: # Jeżeli nie jest ramką, to wypisz log.
                        self.gui_callback(None, log_message=line)

            except (serial.SerialException, OSError):
                # Najczęstszy scenariusz to odłączenie kabla w trakcie pracy.
                self.running = False
                try:
                    self.ser.close()
                except Exception: pass

                if self.gui_callback:
                    self.gui_callback(None, log_message="#CONNECTION_LOST#")
                break

            except Exception as e:
                # Inne błędy odczytu nie powinny zatrzymywać całej aplikacji.
                if self.running and self.gui_callback:
                    self.gui_callback(None, log_message=f"Błąd odczytu: {e}")
                break

    def get_current_angles(self):
        """
        Zwraca aktualne kąty jako listę.
        Tworzona jest kopia, aby zapobiec przypadkowej modyfikacji danych przez inne moduły.
        """
        with self.lock:
            return self.current_angles.copy()

    def send_target_angles(self, th1, th2, th3, th4, th5):
        """
        Wysyła wartości docelowe osi do ESP32.
        Oczekuje wartości w radianach — konwertuje je na stopnie.
        Format wysyłanej komendy: X...,Y...,Z...,E...,A...
        """
        if not self.ser or not self.ser.is_open:
            return False, "Port nie jest otwarty"

        # Konwersja radianów na stopnie i budowa ramki
        radians = [th1, th2, th3, th4, th5]
        degrees = [math.degrees(v) for v in radians]
        cmd = (
            f"X{degrees[0]:.2f},"
            f"Y{degrees[1]:.2f},"
            f"Z{degrees[2]:.2f},"
            f"E{degrees[3]:.2f},"
            f"S{degrees[4]:.2f}\n"
        )

        try: # Wysłanie ramki i wyświetlenie potwierdzenia
            self.ser.write(cmd.encode("utf-8"))
            return True, f"Wysłano: {cmd.strip()}"
        except Exception as e:
            return False, f"Błąd wysyłania: {e}"

    def close(self):
        """
        Zatrzymuje wątek odczytu i zamyka port.
        """
        self.running = False

        # Czekamy na zakończenie wątku, żeby uniknąć wycieków wątków.
        if self.reader_thread:
            self.reader_thread.join(timeout=1.0)

        # Bezpieczne zamknięcie portu.
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except: pass

# =====================================================================
# KINEMATYKA (Model matematyczny robota 5-DOF)
# =====================================================================

class RobotKinematics:
    def __init__(self):
        """
        Inicjalizacja modelu kinematycznego dla dedykowanej konstrukcji robota.
        Wymiary i limity są zdefiniowane bezpośrednio wewnątrz klasy.
        """
        # Stałe parametry [mm]
        self.a1 = 18.4
        self.a2 = 149.0
        self.a3 = 120.3
        self.a4 = 87.8
        self.d1 = 110.8
        self.d5 = 10.0
        self.d6 = 23.0
        
        # Definicja limitów w złączach [rad]
        range_const = math.pi / 2
        self.limits = {
            'th1': (-range_const, range_const), # X
            'th2': (0, range_const * 1.5),      # YA
            'th3': (-range_const, range_const), # Z
            'th4': (-range_const, range_const), # E
            'th5': (-math.pi, math.pi)          # A
        }

        # Prekomputacja stałych
        self.L1 = self.a2
        self.L2 = self.a3
        self.L3 = math.sqrt((self.a4 + self.d6)**2 + self.d5**2)
        
        # Stałe do twierdzenia cosinusów
        self.L1_sq = self.L1**2
        self.L2_sq = self.L2**2
        self.denom = 2 * self.L1 * self.L2
        
        # Kwadraty zasięgów (do szybkiej weryfikacji bez pierwiastkowania)
        self.max_reach_sq = (self.L1 + self.L2)**2
        self.min_reach_sq = (self.L1 - self.L2)**2
        
        # Przesunięcie środka osi przez serwo
        self.geo_phi_offset = math.atan2(self.d5, self.a4 + self.d6)

        # współczynniki do wyznaczania punktu krytycznego
        self.radius = 16.0
        if self.a4 > 0:
            self.crit_axial_factor = 65.2 / self.a4  # Proporcja wzdłuż osi
            self.crit_offset_factor = 29.2 / self.a4 # Proporcja prostopadła do osi
        else:
            self.crit_axial_factor = 0
            self.crit_offset_factor = 0

    def _rot_z(self, t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    def _rot_x(self, a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
    def _trans_z(self, d):
        return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])
    def _trans_x(self, a):
        return np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    def _compute_chain(self, th1, th2, th3, th4, th5=0.0):
        """
        Zwraca listę macierzy transformacji kinematyki prostej
        """
        A1 = self._rot_z(th1)    @ self._trans_z(self.d1) @ self._trans_x(self.a1) @ self._rot_x(np.pi/2)
        A2 = self._rot_z(th2)                             @ self._trans_x(self.a2)
        A3 = self._rot_z(-th3)                            @ self._trans_x(self.a3)
        A4 = self._rot_z(-th4)                            @ self._trans_x(self.a4) @ self._rot_x(-np.pi / 2)
        A5 = self._rot_z(np.pi/2)@ self._trans_z(self.d5)                          @ self._rot_x(np.pi / 2)
        A6 = self._rot_z(-th5)   @ self._trans_z(self.d6)

        T_base = np.eye(4)
        T1 = A1
        T2 = T1 @ A2
        T3 = T2 @ A3
        T4 = T3 @ A4
        T5 = T4 @ A5
        T_E = T5 @ A6

        return [T_base, T1, T2, T3, T4, T5, T_E]

    def get_joint_positions(self, th1, th2, th3, th4, th5=0.0):
        """
        Zwraca tablicę punktów (pozycje węzłów)
        """
        matrices = self._compute_chain(th1, th2, th3, th4, th5)
        return np.array([m[:3, 3] for m in matrices])

    def get_tcp_matrix(self, th1, th2, th3, th4, th5=0.0):
        """
        Zwraca wyłącznie macierz transformacji efektora końcowego (TCP)
        """
        chain = self._compute_chain(th1, th2, th3, th4, th5)
        return chain[-1]
    
    def get_jacobian(self, th1, th2, th3, th4, th5):
        """
        Oblicza Jakobian geometryczny (6x5) dla aktualnej konfiguracji.
        Zwraca macierz wiążącą prędkości złączy z prędkościami efektora.
        """

        matrices = self._compute_chain(th1, th2, th3, th4, th5) # Pobranie łańcucha kinematycznego
        p_e = matrices[-1][:3, 3] # Pozycja końcówki (TCP)
        J = np.zeros((6, 5)) # Inicjalizacja pustej macierzy
        
        # Iterujemy przez 5 złączy
        for i in range(5):
            # Pobieramy macierz transformacji poprzedniego układu współrzędnych
            T_prev = matrices[i] 
            
            # Oś obrotu Z to zawsze 3 kolumna macierzy rotacji
            z_axis = T_prev[:3, 2]
            
            # Pozycja środka układu współrzędnych
            p_curr = T_prev[:3, 3]
            
            # --- Część liniowa ---
            vec_diff = p_e - p_curr
            J[:3, i] = np.cross(z_axis, vec_diff)
            
            # --- Część kątowa ---
            J[3:, i] = z_axis
        return J
    
    def inverse_kinematics(self, R, Z, th1, phi_deg=0.0, elbow_up=True, reverse_base=False):
        """
        Funkcja rozwiązuje analitycznie zadanie odwrotne kinematyki metodą geometryczną.
        
        Algorytm redukuje układ trójwymiarowy manipulatora do problemu na płaszczyźnie 2D
        poprzez wyznaczenie orientacji bazy. Zmienne konfiguracyjne ramienia obliczane są 
        z wykorzystaniem twierdzenia cosinusów oraz funkcji atan2.
        """
        # Wyznaczenie orientacji efektora
        phi_rad = math.radians(phi_deg)
        phi_corr = phi_rad + self.geo_phi_offset
        
        # Uwzględnienie orientacji tylnej - robot nie musi się obracać w stronę punktu,
        # może spróbować sięgnąć za siebie
        R_ik = -R if reverse_base else R
        
        # Pozycja nadgarstka (Silnik E)
        R_wrist = R_ik - self.a1 - self.L3 * math.cos(phi_corr)
        Z_wrist = Z - self.d1 - self.L3 * math.sin(phi_corr)
        
        # Weryfikacja osiągalności (zoptymalizowana metoda pitagorasa)
        D_sq = R_wrist**2 + Z_wrist**2
        if D_sq > self.max_reach_sq or D_sq < self.min_reach_sq:
            return None
        
        # Twierdzenie cosinusów na preliczonych kwadratach
        cos_th3 = (D_sq - self.L1_sq - self.L2_sq) / self.denom
        
        # Obcięcie wartości wykraczających poza zakres [-1,1] (zabezpieczenie przed błędami float).
        cos_th3 = np.clip(cos_th3, -1.0, 1.0)
        
        # Pozycja ramienia L2 (Silnik Z)
        th3_ik = math.acos(cos_th3)
        if not elbow_up: th3_ik = -th3_ik
            
        # Kąty pomocnicze
        alpha_angle = math.atan2(Z_wrist, R_wrist)
        beta_angle = math.atan2(self.L2 * math.sin(th3_ik), self.L1 + self.L2 * math.cos(th3_ik))
        
        # Kąty poszczególnych silników
        th2 = alpha_angle - beta_angle
        th4_ik = phi_rad - th2 - th3_ik
        th3 = -th3_ik
        th4 = -th4_ik
        
        # Normalizacja kątów do zakresu [-pi, pi]
        th1 = math.atan2(math.sin(th1), math.cos(th1))
        th2 = math.atan2(math.sin(th2), math.cos(th2))
        th3 = math.atan2(math.sin(th3), math.cos(th3))
        th4 = math.atan2(math.sin(th4), math.cos(th4))
        
        return (th1, th2, th3, th4)

    def check_constraints(self, angles, positions):
        """
        Weryfikacja limitów i kolizji z podłożem.
        :param angles: krotka kątów (th1, th2, th3, th4)
        :param positions: tablica pozycji węzłów zwrócona przez get_joint_positions
        """
        th1, th2, th3, th4 = angles

        # --- Zakres ruchu ---
        if not (self.limits['th1'][0] <= th1 <= self.limits['th1'][1]): return False
        if not (self.limits['th2'][0] <= th2 <= self.limits['th2'][1]): return False
        if not (self.limits['th3'][0] <= th3 <= self.limits['th3'][1]): return False
        if not (self.limits['th4'][0] <= th4 <= self.limits['th4'][1]): return False

        # --- Kolizja z podłożem ---
        # Sprawdzenie Łokcia (T3) (16mm)
        if positions[3][2] - self.radius < 0: return False 
        
        # Sprawdzenie Efektora (TCP/TE)
        if positions[-1][2] < 0: return False 

        # Sprawdzenie Punktu Krytycznego
        p3 = positions[3]
        p4 = positions[4]

        vec_x = p4[0] - p3[0]
        vec_y = p4[1] - p3[1]
        vec_z = p4[2] - p3[2]

        # Przejście wzdłuż wektora T3-T4 (65.2mm od T3)
        z_axial = p3[2] + vec_z * self.crit_axial_factor
        
        # Rzutowanie offsetu grubości ramienia (29.2mm) na oś Z
        len_xy_raw = math.sqrt(vec_x**2 + vec_y**2)
        z_critical = z_axial - (len_xy_raw * self.crit_offset_factor)
        if z_critical < 0: return False

        return True

    def calculate_joint_distance(self, q_current, q_target):
        """
        Oblicza koszt ruchu robota - jak daleko robot musiałby się obrócić, aby przejść z obecnej 
        pozycji (q_current) do nowej (q_target). Służy do wyboru najmniej skomplikowanej trasy ruchu.
        """
        # Jeśli nie ma ważnego rozwiązania, koszt jest nieskończony.
        if q_target is None: return float('inf')
        
        total_dist = 0
        weights = [1.0, 15.0, 5.0, 1.0, 0] # wagi dla [Oś X, Oś YA, Oś Z, Oś E, Oś A]
        
        for i in range(4):
            diff = q_target[i] - q_current[i] # Różnica między pozycją docelową a obecną
            
            # Upewnienie się, że zawsze mierzymy najkrótszą drogę.
            # Zmiana z 350 stopni na 10 stopni to 20 stopni, a nie -340 stopni.
            normalized = (diff + math.pi) % (2 * math.pi) - math.pi
            total_dist += abs(normalized) * weights[i]
            
        return total_dist

    def calculate_configuration_cost(self, angles, current_angles):
        """
        Główna funkcja kosztu oceniająca jakość potencjalnej konfiguracji robota.
        Łączy niezależne koszty (limity, ruch, osobliwości) w jedną ważoną wartość.
        """
        th1, th2, th3, th4, th5 = angles
        
        ## ---------- KOSZT LIMITÓW ZŁĄCZY [0.0 - 50.0] ----------
        # Im bliżej limitów złączy, tym większa koszt, co wymusza pracę w bezpiecznym centrum zakresu
        # - Środek zakresu: koszt = 0
        # - 50% zakresu: koszt zaniedbywalny (~0.16 na złącze)
        # - 75% zakresu: koszt mały (~1.78 na złącze)
        # - 100% zakresu: koszt rośnie gwałtownie do 10.0 na złącze

        limit_cost = 0
        vals = [th1, th2, th3, th4, th5]
        lims = [self.limits['th1'], self.limits['th2'], self.limits['th3'], self.limits['th4'], self.limits['th5']]
        
        for v, (mn, mx) in zip(vals, lims):
            norm = (v - mn) / (mx - mn) # Normalizacja do zakresu [0, 1]
            centered = (norm - 0.5) * 2.0 # Normalizacja do zakresu [-1, 1]
            barrier = centered ** 6
            limit_cost += barrier * 10.0
            
        ## ---------- KOSZT RUCHU [1.0 - 50.0+] ----------
        # Im większy ruch musi wykonać, tym większy koszt
        motion_cost = 2.0 * self.calculate_joint_distance(current_angles, angles)
            
        ## ---------- KOSZT OSOBLIWOŚCI [0.0 - 1000.0] ----------
        J = self.get_jacobian(th1, th2, th3, th4, th5)
        # J(q) = 1/sqrt(det(J * J_T))
        J_pos = J[:3, :]
        manipulability = math.sqrt(np.linalg.det(J_pos @ J_pos.T))
        singularity_cost = 1.0 / (manipulability + 0.001)
        
        return limit_cost + motion_cost + singularity_cost

    def solve_ik(self, x, y, z, current_angles, phi_deg=None, roll_deg=0.0):
        """
        Główny interfejs sterowania. Konwertuje zadanie do postaci macierzowej (T_Goal),
        a następnie rozwiązuje problem odwrotny.
        """
        candidates = []
        th5_target = math.radians(roll_deg)

        # --- WARIANT 1: Zadana orientacja (Jednoznaczna macierz celu) ---
        if phi_deg is not None:
            # Tworzymy macierz 4x4
            T_goal = self._construct_matrix(x, y, z, phi_deg)
            sol, strategy = self._solve_from_matrix(T_goal, current_angles)
            return (sol + (th5_target,), strategy) if sol else (None, "Cel nieosiągalny")

        # --- WARIANT 2: Auto-orientacja (Optymalizacja macierzy celu) ---
        else:
            # Przeszukujemy przestrzeń możliwych macierzy T, aby znaleźć tę najbardziej korzystną
            for phi in range(-180, 180, 10):
                T_candidate = self._construct_matrix(x, y, z, phi)
                
                # Sprawdzamy, czy ta konkretna macierz jest osiągalna
                sol, name = self._solve_from_matrix(T_candidate, current_angles, check_strategies=True)
                
                if sol:
                    full_sol = sol + (th5_target,)
                    cost = self.calculate_configuration_cost(full_sol, current_angles)
                    candidates.append((cost, full_sol, f"{name} (Auto φ={phi}°)"))
            
            if not candidates:
                return None, "Brak rozwiązania"
            
            # Zwracamy rozwiązanie o najniższym koszcie
            candidates.sort(key=lambda x: x[0])
            return candidates[0][1], candidates[0][2]

    def _construct_matrix(self, x, y, z, phi_deg):
        """Konstrukcja Macierzy Transformacji Jednorodnej (4x4) z danych wejściowych."""
        yaw = math.atan2(y, x)
        pitch = math.radians(phi_deg)
        
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        
        # Złożenie rotacji: R = RotZ(yaw) * RotY(pitch)
        R = np.array([
            [cy * cp, -sy, cy * sp],
            [sy * cp,  cy, sy * sp],
            [   -sp,    0,     cp]
        ])
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def _solve_from_matrix(self, T, current_angles, check_strategies=False):
        """Dekompozycja macierzy T i rozwiązanie analityczne układu równań."""
        
        # 1. Ekstrakcja danych z macierzy T (Pozycja + Wektor podejścia)
        p_x, p_y, p_z = T[0, 3], T[1, 3], T[2, 3]
        
        # Odtworzenie kąta pochylenia z wektora normalnego (kolumna 3)
        a_x, a_y, a_z = T[0, 2], T[1, 2], T[2, 2]
        phi_deg = math.degrees(math.atan2(a_z, math.sqrt(a_x**2 + a_y**2)))
        
        # 2. Parametry pomocnicze do solvera geometrycznego
        R_target = math.sqrt(p_x**2 + p_y**2)
        th1_base = math.atan2(p_y, p_x)

        # Definicja strategii (Elbow Up/Down, Base Normal/Reverse)
        strategies = [(True, False), (False, False)]
        if p_x <= 50: strategies.extend([(True, True), (False, True)])

        best_sol = None
        min_cost = float('inf')
        best_name = ""

        for elbow_up, reverse_base in strategies:
            # Dostosowanie kąta bazy dla trybu wstecznego
            th1_in = ((th1_base + math.pi) + math.pi) % (2*math.pi) - math.pi if reverse_base else th1_base
            

            sol = self.inverse_kinematics(R_target, p_z, th1_in, phi_deg, elbow_up, reverse_base)
            
            if sol:
                fk_positions = self.get_joint_positions(*sol)
                if self.check_constraints(sol, fk_positions):
                    if check_strategies:
                        return sol, "Auto"
                    
                    cost = self.calculate_configuration_cost(sol + (current_angles[4],), current_angles)
                    if cost < min_cost:
                        min_cost = cost
                        best_sol = sol
                        best_name = "Elbow Up" if elbow_up else "Elbow Down"

        return (best_sol, best_name) if best_sol else (None, None)
    
# =====================================================================
# GUI APPLICATION
# =====================================================================

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Panel")
        self.root.geometry("1350x700")

        # Inicjalizacja modelu
        self.kin = RobotKinematics()

        self.last_gui_update_time = 0.0 # Timestamp ostatniej aktualizacji GUI
        self.last_tcp_pos = np.array([0.0, 0.0, 0.0]) # Ostatnia znana pozycja TCP
        
        # Komunikacja z robotem
        self.robot = RobotSerial()
        self.robot.gui_callback = self.update_position_display
        
        # Dane do wizualizacji 3D
        self.plot_artists = []
        # Maksymalny zasięg robota (do wizualizacji)
        self.max_reach = self.kin.a2 + self.kin.a3 + self.kin.a4 + self.kin.d6
        self.control_mode = tk.StringVar(value='position')
        
        # Sterowanie kątami (ciągłe wysyłanie)
        self.angle_send_timer = None
        self.angle_send_active = False
        self._collision_warning_shown = False
        self._last_collision_state = False
        
        # Dane do sekwencji ruchów
        self.is_connected = False
        self.sequence_data = []  
        self.sequence_list_frame = None
        self.canvas_window_seq = None
        self.current_sequence_index = 0
        self.sequence_playing = False
        self.sequence_timer = None
        self.play_button = None 
        self.target_xyz = None 
        self.POSITION_TOLERANCE = 2.0  # mm

        # Inicjalizacja GUI
        self.setup_ui()
        self.setup_3d_plot()
        self.update_3d_visualization()
        
        # Próba automatycznego połączenia z robotem po starcie aplikacji
        self.root.after(100, self.connect_robot)

    def setup_ui(self):
        # Definicja kolorów
        COLORS = {
            'BG': "#2f3347", 'FRAME': "#262a3e", 'BORDER': "#565a6c",
            'TEXT': "#c4cad0", 'ACCENT': "#101122", 'BTN': "#2f3347",
            'STOP': "#cb0000", 'PLAY': "#046D04", 'STOPHOV': "#cb0000", 'PLAYHOV': "#046D04"
        }

        # Podstawowe style dla elementów ttk
        style = ttk.Style()
        style.theme_use('clam')
        style.configure(".", background=COLORS['BG'], foreground=COLORS['TEXT'], fieldbackground=COLORS['FRAME'],
                        darkcolor=COLORS['FRAME'], lightcolor=COLORS['FRAME'], bordercolor=COLORS['BORDER'], font=('Arial', 10))
        style.configure("TLabelframe", background=COLORS['FRAME'], bordercolor=COLORS['BORDER'], relief="solid")
        style.configure("Header.TLabel", background=COLORS['FRAME'], foreground=COLORS['TEXT'], padding=2, font=('Arial', 10, 'bold'), anchor='center')
        style.configure("TLabel", background=COLORS['FRAME'], foreground=COLORS['TEXT'])
        style.configure("TEntry", fieldbackground=COLORS['FRAME'], foreground=COLORS['TEXT'], insertcolor=COLORS['TEXT'], bordercolor=COLORS['BORDER'])
        style.configure("TRadiobutton", background=COLORS['FRAME'], foreground=COLORS['TEXT'])
        style.map("TRadiobutton", background=[('active', COLORS['FRAME'])], foreground=[('active', COLORS['TEXT'])])
        
        # Przyciski
        btn_configs = [
            ("TButton", COLORS['BTN'], COLORS['ACCENT']),
            ("Play.TButton", COLORS['PLAY'], COLORS['PLAYHOV']),
            ("Stop.TButton", COLORS['STOP'], COLORS['STOPHOV'])
        ]
        for name, bg, active in btn_configs:
            style.configure(name, background=bg, foreground="white" if name != "TButton" else COLORS['TEXT'], bordercolor=COLORS['BORDER'])
            style.map(name, background=[('active', active)])

        # Główna ramka
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nswe")
        
        #  Lewa kolumna - kontrolki
        scroll_canvas = tk.Canvas(main_frame, borderwidth=0, background=COLORS['BG'], highlightthickness=0)
        scroll_canvas.grid(row=0, column=0, sticky="nswe", padx=(0, 5))
        vscrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=scroll_canvas.yview)
        vscrollbar.grid(row=0, column=0, sticky="nse") 
        scroll_canvas.configure(yscrollcommand=vscrollbar.set)
        
        self.content_frame = tk.Frame(scroll_canvas, bg=COLORS['BG']) 
        self.content_frame.grid_columnconfigure(0, weight=1) 
        self.canvas_window = scroll_canvas.create_window((0, 0), window=self.content_frame, anchor="nw", width=1) 
        
        # Zmiana rozmiaru zawartości dla scrollbara
        self.content_frame.bind("<Configure>", lambda e: scroll_canvas.configure(scrollregion=scroll_canvas.bbox("all")))
        scroll_canvas.bind("<Configure>", lambda e: scroll_canvas.itemconfig(self.canvas_window, width=e.width - vscrollbar.winfo_width()))

        # Prawa kolumna - wizualizacja 3D
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky="nswe", padx=(5, 0))
        
        # ================================================
        # ============ ELEMENTY LEWEJ KOLUMNY ============
        # ================================================
        
        # 1. Status
        header_label = ttk.Label(self.content_frame, text="Status połączenia", style="Header.TLabel")
        status_frame = ttk.LabelFrame(self.content_frame, labelwidget=header_label, padding="5")
        status_frame.grid(row=0, column=0, sticky="ew", pady=5, padx=5)
        self.status_label = ttk.Label(status_frame, text="Rozłączony", foreground="red")
        self.status_label.grid(row=0, column=0, sticky="w")
        ttk.Button(status_frame, text="Połącz", command=self.connect_robot).grid(row=0, column=1, padx=5)
        
        # 2. Tryb sterowania
        mode_header = ttk.Label(self.content_frame, text="Tryb sterowania", style="Header.TLabel")
        mode_frame = ttk.LabelFrame(self.content_frame, labelwidget=mode_header, padding="5")
        mode_frame.grid(row=1, column=0, sticky="ew", pady=5, padx=5)
        ttk.Radiobutton(mode_frame, text="Sterowanie pozycją (XYZ)", variable=self.control_mode, value='position', 
                        command=self.switch_control_mode, style="TRadiobutton").grid(row=0, column=0, sticky="w", pady=2)
        ttk.Radiobutton(mode_frame, text="Sterowanie kątami (ciągłe)", variable=self.control_mode, value='angles', 
                        command=self.switch_control_mode, style="TRadiobutton").grid(row=0, column=1, sticky="w", pady=2)
        
        # 3. Aktualna pozycja
        pos_header = ttk.Label(self.content_frame, text="Aktualna pozycja", style="Header.TLabel")
        pos_frame = ttk.LabelFrame(self.content_frame, labelwidget=pos_header, padding="5")
        pos_frame.grid(row=2, column=0, sticky="ew", pady=5, padx=5)
        
        self.pos_labels = {}
        for i, axis in enumerate(['X', 'Y', 'Z', 'E', 'S', 'A']):
            ttk.Label(pos_frame, text=f"{axis}:").grid(row=0, column=i, sticky="w", pady=2)
            lbl = ttk.Label(pos_frame, text="0.00°", font=('Arial', 10))
            lbl.grid(row=0, column=i, sticky="w", padx=10)
            self.pos_labels[axis] = lbl
            
        ttk.Separator(pos_frame, orient='horizontal').grid(row=1, column=0, columnspan=5, sticky='ew', pady=5)
        
        self.tcp_labels = {}
        for i, axis in enumerate(['TCP_X', 'TCP_Y', 'TCP_Z']):
            col = i * 2
            ttk.Label(pos_frame, text=f"{axis.replace('TCP_', '')} [mm]:").grid(row=2, column=col, sticky="w", pady=2)
            lbl = ttk.Label(pos_frame, text="0.00", font=('Arial', 10, 'bold'), foreground="#377df0")
            lbl.grid(row=2, column=col+1, sticky="w", padx=5)
            self.tcp_labels[axis] = lbl

        # 4. Kąty docelowe (IK)
        angles_header = ttk.Label(self.content_frame, text="Kąty docelowe (wynik IK)", style="Header.TLabel")
        self.angles_frame = ttk.LabelFrame(self.content_frame, labelwidget=angles_header, padding="10")
        self.angles_frame.grid(row=3, column=0, sticky="ew", pady=5, padx=5)
        self.angle_labels = {}
        labels_map = ['θ1', 'θ2', 'θ3', 'θ4', 'θ5']
        for i, axis in enumerate(labels_map):
            ttk.Label(self.angles_frame, text=f"{axis}:").grid(row=0, column=i*2, sticky="e", pady=2)
            lbl = ttk.Label(self.angles_frame, text="0.00°", font=('Arial', 10))
            lbl.grid(row=0, column=i*2+1, sticky="w", padx=5, pady=2)
            self.angle_labels[axis] = lbl
        
        # 5. Sterowanie kątami (Suwaki)
        control_header = ttk.Label(self.content_frame, text="Sterowanie kątami [°]", style="Header.TLabel")
        self.angle_control_frame = ttk.LabelFrame(self.content_frame, labelwidget=control_header, padding="10")
        self.angle_control_frame.grid(row=4, column=0, sticky="ew", pady=5, padx=5)
        
        self.angle_sliders = {}
        self.angle_value_labels = {}
        joint_defs = [
            ('th1', 'θ1 (X - Baza)'), 
            ('th2', 'θ2 (Y - Ramię L2)'), 
            ('th3', 'θ3 (Z - Ramię L3)'), 
            ('th4', 'θ4 (E - Nadgarstek)'),
            ('th5', 'θ5 (A - Obrót)')
        ]
        
        for i, (key, label) in enumerate(joint_defs):
            ttk.Label(self.angle_control_frame, text=label).grid(row=i, column=0, sticky="w", pady=5)
            # UŻYCIE LIMITÓW Z KLASY KINEMATYKI
            min_rad, max_rad = self.kin.limits[key]
            
            slider = tk.Scale(self.angle_control_frame, from_=math.degrees(min_rad), to=math.degrees(max_rad),    
                              orient=tk.HORIZONTAL, resolution=0.1, length=200,
                              bg=COLORS['FRAME'], fg=COLORS['TEXT'], troughcolor=COLORS['BG'], highlightthickness=0, 
                              command=lambda val, k=key: self.on_angle_slider_change(k, val))
            slider.set(0.0)
            slider.grid(row=i, column=1, padx=5, pady=5)
            self.angle_sliders[key] = slider
            
            val_lbl = ttk.Label(self.angle_control_frame, text="0.0°")
            val_lbl.grid(row=i, column=2, padx=5)
            self.angle_value_labels[key] = val_lbl
        
        self.live_control_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(self.angle_control_frame, text="WŁĄCZ LIVE (Transmisja ciągła)", variable=self.live_control_var, 
                        command=self.toggle_live_control, style="TRadiobutton").grid(row=len(joint_defs), column=0, columnspan=3, pady=10, sticky="w")

        ttk.Button(self.angle_control_frame, text="DODAJ AKTUALNE KĄTY (do sekwencji)", 
                   command=self.add_current_angles_to_sequence).grid(row=len(joint_defs)+1, column=0, columnspan=3, pady=5, sticky="ew")
        
        # 6. Pozycja docelowa (Input XYZ)
        target_header = ttk.Label(self.content_frame, text="Pozycja docelowa [mm]", style="Header.TLabel")
        self.target_frame = ttk.LabelFrame(self.content_frame, labelwidget=target_header, padding="10")
        self.target_frame.grid(row=5, column=0, sticky="ew", pady=5, padx=5)
        
        input_defs = [
            ("X:", "200", "x_entry"), 
            ("Y:", "-200", "y_entry"), 
            ("Z:", "250", "z_entry"), 
            ("Pitch φ [°]:", "0", "phi_entry"),
            ("Roll [°]:", "0", "roll_entry")
        ]
        for i, (lbl_txt, def_val, attr_name) in enumerate(input_defs):
            ttk.Label(self.target_frame, text=lbl_txt).grid(row=i, column=0, sticky="w", pady=5)
            entry = ttk.Entry(self.target_frame, width=15)
            entry.grid(row=i, column=1, padx=5)
            entry.insert(0, def_val)
            setattr(self, attr_name, entry)
        
        self.auto_phi_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.target_frame, text="Orientacja automatyczna",variable=self.auto_phi_var, 
                        command=self.toggle_phi_entry, style="TRadiobutton").grid(row=5, column=0, columnspan=2, pady=5, sticky="w")
        
        btn_row = ttk.Frame(self.target_frame)
        btn_row.grid(row=6, column=0, columnspan=2, sticky="ew", pady=5)
        btn_row.columnconfigure((0, 1), weight=1, uniform="g")
        ttk.Button(btn_row, text="WYŚLIJ POZYCJĘ", command=self.send_position).grid(row=0, column=0, padx=(0, 2), sticky="ew")
        ttk.Button(btn_row, text="DODAJ PUNKT", command=self.add_point_to_sequence).grid(row=0, column=1, padx=(2, 0), sticky="ew")
        
        self.toggle_phi_entry()
        
        # 7. Log
        log_header = ttk.Label(self.content_frame, text="Log komunikacji", style="Header.TLabel")
        log_frame = ttk.LabelFrame(self.content_frame, labelwidget=log_header, padding="10")
        log_frame.grid(row=7, column=0, sticky="ew", pady=5, padx=5)
        self.log_text = tk.Text(log_frame, height=10, width=50, bg=COLORS['FRAME'], fg=COLORS['TEXT'], 
                                insertbackground=COLORS['TEXT'], selectbackground=COLORS['ACCENT'],
                                highlightbackground=COLORS['BORDER'], highlightthickness=1)
        self.log_text.grid(row=0, column=0, sticky="nswe")
        sb = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.log_text['yscrollcommand'] = sb.set
        log_frame.grid_columnconfigure(0, weight=1)

        # 8. Programowanie sekwencyjne
        self._create_sequence_frame(self.content_frame, 8)

        # ================================================
        # ============ ELEMENTY PRAWEJ KOLUMNY ===========
        # ================================================

        viz_frame = ttk.LabelFrame(right_frame, text="Wizualizacja 3D", padding="10")
        viz_frame.grid(row=0, column=0, sticky="nswe")
        
        self.fig = Figure(figsize=(7, 6), dpi=100, facecolor=COLORS['BG']) 
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Konfiguracja kolorów wykresu
        self.ax.set_facecolor(COLORS['BG'])
        for axis in [self.ax.xaxis, self.ax.yaxis, self.ax.zaxis]:
            axis.label.set_color(COLORS['TEXT'])
            axis.set_pane_color((0.14, 0.16, 0.24, 1.0))
            axis.set_tick_params(colors=COLORS['TEXT'])
            
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        ttk.Button(viz_frame, text="Resetuj widok", command=self.reset_3d_view).pack(side=tk.TOP, anchor="nw", pady=5, padx=5)
        self.canvas.get_tk_widget().config(bg=COLORS['BG'])
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Konfiguracja wag grid
        self.root.columnconfigure(0, weight=1); self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1); main_frame.columnconfigure(1, weight=1) 
        main_frame.rowconfigure(0, weight=1); 
        right_frame.rowconfigure(0, weight=1); right_frame.columnconfigure(0, weight=1)
        
        self.switch_control_mode()

    def setup_3d_plot(self):
        """Inicjalizacja statycznych elementów wykresu."""
        L_COL = "#c4cad0"
        self.ax.set_xlabel('X [mm]', color=L_COL)
        self.ax.set_ylabel('Y [mm]', color=L_COL)
        self.ax.set_zlabel('Z [mm]', color=L_COL)
        self.ax.set_title('Pozycja robota', color=L_COL)
        
        self.ax.set_xlim([-self.max_reach, self.max_reach])
        self.ax.set_ylim([-self.max_reach, self.max_reach])
        self.ax.set_zlim([0, self.max_reach])
        self.ax.view_init(elev=20, azim=45)
        
        # Siatka
        r = np.arange(-300, 301, 100)
        X, Y = np.meshgrid(r, r)
        self.ax.plot_wireframe(X, Y, np.zeros_like(X), alpha=0.1, color='gray', linewidth=0.5)

    def reset_3d_view(self):
        self.ax.set_xlim([-self.max_reach, self.max_reach])
        self.ax.set_ylim([-self.max_reach, self.max_reach])
        self.ax.set_zlim([0, self.max_reach])
        self.ax.view_init(elev=20, azim=45)
        self.canvas.draw()

    def update_3d_visualization(self):
        """Odświeża pozycję ramienia na wykresie 3D."""
        try:
            # Usunięcie starych elementów (linii i strzałek)
            for artist in self.plot_artists:
                if isinstance(artist, list):
                    for i in artist: i.remove()
                else:
                    artist.remove()
            self.plot_artists.clear()
            
            # Pobranie pozycji
            angles = self.robot.get_current_angles()
            rads = [math.radians(a) for a in angles[:5]]
            
            # Pozycje węzłów
            positions = self.kin.get_joint_positions(*rads)
            
            # Orientacja TCP
            Te = self.kin.get_tcp_matrix(*rads)
            R = Te[:3, :3]
            
            # Szkielet robota
            xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]
            lines = self.ax.plot(xs, ys, zs, 'o-', linewidth=2, markersize=6, color='#377df0', markerfacecolor='lightblue')
            tcp = self.ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], c="#cb0000", s=50, edgecolors='darkred', linewidths=2)
            
            # Wektory osi (Strzałki)
            len_vec = 40.0
            tcp_x, tcp_y, tcp_z = xs[-1], ys[-1], zs[-1]
            
            ax_x = self.ax.quiver(tcp_x, tcp_y, tcp_z, R[0,0], R[1,0], R[2,0], length=len_vec, color='b')
            ax_y = self.ax.quiver(tcp_x, tcp_y, tcp_z, R[0,1], R[1,1], R[2,1], length=len_vec, color='g')
            ax_z = self.ax.quiver(tcp_x, tcp_y, tcp_z, R[0,2], R[1,2], R[2,2], length=len_vec, color='r')
            
            self.plot_artists.extend([lines, tcp, ax_x, ax_y, ax_z])
            
            # Etykiety
            labels = ['Base', 'T1', 'T2', 'T3', '', '', 'TE']
            for i, pos in enumerate(positions):
                if i < len(labels):
                    txt = self.ax.text(pos[0], pos[1], pos[2], f'  {labels[i]}', fontsize=8, color="#c4cad0")
                    self.plot_artists.append(txt)
            
            self.canvas.draw()
            
        except Exception as e:
            if not hasattr(self, '_viz_err'): 
                self.log(f"Błąd wizualizacji: {e}")
                self._viz_err = True
        
        self.root.after(50, self.update_3d_visualization)
        
    def log(self, message):
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)

    def update_position_display(self, angles=None, log_message=None):
        """Callback z wątku serial."""
        
        def _update_ui():
            if log_message:
                if log_message == "#CONNECTION_LOST#":
                    self.status_label.config(text="Rozłączony (Błąd I/O)", foreground="red")
                    self.log("BŁĄD KRYTYCZNY: Utracono połączenie")
                else:
                    self.log(log_message)
            
            if angles:
                # Aktualizacja etykiet kątów
                for i, axis in enumerate(['X', 'Y', 'Z', 'E', 'S', 'A']):
                    if i < len(angles):
                        self.pos_labels[axis].config(text=f"{angles[i]:.2f}°")
                
                # Obliczanie pozycji tcp
                current_rads = [math.radians(a) for a in angles[:5]]
                tcp_matrix = self.kin.get_tcp_matrix(*current_rads)
                current_xyz = tcp_matrix[:3, 3]
                self.last_tcp_pos = current_xyz
                for i, axis in enumerate(['TCP_X', 'TCP_Y', 'TCP_Z']):
                    self.tcp_labels[axis].config(text=f"{current_xyz[i]:.2f}")

        # Limitowanie częstotliwości odświeżania GUI (throttle)
        now = time.time() * 1000.0
        if log_message or (now - self.last_gui_update_time >= 200.0):
            self.last_gui_update_time = now
            self.root.after(0, _update_ui)

    def toggle_phi_entry(self):
        state = 'disabled' if self.auto_phi_var.get() else 'normal'
        self.phi_entry.config(state=state)

    def connect_robot(self):
        success, msg = self.robot.connect()
        self.status_label.config(text="Połączono" if success else "Rozłączony", 
                                 foreground="green" if success else "red")
        if not success: messagebox.showerror("Błąd", msg)
        self.log(msg)
        self.is_connected = success
    
    def send_position(self):
        try:
            def safe_float(entry, default=0.0):
                txt = entry.get().strip().replace(',', '.')
                return float(txt) if txt else default

            x = safe_float(self.x_entry)
            y = safe_float(self.y_entry)
            z = safe_float(self.z_entry)

            if self.auto_phi_var.get():
                phi = None 
            else:
                phi = safe_float(self.phi_entry)

            roll = safe_float(self.roll_entry)

            self.log(f"IK dla: X={x}, Y={y}, Z={z}, R={roll}")
            cur_rads = [math.radians(a) for a in self.robot.get_current_angles()[:5]]
            
            sol, name = self.kin.solve_ik(x, y, z, tuple(cur_rads), phi_deg=phi, roll_deg=roll)
            
            if sol is None:
                messagebox.showerror("Błąd IK", "Pozycja poza zasięgiem.")
                return
            
            pos = self.kin.get_joint_positions(*sol)
            if pos[5][2] < -0.01 or pos[6][2] < -0.01:
                messagebox.showerror("Kolizja", "Rozwiązanie koliduje z podłożem!")
                return
            
            labels = ['θ1', 'θ2', 'θ3', 'θ4', 'θ5']
            for i, (val, lbl) in enumerate(zip(sol, labels)):
                self.angle_labels[lbl].config(text=f"{math.degrees(val):.2f}°")
            
            ok, msg = self.robot.send_target_angles(*sol)
            if ok: 
                self.log(f"Wysłano konfigurację: {name}")
            else: 
                messagebox.showerror("Błąd komunikacji", msg)
                
        except ValueError as e:
            messagebox.showerror("Błąd danych", str(e))
        except KeyError as e:
            messagebox.showerror("Błąd GUI", f"Brak klucza w słowniku etykiet: {e}")
            self.log(f"DEBUG: Dostępne klucze: {list(self.angle_labels.keys())}")
        except Exception as e:
            self.log(f"Nieoczekiwany błąd: {e}")
            messagebox.showerror("Błąd krytyczny", str(e))
    
    def on_angle_slider_change(self, key, val):
        angle = float(val)
        self.angle_value_labels[key].config(text=f"{angle:.1f}°")
        
        # Szybkie sprawdzenie kolizji dla suwaków przez FK
        rads = [math.radians(self.angle_sliders[k].get()) for k in ['th1','th2','th3','th4','th5']]
        pos = self.kin.get_joint_positions(*rads)
        min_z = np.min(pos[:, 2])
        
        is_collision = min_z < 0
        color = 'red' if is_collision else 'green'
        self.angle_value_labels[key].config(foreground=color)
        
        if is_collision and not self._last_collision_state:
            self.log(f"OSTRZEŻENIE: Kolizja Z={min_z:.1f}mm")
        self._last_collision_state = is_collision
    
    def switch_control_mode(self):
        mode = self.control_mode.get()
        self.stop_continuous_send()
        self.live_control_var.set(False)
        
        if mode == 'position':
            self.target_frame.grid()
            self.angle_control_frame.grid_remove()
            self.angles_frame.grid()
            self.log("Tryb: Pozycja (XYZ)")
        else:
            cur = self.robot.get_current_angles()
            for i, k in enumerate(['th1','th2','th3','th4']):
                self.angle_sliders[k].set(cur[i])
            
            self.target_frame.grid_remove()
            self.angle_control_frame.grid()
            self.angles_frame.grid_remove()
            self.log("Tryb: Kąty (Joints)")
    
    def start_continuous_send(self):
        self.angle_send_active = True
        self.send_angles_continuously()
    
    def stop_continuous_send(self):
        self.angle_send_active = False
        if self.angle_send_timer:
            self.root.after_cancel(self.angle_send_timer)
            self.angle_send_timer = None
    
    def send_angles_continuously(self):
        if not self.angle_send_active: return
        
        rads = [math.radians(self.angle_sliders[k].get()) for k in ['th1','th2','th3','th4','th5']]
        pos = self.kin.get_joint_positions(*rads)
        
        if np.min(pos[:, 2]) < 0:
            if not self._collision_warning_shown:
                self.log("KRYTYCZNE: Ruch zablokowany (Kolizja)")
                self._collision_warning_shown = True
        else:
            if self._collision_warning_shown:
                self.log("Strefa bezpieczna")
                self._collision_warning_shown = False
            
            t = time.time()
            if not hasattr(self, '_last_snd') or t - self._last_snd >= 0.2:
                self.robot.send_target_angles(*rads)
                self._last_snd = t
                
        self.angle_send_timer = self.root.after(50, self.send_angles_continuously)
    
    def on_closing(self):
        self.is_connected = False
        self.stop_continuous_send()
        self.stop_sequence("APP CLOSE")
        self.robot.close()
        self.root.destroy()
        
    # --- PROGRAMOWANIE SEKWENCYJNE ---
    
    def toggle_live_control(self):
        if self.live_control_var.get():
            if self.sequence_playing:
                self.live_control_var.set(False)
                return
            self.start_continuous_send()
        else:
            self.stop_continuous_send()

    def _create_sequence_frame(self, master, row):
        frame = ttk.LabelFrame(master, text="Rejestrator/Odtwarzacz sekwencji", padding="10")
        frame.grid(row=row, column=0, sticky="nsew", pady=5, padx=5)
        frame.columnconfigure(0, weight=1)
        
        ctrl = tk.Frame(frame, bg="#262a3e")
        ctrl.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 5))
        ctrl.columnconfigure((0, 4), weight=1)
        
        ttk.Button(ctrl, text="WCZYTAJ", command=self.load_sequence_from_json).grid(row=0, column=1, padx=5)
        ttk.Button(ctrl, text="ZAPISZ", command=self.save_sequence_to_json).grid(row=0, column=2, padx=5)
        self.play_button = ttk.Button(ctrl, text="START SEKWENCJI", command=self.play_sequence, style='Play.TButton')
        self.play_button.grid(row=0, column=3, padx=5)
        
        ttk.Separator(frame, orient='horizontal').grid(row=1, column=0, columnspan=2, sticky='ew', pady=5)
        
        cnv = tk.Canvas(frame, borderwidth=0, background="#262a3e", highlightthickness=0)
        vsb = ttk.Scrollbar(frame, orient="vertical", command=cnv.yview)
        
        self.sequence_list_frame = ttk.Frame(cnv, padding="5", style='TLabelframe')
        self.sequence_list_frame.columnconfigure(0, weight=1)
        
        self.canvas_window_seq = cnv.create_window((0, 0), window=self.sequence_list_frame, anchor="nw", width=1)
        cnv.configure(yscrollcommand=vsb.set)
        
        cnv.grid(row=2, column=0, sticky="nswe")
        vsb.grid(row=2, column=1, sticky="ns")
        frame.rowconfigure(2, weight=1)
        
        self.sequence_list_frame.bind("<Configure>", lambda e: cnv.configure(scrollregion=cnv.bbox("all")))
        cnv.bind("<Configure>", lambda e: cnv.itemconfig(self.canvas_window_seq, width=e.width - vsb.winfo_width()))
        
        self._update_sequence_display()

    def load_sequence_from_json(self):
        f = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if f:
            try:
                with open(f, 'r', encoding='utf-8') as fp:
                    self.sequence_data = json.load(fp)
                self._update_sequence_display()
                self.log(f"Wczytano {len(self.sequence_data)} pkt")
            except Exception as e:
                messagebox.showerror("Błąd", str(e))
    
    def save_sequence_to_json(self):
        f = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if f:
            try:
                with open(f, 'w', encoding='utf-8') as fp:
                    json.dump(self.sequence_data, fp, indent=4)
                self.log("Zapisano sekwencję")
            except Exception as e:
                messagebox.showerror("Błąd zapisu", str(e))

    def play_sequence(self):
        if not self.is_connected:
            messagebox.showwarning("Info", "Brak połączenia")
            return
        
        if self.live_control_var.get():
            self.live_control_var.set(False)
            self.stop_continuous_send()
            
        if self.sequence_playing:
            self.stop_sequence()
        elif self.sequence_data:
            self.sequence_playing = True
            self.current_sequence_index = 0
            self.play_button.config(text="STOP SEKWENCJI", style='Stop.TButton')
            self._execute_sequence_step()
            
    def stop_sequence(self, msg="Stop"):
        self.sequence_playing = False
        if self.sequence_timer:
            self.root.after_cancel(self.sequence_timer)
            self.sequence_timer = None
        if self.play_button:
            self.play_button.config(text="START SEKWENCJI", style='Play.TButton')
        self.log(msg)
        self._update_sequence_display()

    def _execute_sequence_step(self):
        if not self.sequence_playing or self.current_sequence_index >= len(self.sequence_data):
            self.stop_sequence("Koniec sekwencji")
            return
        
        pt = self.sequence_data[self.current_sequence_index]
        self.log(f"Krok {self.current_sequence_index+1}: {pt['X'], pt['Y'], pt['Z']}")
        self.target_xyz = np.array([pt['X'], pt['Y'], pt['Z']])
        self._update_sequence_display(self.current_sequence_index)
        
        try:
            if pt.get('Joints'):
                raw_joints = pt['Joints']
                if len(raw_joints) == 4: raw_joints.append(0.0)
                rads = [math.radians(d) for d in raw_joints]
                self.robot.send_target_angles(*rads)
            else:
                cur = [math.radians(a) for a in self.robot.get_current_angles()[:4]]
                phi = pt['Fi'] if not pt['Auto_Fi'] else None
                roll = pt.get('Rol', 0.0)

                # UŻYCIE KLASY KINEMATYKI
                sol, _ = self.kin.solve_ik(pt['X'], pt['Y'], pt['Z'], tuple(cur), phi_deg=phi, roll_deg=roll)
                
                if sol: self.robot.send_target_angles(*sol)
                else: raise Exception("Brak IK")
                
            self._wait_for_position()
        except Exception as e:
            self.stop_sequence(f"Błąd kroku: {e}")

    def _wait_for_position(self):
        if not self.sequence_playing: return
        
        dist = np.linalg.norm(self.last_tcp_pos - self.target_xyz)
        if dist <= self.POSITION_TOLERANCE:
            self.current_sequence_index += 1
            self.sequence_timer = self.root.after(100, self._execute_sequence_step)
        else:
            self.sequence_timer = self.root.after(100, self._wait_for_position)

    def add_point_to_sequence(self):
        try:
            x, y, z = float(self.x_entry.get()), float(self.y_entry.get()), float(self.z_entry.get())
            auto = self.auto_phi_var.get()
            phi = None if auto else float(self.phi_entry.get())
            roll = float(self.roll_entry.get())

            cur = [math.radians(a) for a in self.robot.get_current_angles()[:5]]
            
            # Weryfikacja osiągalności przed dodaniem
            sol, _ = self.kin.solve_ik(x, y, z, tuple(cur), phi_deg=phi, roll_deg=roll)
            
            if sol:
                self.sequence_data.append({"X":x, "Y":y, "Z":z, "Fi":phi if phi else 0, "Auto_Fi":auto,"Rol": roll,})
                self._update_sequence_display()
            else:
                messagebox.showerror("Błąd", "Pozycja nieosiągalna")
        except ValueError: pass

    def add_current_angles_to_sequence(self):
        rads = [math.radians(self.angle_sliders[k].get()) for k in ['th1','th2','th3','th4','th5']]
        
        # Pobranie pozycji przez FK
        pos = self.kin.get_joint_positions(*rads)[-1]
        fi_rad = math.atan2(math.sin(sum(rads[1:4])), math.cos(sum(rads[1:4])))
        
        roll_val = self.angle_sliders['th5'].get()

        self.sequence_data.append({
            "X": round(pos[0], 2), "Y": round(pos[1], 2), "Z": round(pos[2], 2),
            "Fi": round(math.degrees(fi_rad), 2),
            "Rol": round(roll_val, 2),
            "Auto_Fi": False,
            "Joints": [math.degrees(r) for r in rads]
        })
        self._update_sequence_display()

    def remove_point(self, idx):
        del self.sequence_data[idx]
        self._update_sequence_display()

    def _update_sequence_display(self, highlight_idx=None):
        for w in self.sequence_list_frame.winfo_children(): w.destroy()
        
        if not self.sequence_data:
            ttk.Label(self.sequence_list_frame, text="Brak punktów").grid(row=0, column=0, padx=5)
            return
            
        for i, pt in enumerate(self.sequence_data):
            is_hl = (i == highlight_idx and self.sequence_playing)
            fg = "#FFD700" if is_hl else "#c4cad0"
            style = 'TButton' if is_hl else 'TLabelframe'
            
            f = ttk.Frame(self.sequence_list_frame, padding="5", relief=tk.SOLID, borderwidth=1, style=style)
            f.grid(row=i, column=0, sticky="ew", pady=2); f.columnconfigure(1, weight=1)
            
            ttk.Label(f, text=f"PKT {i+1}", font=('Arial', 10, 'bold'), foreground=fg).grid(row=0, column=0, sticky="w")
            txt = f"X:{pt['X']:.1f} Y:{pt['Y']:.1f} Z:{pt['Z']:.1f}"
            ttk.Label(f, text=txt, foreground=fg).grid(row=1, column=0, sticky="w")
            ttk.Button(f, text="✕", width=2, command=lambda x=i: self.remove_point(x)).grid(row=0, column=1, rowspan=2, sticky="e")

def main():
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()