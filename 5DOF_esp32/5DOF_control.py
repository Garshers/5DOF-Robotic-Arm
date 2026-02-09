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

# -------------------- KOMUNIKACJA UART Z ESP-32 ----------------------

class RobotSerial:
    def __init__(self, port=None, baudrate=115200):
        self.port_name = port
        self.baudrate = baudrate

        self.ser = None             # Obiekt Serial
        self.running = False        # Flaga pracy wątku
        self.reader_thread = None   # Wątek odbioru

        # Aktualne wartości [X, Y, Z, E, A, S]
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Mutex do bezpiecznego dostępu do danych z różnych wątków
        self.lock = threading.Lock()
        
        # Zdarzenie do synchronizacji wysyłania komend (Handshake)
        self.cmd_ok_event = threading.Event()
        
        # Callback do GUI (aktualizacja wykresów i logów)
        self.gui_callback = None

        # WATCHDOG: Czas ostatniego poprawnego pakietu
        self.last_packet_time = 0.0

    def find_esp32_port(self):
        """Automatyczne wyszukiwanie portu z ESP32/CP210x/CH340."""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            # Szukamy typowych sterowników USB-UART
            if "CP210" in p.description or "CH340" in p.description or "USB Serial" in p.description: 
                return p.device
        return None

    def connect(self):
        """Nawiązuje połączenie i startuje wątek nasłuchujący."""
        if self.ser and self.ser.is_open:
            self.close()

        target_port = self.port_name or self.find_esp32_port()
        if target_port is None:
            return False, "Nie znaleziono urządzenia ESP-32"

        try:
            # Timeout=0.1s - krótki timeout, aby pętla szybko sprawdzała warunki
            self.ser = serial.Serial(target_port, self.baudrate, timeout=0.1)
            
            # Reset ESP32 przez DTR
            time.sleep(2) 
            
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # Resetujemy czas watchdoga na start
            self.last_packet_time = time.time()

            self.running = True
            self.reader_thread = threading.Thread(
                target=self._continuous_read,
                daemon=True
            )
            self.reader_thread.start()

            self.port_name = target_port
            return True, f"Połączono z {self.port_name}"

        except Exception as e:
            self.close()
            return False, f"Błąd połączenia: {e}"

    def _continuous_read(self):
        """
        Główna pętla odbiorcza. 
        Zawiera filtrację zakłóceń (EMI) i WATCHDOG danych.
        """
        AXIS_MAP = {"X": 0, "Y": 1, "Z": 2, "E": 3, "S": 4, "A": 5}
        
        failure_count = 0 
        MAX_FAILURES = 20  
        
        # WATCHDOG: Limit czasu bez danych (w sekundach)
        DATA_TIMEOUT = 2.0 

        # Odświeżamy czas na wejściu, żeby nie wyrzuciło błędu od razu
        self.last_packet_time = time.time()

        while self.running and self.ser and self.ser.is_open:
            try:
                # --- 1. SPRAWDZENIE WATCHDOGA (CZY DANE PŁYNĄ?) ---
                if time.time() - self.last_packet_time > DATA_TIMEOUT:
                    raise TimeoutError(f"Brak danych z ESP32 przez {DATA_TIMEOUT}s (Zwis?)")

                # --- 2. ODCZYT FIZYCZNY ---
                try:
                    raw_line = self.ser.readline()
                    # Sukces fizycznego odczytu (nawet pustego) zeruje licznik błędów I/O
                    failure_count = 0 
                except (serial.SerialException, OSError) as e:
                    failure_count += 1
                    if failure_count > MAX_FAILURES:
                        raise e 
                    time.sleep(0.05) 
                    continue
                except Exception:
                    continue 

                if not raw_line: 
                    continue # Pusta linia (timeout readline'a), wracamy do początku pętli (i sprawdzamy watchdoga)
                
                # Dekodowanie
                line = raw_line.decode("utf-8", errors="ignore").strip()
                if not line: continue

                # --- 3. OBSŁUGA LOGIKI ---
                
                # Wykrycie potwierdzenia komendy
                if line == "CMD_OK":
                    self.cmd_ok_event.set()
                    self.last_packet_time = time.time() # To też jest znak życia
                    continue

                # Wykrycie ramki danych telemetrycznych <...>
                if '<' in line and '>' in line:
                    try:
                        start = line.find('<')
                        end = line.find('>')
                        if end > start:
                            content = line[start+1 : end]
                            items = content.split(',')
                            updates = []
                            for item in items:
                                if len(item) >= 2:
                                    axis = item[0].upper()
                                    if axis in AXIS_MAP:
                                        try:
                                            val = float(item[1:])
                                            updates.append((AXIS_MAP[axis], val))
                                        except ValueError: pass
                            
                            if updates:
                                # WATCHDOG: Mamy poprawne dane, resetujemy timer
                                self.last_packet_time = time.time()

                                with self.lock:
                                    for idx, val in updates:
                                        self.current_angles[idx] = val
                                    data_copy = self.current_angles.copy()
                                
                                if self.gui_callback:
                                    self.gui_callback(data_copy)
                    except Exception: 
                        pass 

                # Inne komunikaty tekstowe
                else:
                    if self.gui_callback and len(line) > 1:
                        # Każdy tekst od ESP to znak życia
                        self.last_packet_time = time.time()
                        self.gui_callback(None, log_message=line)

            except Exception as e:
                # --- OBSŁUGA FAKTYCZNEGO ROZŁĄCZENIA ---
                self.running = False
                if self.gui_callback:
                    self.gui_callback(None, log_message=f"Błąd połączenia: {e}")
                    # WYMUSZENIE RECONNECTU W GUI
                    self.gui_callback(None, log_message="#AUTO_RECONNECT#")
                break

    def send_target_angles(self, th1, th2, th3, th4, th5):
        if not self.ser or not self.ser.is_open:
            return False, "Port nie jest otwarty"

        radians = [th1, th2, th3, th4, th5]
        degrees = [math.degrees(v) for v in radians]
        
        payload = (
            f"X{degrees[0]:.2f},"
            f"Y{degrees[1]:.2f},"
            f"Z{degrees[2]:.2f},"
            f"E{degrees[3]:.2f},"
            f"S{degrees[4]:.2f}"
        )
        
        checksum = 0
        for char in payload:
            checksum ^= ord(char)
            
        cmd = f"{payload}*{checksum:02X}\n"

        try:
            self.ser.write(cmd.encode("utf-8"))
            return True, f"Wysłano: {cmd.strip()}"
        except Exception as e:
            return False, f"Błąd wysyłania: {e}"

    def get_current_angles(self):
        with self.lock:
            return self.current_angles.copy()

    def close(self):
        self.running = False
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except: pass

# ---------- KINEMATYKA (Model matematyczny robota 5-DOF) -------------

class RobotKinematics:
    def __init__(self):
        self.d1 = 2.0 + 78.8 + 34.9
        self.a1 = 18.37
        self.a2 = 149.0
        self.a3 = 120.3
        self.a4 = 70.84
        self.d5 = 10.01
        self.d6 = 10.2 + 58.87

        range_const = math.pi / 2
        dR = math.pi / 180.0 # 1 stopień marginesu
        self.limits = {
            'th1': (-range_const - dR, range_const + dR),   # X
            'th2': (0 - dR, range_const * 1.5  + dR),       # Y/A
            'th3': (-range_const - dR, range_const + dR),   # Z
            'th4': (-range_const - dR, range_const + dR),   # E
            'th5': (-math.pi - dR, math.pi + dR)            # S
        }

        self.L1 = self.a2
        self.L2 = self.a3
        self.L3 = math.sqrt((self.a4 + self.d6)**2 + self.d5**2)
        
        self.L1_sq = self.L1**2
        self.L2_sq = self.L2**2
        self.denom = 2 * self.L1 * self.L2
        
        self.max_reach_sq = (self.L1 + self.L2)**2
        self.min_reach_sq = (self.L1 - self.L2)**2
        
        self.geo_phi_offset = math.atan2(self.d5, self.a4 + self.d6)

        self.radius = 16.0
        if self.a4 > 0:
            self.crit_axial_factor = 65.2 / self.a4
            self.crit_offset_factor = 29.2 / self.a4
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
        A1 = self._rot_z(th1)    @ self._trans_z(self.d1) @ self._trans_x(self.a1) @ self._rot_x(np.pi/2)
        A2 = self._rot_z(th2)                             @ self._trans_x(self.a2)
        A3 = self._rot_z(-th3)                            @ self._trans_x(self.a3)
        A4 = self._rot_z(-th4)                            @ self._trans_x(self.a4) @ self._rot_x(-np.pi / 2)
        A5 = self._rot_z(np.pi/2)@ self._trans_z(-self.d5)                         @ self._rot_x(np.pi / 2)
        A6 = self._rot_z(-th5)   @ self._trans_z(self.d6)

        T_base = np.eye(4)
        T1 = A1
        T2 = T1 @ A2
        T3 = T2 @ A3
        T4 = T3 @ A4
        T5 = T4 @ A5
        T_E = T5 @ A6

        return [T_base, T1, T2, T3, T4, T5, T_E]

    @staticmethod
    def _normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def get_joint_positions(self, th1, th2, th3, th4, th5=0.0):
        matrices = self._compute_chain(th1, th2, th3, th4, th5)
        return np.array([m[:3, 3] for m in matrices])

    def get_tcp_matrix(self, th1, th2, th3, th4, th5=0.0):
        chain = self._compute_chain(th1, th2, th3, th4, th5)
        return chain[-1]
    
    def get_jacobian(self, th1, th2, th3, th4, th5):
        matrices = self._compute_chain(th1, th2, th3, th4, th5)
        p_e = matrices[-1][:3, 3]
        J = np.zeros((6, 5))
        
        for i in range(5):
            T_prev = matrices[i]
            z_axis = T_prev[:3, 2]
            p_curr = T_prev[:3, 3]
            vec_diff = p_e - p_curr
            J[:3, i] = np.cross(z_axis, vec_diff)
            J[3:, i] = z_axis
        return J
    
    def inverse_kinematics(self, R, Z, th1, phi_deg=0.0, elbow_up=True, reverse_base=False):
        phi_rad = self._normalize_angle(math.radians(phi_deg + 270.0))
        phi_corr = phi_rad - self.geo_phi_offset
        
        R_ik = -R if reverse_base else R
        
        R_wrist = R_ik - self.a1 - self.L3 * math.cos(phi_corr)
        Z_wrist = Z - self.d1 - self.L3 * math.sin(phi_corr)
        
        D_sq = R_wrist**2 + Z_wrist**2
        if D_sq > self.max_reach_sq or D_sq < self.min_reach_sq:
            return None
        
        cos_th3 = (D_sq - self.L1_sq - self.L2_sq) / self.denom
        cos_th3 = np.clip(cos_th3, -1.0, 1.0)
        
        th3_ik = math.acos(cos_th3)
        if not elbow_up: th3_ik = -th3_ik
            
        alpha_angle = math.atan2(Z_wrist, R_wrist)
        beta_angle = math.atan2(self.L2 * math.sin(th3_ik), self.L1 + self.L2 * math.cos(th3_ik))
        
        th2 = alpha_angle - beta_angle
        th4_ik = phi_rad - th2 - th3_ik
        th3 = -th3_ik
        th4 = -th4_ik
        
        th1 = self._normalize_angle(th1)
        th2 = self._normalize_angle(th2)
        th3 = self._normalize_angle(th3)
        th4 = self._normalize_angle(th4)
        
        return (th1, th2, th3, th4)

    def check_constraints(self, angles, positions):
        if len(angles) == 5:
            th1, th2, th3, th4, th5 = angles
        else:
            th1, th2, th3, th4 = angles[:4]
            th5 = 0.0

        for key, val in zip(['th1', 'th2', 'th3', 'th4', 'th5'], [th1, th2, th3, th4, th5]):
            min_lim, max_lim = self.limits[key]
            if not (min_lim <= val <= max_lim):
                return False, f"Limit {key}"

        elbow_z = positions[3][2]
        if elbow_z - self.radius < 0:
            return False, f"Kolizja łokcia (Z={elbow_z:.1f})"

        tcp_z = positions[-1][2]
        if tcp_z < 0:
             return False, f"Efektor w ziemi (Z={tcp_z:.1f})"

        p3 = positions[3]
        p4 = positions[4]
        
        vec_z = p4[2] - p3[2]
        z_axial = p3[2] + vec_z * self.crit_axial_factor
        
        vec_x = p4[0] - p3[0]
        vec_y = p4[1] - p3[1]
        len_xy = math.sqrt(vec_x**2 + vec_y**2)
        
        z_critical = z_axial - (len_xy * self.crit_offset_factor)
        
        if z_critical < 0: 
            return False, f"Kolizja z bazą (Crit={z_critical:.1f})"

        return True, "OK"

    def calculate_joint_distance(self, q_current, q_target):
        if q_target is None: return float('inf')
        
        total_dist = 0
        weights = [1.0, 5.0, 3.0, 1.0, 1.0]
        
        for i in range(5):
            diff = q_target[i] - q_current[i]
            normalized = self._normalize_angle(diff)
            total_dist += abs(normalized) * weights[i]
            
        return total_dist

    def calculate_configuration_cost(self, angles, current_angles):
        th1, th2, th3, th4, th5 = angles
        
        limit_cost = 0
        vals = [th1, th2, th3, th4, th5]
        lims = [self.limits['th1'], self.limits['th2'], self.limits['th3'], self.limits['th4'], self.limits['th5']]
        
        for v, (mn, mx) in zip(vals, lims):
            norm = (v - mn) / (mx - mn)
            centered = (norm - 0.5) * 2.0
            barrier = centered ** 6
            limit_cost += barrier * 10.0
            
        motion_cost = 2.0 * self.calculate_joint_distance(current_angles, angles)
            
        J = self.get_jacobian(th1, th2, th3, th4, th5)
        J_pos = J[:3, :]
        manipulability = math.sqrt(np.linalg.det(J_pos @ J_pos.T))
        singularity_cost = 1.0 / (manipulability + 0.001)
        
        return limit_cost + motion_cost + singularity_cost

    def _construct_matrix(self, x, y, z, phi_deg):
        """
        Konstrukcja macierzy orientacji.
        ZMIANA: Definicja Pitch została obrócona tak, aby 0 stopni oznaczało poziom.
        """
        yaw = math.atan2(y, x)
        pitch = math.radians(phi_deg)
        
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        
        # Nowa definicja macierzy (0 = Poziomo, 90 = Pionowo góra)
        # Oś Z (Approach) to: [cy*cp, sy*cp, sp]
        # Oś X (Normal) to:   [cy*sp, sy*sp, -cp] (lub inna ortogonalna)
        
        # Poniżej macierz zgodna z definicją: Phi mierzone od poziomu
        R = np.array([
            [cy * sp, -sy, cy * cp],
            [sy * sp,  cy, sy * cp],
            [   -cp,    0,     sp]
        ])
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def _solve_from_matrix(self, T, current_angles, check_strategies=False):
        p_x, p_y, p_z = T[0, 3], T[1, 3], T[2, 3]
        
        # Wektor podejścia (Approach Vector - Oś Z efektora)
        a_x, a_y, a_z = T[0, 2], T[1, 2], T[2, 2]
        
        # Obliczenie kąta bazy (Yaw)
        th1_base_temp = math.atan2(p_y, p_x)
        cy, sy = math.cos(th1_base_temp), math.sin(th1_base_temp)
        
        # Rzutowanie wektora podejścia na płaszczyznę pionową ramienia (Radial, Z)
        # a_radial to składowa pozioma (przód/tył)
        a_radial = a_x * cy + a_y * sy
        
        # Obliczenie Phi (Pitch) z wektora [a_radial, a_z]
        # Zgodnie z nową definicją w _construct_matrix:
        # a_radial = cp (cosinus), a_z = sp (sinus)
        # tan(phi) = sin/cos = a_z / a_radial
        phi_deg = math.degrees(math.atan2(a_z, a_radial))
        
        R_target = math.sqrt(p_x**2 + p_y**2)
        th1_base = math.atan2(p_y, p_x)

        strategies = [(True, False), (False, False)]
        if R_target < 250 or p_z < 50:
            strategies.extend([(True, True), (False, True)])

        best_sol = None
        min_cost = float('inf')
        best_name = ""

        for elbow_up, reverse_base in strategies:
            th1_in = self._normalize_angle(th1_base + math.pi) if reverse_base else th1_base
            
            sol = self.inverse_kinematics(R_target, p_z, th1_in, phi_deg, elbow_up, reverse_base)
            
            if sol:
                fk_positions = self.get_joint_positions(*sol)
                is_valid, _ = self.check_constraints(sol, fk_positions)
                
                if is_valid:
                    if check_strategies:
                        return sol, "Auto"
                    
                    cost = self.calculate_configuration_cost(sol + (current_angles[4],), current_angles)
                    if cost < min_cost:
                        min_cost = cost
                        best_sol = sol
                        best_name = f"{'Elbow Up' if elbow_up else 'Elbow Down'}, {'Reverse' if reverse_base else 'Forward'}"

        return (best_sol, best_name) if best_sol else (None, None)

    def solve_ik(self, x, y, z, current_angles, phi_deg=None, roll_deg=0.0, local_search=False):
        th5_target = math.radians(roll_deg)

        if phi_deg is not None:
            T_goal = self._construct_matrix(x, y, z, phi_deg)
            sol, strategy = self._solve_from_matrix(T_goal, current_angles)
            return (sol + (th5_target,), strategy) if sol else (None, "Cel nieosiągalny")

        else:
            def evaluate_phi(phi_val):
                T_c = self._construct_matrix(x, y, z, phi_val)
                s_sol, s_name = self._solve_from_matrix(T_c, current_angles, check_strategies=True)
                if s_sol:
                    c = self.calculate_configuration_cost(s_sol + (th5_target,), current_angles)
                    return c, s_sol, s_name, phi_val
                return float('inf'), None, None, phi_val

            if local_search:
                if len(current_angles) >= 4:
                    curr_phi_rad = current_angles[1] + current_angles[2] + current_angles[3]
                    curr_phi_deg = math.degrees(curr_phi_rad)
                    curr_phi_deg = (curr_phi_deg + 180) % 360 - 180
                else:
                    curr_phi_deg = 0.0
                
                scan_range = 25
                start_scan = int(curr_phi_deg - scan_range)
                end_scan = int(curr_phi_deg + scan_range)
                step_val = 2
            else:
                start_scan = -175
                end_scan = 175
                step_val = 5

            best_phi_coarse = 0.0
            min_cost = float('inf')
            found_valid = False

            for phi in range(start_scan, end_scan, step_val):
                cost, _, _, _ = evaluate_phi(phi)
                if cost < min_cost:
                    min_cost = cost
                    best_phi_coarse = phi
                    found_valid = True

            if local_search and not found_valid:
                for phi in range(-175, 175, 5):
                    cost, _, _, _ = evaluate_phi(phi)
                    if cost < min_cost:
                        min_cost = cost
                        best_phi_coarse = phi
                        found_valid = True

            if not found_valid:
                return None, "Brak rozwiązania (zasięg/kolizja)"

            search_span = float(step_val) * 1.5 
            a = best_phi_coarse - search_span
            b = best_phi_coarse + search_span
            
            GR = 0.61803398875
            tol = 0.1
            
            c = b - (b - a) * GR
            d = a + (b - a) * GR
            
            while abs(b - a) > tol:
                cost_c, _, _, _ = evaluate_phi(c)
                cost_d, _, _, _ = evaluate_phi(d)
                
                if cost_c == float('inf') and cost_d == float('inf'):
                    mid = (a + b) / 2; a = mid - 0.5; b = mid + 0.5; break 

                if cost_c < cost_d:
                    b = d; d = c; c = b - (b - a) * GR
                else:
                    a = c; c = d; d = a + (b - a) * GR

            final_phi = (a + b) / 2
            final_cost, final_sol, final_name, final_phi_norm = evaluate_phi(final_phi)

            if final_sol:
                return final_sol + (th5_target,), f"Auto (φ={final_phi_norm:.1f}°)"
            elif found_valid:
                cost_rough, sol_rough, name_rough, phi_rough = evaluate_phi(best_phi_coarse)
                return sol_rough + (th5_target,), f"Auto-Coarse (φ={phi_rough:.1f}°)"
            
            return None, "Błąd optymalizacji"

    def generate_linear_path(self, start_cfg, end_cfg, step_mm=10.0):
        p0 = np.array([start_cfg['x'], start_cfg['y'], start_cfg['z']])
        p1 = np.array([end_cfg['x'], end_cfg['y'], end_cfg['z']])
        
        dist = np.linalg.norm(p1 - p0)
        if dist < 0.1: return [] 
        
        num_steps = int(np.ceil(dist / step_mm))
        if num_steps < 5: num_steps = 5
        
        steps = np.linspace(0, 1, num_steps + 1)
        
        trajectory_angles = []
        last_joints_deg = start_cfg['current_joints']
        
        print(f"[PATH] Generowanie: {num_steps} kroków. Start={p0}, End={p1}")

        for t in steps[1:]: 
            curr_pos = p0 + (p1 - p0) * t
            curr_roll = start_cfg['roll'] + (end_cfg['roll'] - start_cfg['roll']) * t
            
            last_joints_rad = [math.radians(a) for a in last_joints_deg[:5]]
            
            full_sol, strategy = self.solve_ik(
                curr_pos[0], curr_pos[1], curr_pos[2], 
                tuple(last_joints_rad), 
                phi_deg=None, 
                roll_deg=curr_roll,
                local_search=True
            )
            
            if full_sol is None:
                print(f"[IK FAIL] t={t:.2f} Pos={curr_pos}")
                return None, f"Błąd IK w t={t:.2f}"

            deg_sol = [math.degrees(v) for v in full_sol]
            
            max_diff = 0
            for i in range(5):
                diff = abs(deg_sol[i] - last_joints_deg[i])
                while diff > 180: diff = abs(diff - 360)
                if diff > max_diff: max_diff = diff
            
            if max_diff > 35.0:
                print(f"[PATH ERROR] Skok {max_diff:.1f}° w t={t:.2f}")
                return None, f"Gwałtowny skok ({max_diff:.1f}°). Przerwanie."

            trajectory_angles.append(deg_sol)
            last_joints_deg = deg_sol 
            
        return trajectory_angles, "OK"
    
# -------------------------------- GUI APP ---------------------------------

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
        self.max_reach = self.kin.a2 + self.kin.a3 + math.atan2(self.kin.d5, self.kin.a4 + self.kin.d6)
        self.control_mode = tk.StringVar(value='position')
        
        # Sterowanie kątami
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
        self.POSITION_TOLERANCE = 1.0  # mm

        # Inicjalizacja GUI
        self.setup_ui()
        self.setup_3d_plot()
        self.update_3d_visualization()
        
        # Próba automatycznego połączenia z robotem po starcie aplikacji
        self.root.after(100, self.connect_robot)

    def setup_ui(self):
        # Kolory
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
        # ================= LEWA KOLUMNA =================
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
            min_rad, max_rad = self.kin.limits[key]
            
            slider = tk.Scale(self.angle_control_frame, from_=math.degrees(min_rad  + math.pi/180), to=math.degrees(max_rad - math.pi/180),    
                              orient=tk.HORIZONTAL, resolution=0.01, length=200,
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
            ("Y:", "0", "y_entry"), 
            ("Z:", "3", "z_entry"), 
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
        
        move_btn_row = ttk.Frame(self.target_frame)
        move_btn_row.grid(row=6, column=0, columnspan=2, sticky="ew", pady=2)
        move_btn_row.columnconfigure((0, 1), weight=1, uniform="g")

        ttk.Button(move_btn_row, text="RUCH PTP", command=self.send_position).grid(row=0, column=0, padx=(0, 2), sticky="ew")
        ttk.Button(move_btn_row, text="RUCH LINIOWY", command=self.perform_linear_move).grid(row=0, column=1, padx=(2, 0), sticky="ew")
        ttk.Button(self.target_frame, text="DODAJ PUNKT DO LISTY", command=self.add_point_to_sequence).grid(row=7, column=0, columnspan=2, sticky="ew", pady=(5, 0))

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
        # ================= PRAWA KOLUMNA ===============
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
            
            # Strzałki osi TCP
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
        def _update_ui():
            # --- 1. Obsługa Logów i Statusów ---
            if log_message:
                if log_message == "#AUTO_RECONNECT#":
                    self.status_label.config(text="Rozłączony (Próba wznowienia...)", foreground="orange")
                    self.log("Utracono połączenie. Próba wznowienia za 3s...")
                    self.is_connected = False
                    # Uruchomienie timera, który spróbuje połączyć się ponownie
                    self.root.after(3000, self.auto_reconnect_attempt)
                    return # Ważne: przerywamy, aby nie przetwarzać dalej
                
                elif log_message == "#CONNECTION_LOST#":
                     self.status_label.config(text="Błąd krytyczny", foreground="red")
                     self.is_connected = False
                
                else:
                    self.log(log_message)
            
            # --- 2. Aktualizacja Kątów i Pozycji TCP ---
            if angles:
                # Aktualizacja etykiet kątów (Joints)
                for i, axis in enumerate(['X', 'Y', 'Z', 'E', 'S', 'A']):
                    if i < len(angles):
                        self.pos_labels[axis].config(text=f"{angles[i]:.2f}°")
                
                # Obliczanie pozycji TCP (Forward Kinematics) dla wyświetlania XYZ
                try:
                    current_rads = [math.radians(a) for a in angles[:5]]
                    tcp_matrix = self.kin.get_tcp_matrix(*current_rads)
                    current_xyz = tcp_matrix[:3, 3]
                    
                    self.last_tcp_pos = current_xyz # Zapamiętanie do logiki ruchu
                    
                    for i, axis in enumerate(['TCP_X', 'TCP_Y', 'TCP_Z']):
                        self.tcp_labels[axis].config(text=f"{current_xyz[i]:.2f}")
                except Exception:
                    pass # Ignorujemy błędy matematyczne przy niepełnych danych

        # --- 3. Throttling (Limitowanie odświeżania GUI) ---
        # Logi i statusy puszczamy natychmiast, ale dane telemetryczne (angles)
        # odświeżamy nie częściej niż co 200ms, aby nie zamulić interfejsu.
        now = time.time() * 1000.0
        if log_message or (now - self.last_gui_update_time >= 200.0):
            self.last_gui_update_time = now
            self.root.after(0, _update_ui)

    def auto_reconnect_attempt(self):
        """Funkcja wywoływana przez timer do próby ponownego połączenia."""
        # Jeśli w międzyczasie połączono ręcznie, przerywamy pętlę
        if self.is_connected: 
            return 
        
        self.log("Automatyczne wznawianie połączenia...")
        
        # Próba fizycznego połączenia
        success, msg = self.robot.connect()
        
        if success:
            self.status_label.config(text="Połączono (Wznowiono)", foreground="green")
            self.log(f"SUKCES: Połączenie przywrócone ({msg}).")
            self.is_connected = True
            
            # Opcjonalnie: Jeśli byłeś w trakcie sekwencji, tu można by ją wznowić
        else:
            self.log(f"Niepowodzenie: {msg}. Ponowna próba!")
            self.root.after(500, self.auto_reconnect_attempt)

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

            sol, info_msg = self.kin.solve_ik(x, y, z, tuple(cur_rads), phi_deg=phi, roll_deg=roll)
            
            if sol is None:
                error_text = info_msg if info_msg else "Nieznany błąd IK"
                messagebox.showerror("Błąd IK", f"Nie znaleziono rozwiązania:\n{error_text}")
                self.log(f"Błąd IK: {error_text}")
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
                self.log(f"Wysłano konfigurację: {info_msg}")
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

    def perform_linear_move(self):
        if not self.is_connected:
            messagebox.showwarning("Info", "Brak połączenia")
            return

        try:
            # --- 1. Pobranie danych wejściowych ---
            target_x = float(self.x_entry.get())
            target_y = float(self.y_entry.get())
            target_z = float(self.z_entry.get())
            target_roll = float(self.roll_entry.get())

            # --- 2. Pobranie stanu początkowego ---
            start_x, start_y, start_z = self.last_tcp_pos
            current_joints = self.robot.get_current_angles()
            
            # Obliczenie startowego Phi (suma kątów th2+th3+th4) dla płynnego startu
            # Zakładamy, że current_joints są w stopniach
            start_phi = current_joints[1] + current_joints[2] + current_joints[3]
            start_roll = current_joints[4]

            # Jeśli auto-phi włączone, celujemy w utrzymanie obecnego kąta lub 0
            if self.auto_phi_var.get():
                target_phi = start_phi 
            else:
                target_phi = float(self.phi_entry.get())

            # --- 3. Konfiguracja generatora ---
            start_cfg = {
                'x': start_x, 'y': start_y, 'z': start_z, 
                'phi': start_phi, 'roll': start_roll, 'current_joints': current_joints
            }
            end_cfg = {
                'x': target_x, 'y': target_y, 'z': target_z, 
                'phi': target_phi, 'roll': target_roll
            }

            self.log(f"Generowanie trasy: Phi {start_phi:.1f}° -> {target_phi:.1f}°")
            
            # Generujemy ścieżkę (krok 5mm dla stabilności)
            path, msg = self.kin.generate_linear_path(start_cfg, end_cfg, step_mm=5.0)
            
            if path is None:
                messagebox.showerror("Błąd", f"Nie można wyznaczyć trasy:\n{msg}")
                return
            
            self.log(f"Trasa wyznaczona: {len(path)} punktów. Rozpoczynam ruch sekwencyjny...")
            
            # --- 4. Uruchomienie sekwencji (Stop-and-Go) ---
            self.linear_path_queue = path
            self.linear_path_index = 0
            self.execute_linear_step()

        except ValueError:
            messagebox.showerror("Błąd", "Złe dane wejściowe")

    def execute_linear_step(self):
        """Wysyła jeden punkt i uruchamia oczekiwanie."""
        if self.linear_path_index >= len(self.linear_path_queue):
            self.log("Ruch liniowy zakończony.")
            return

        # 1. Wyczyszczenie flagi potwierdzenia
        self.robot.cmd_ok_event.clear()

        # 2. Pobranie punktu
        target_angles = self.linear_path_queue[self.linear_path_index]
        rads = [math.radians(a) for a in target_angles]
        
        # 3. Obliczenie fizycznej pozycji celu (dla warunku nr 2)
        # Musimy wiedzieć gdzie (XYZ) robot ma dojechać w tym kroku
        tcp_matrix = self.kin.get_tcp_matrix(*rads)
        self.current_step_target_xyz = tcp_matrix[:3, 3] 

        # 4. Wysłanie do robota
        self.robot.send_target_angles(*rads)
        
        # 5. Start oczekiwania
        self.root.after(1, self.wait_for_ack_and_motion)

    def wait_for_ack_and_motion(self):
        """
        Czeka na DWIE rzeczy:
        1. Potwierdzenie CMD_OK od ESP (że ramka dotarła).
        2. Zbliżenie się fizyczne do celu (że silniki dojechały).
        """
        # --- WARUNEK 1: Czy ESP potwierdziło odbiór? ---
        if not self.robot.cmd_ok_event.is_set():
            # Jeszcze nie ma ACK -> czekaj
            self.root.after(5, self.wait_for_ack_and_motion)
            return

        # --- WARUNEK 2: Czy robot fizycznie dojechał? ---
        # Sprawdzamy dystans między aktualną pozycją (z telemetry) a celem kroku
        dist = np.linalg.norm(self.last_tcp_pos - self.current_step_target_xyz)
        
        # Tolerancja np. 2.0 mm (musi być mniejsza niż Twój krok generatora czyli 5mm)
        if dist > 2.0:
            # Robot wciąż jedzie -> czekaj
            self.root.after(10, self.wait_for_ack_and_motion)
            return

        # --- SUKCES: Otrzymano ACK i Robot jest na miejscu ---
        self.linear_path_index += 1
        
        if self.linear_path_index % 5 == 0:
            self.log(f"Wykonano krok {self.linear_path_index}/{len(self.linear_path_queue)}")
        
        # Wyślij kolejny punkt
        self.root.after(5, self.execute_linear_step)

    def wait_for_linear_step(self):
        """Sprawdza czy robot dojechał do celu pośredniego."""
        # Oblicz dystans między aktualną pozycją robota (z enkoderów) a celem kroku
        dist = np.linalg.norm(self.last_tcp_pos - self.current_step_target_xyz)

        # Tolerancja np. 1mm
        if dist < 2.0:
            # Dojechał -> następny krok
            self.linear_path_index += 1
            # Log co 5 kroków, żeby nie śmiecić
            if self.linear_path_index % 5 == 0:
                self.log(f"Krok {self.linear_path_index}/{len(self.linear_path_queue)}")
            
            self.execute_linear_step()
        else:
            # Nie dojechał -> sprawdź ponownie za 20ms
            self.root.after(20, self.wait_for_linear_step)

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
        try:
            x, y, z = float(pt['X']), float(pt['Y']), float(pt['Z'])
        except ValueError:
            self.stop_sequence("Błąd danych: Nieprawidłowy format współrzędnych X/Y/Z")
            return

        self.log(f"Krok {self.current_sequence_index+1}: {x, y, z}")
        self.target_xyz = np.array([x, y, z])
        self._update_sequence_display(self.current_sequence_index)
        
        try:
            if pt.get('Joints'):
                raw_joints = pt['Joints']
                if len(raw_joints) == 4: raw_joints.append(0.0)
                rads = [math.radians(d) for d in raw_joints]
                self.robot.send_target_angles(*rads)
            else:
                cur = [math.radians(a) for a in self.robot.get_current_angles()[:5]]
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
            sol, info_msg = self.kin.solve_ik(x, y, z, tuple(cur), phi_deg=phi, roll_deg=roll)
            
            if sol:
                self.sequence_data.append({"X":x, "Y":y, "Z":z, "Fi":phi if phi else 0, "Auto_Fi":auto,"Rol": roll,})
                self._update_sequence_display()
                self.log(f"Dodano punkt: {info_msg}")
            else:
                messagebox.showerror("Błąd", f"Pozycja nieosiągalna:\n{info_msg}")
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