import sympy as sp
import pandas as pd
import numpy as np
import os
import math

# =====================================================================
# CZĘŚĆ 1: DEFINICJA KINEMATYKI I PARAMETRÓW D-H
# =====================================================================

# --- Stałe D-H (parametry robota 5-DOF) ---
# Z diagramu:
l1_val = 18.4               # λ1 (offset w X dla frame 1)
l2_val = 149.0              # L2 (długość ramienia 2)
l3_val = 120.3              # L3 (długość ramienia 3)
l4_val = 87.8               # L4 (długość ramienia 4)
l5_val = 23.0               # L5 (długość ramienia 5)
lambda1_val = 110.8 # 78.8 + 34.9   # λ1 (wysokość podstawy)
lambda5_val = 10.0          # λ5 (offset końcówki w Z)

# Definicja symboli
th1, th2, th3, th4, alpha5 = sp.symbols('θ1 θ2 θ3 θ4 α5')
pi = sp.pi

# --- Funkcje pomocnicze dla macierzy transformacji ---
def RotZ(theta):
    c, s = sp.cos(theta), sp.sin(theta)
    return sp.Matrix([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def RotX(alpha):
    c, s = sp.cos(alpha), sp.sin(alpha)
    return sp.Matrix([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def TransZ(d):
    return sp.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])

def TransX(a):
    return sp.Matrix([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

# =====================================================================
# CZĘŚĆ 2: KINEMATYKA PROSTA (Forward Kinematics)
# =====================================================================

def verify_dh_parameters():
    """
    Wyświetla i weryfikuje parametry D-H z diagramu.
    """
    print(f"\n{'='*60}")
    print("WERYFIKACJA PARAMETRÓW D-H")
    print(f"{'='*60}")
    print("\nTabela parametrów Denavita-Hartenberga:")
    print(f"{'Joint':<8} {'θ_i':<12} {'d_i':<12} {'a_i':<12} {'α_i':<12}")
    print("-" * 60)
    print(f"{'1':<8} {'θ1':<12} {f'{lambda1_val:.3f}':<12} {f'{l1_val:.3f}':<12} {'π/2':<12}")
    print(f"{'2':<8} {'θ2':<12} {'0':<12} {f'{l2_val:.3f}':<12} {'0':<12}")
    print(f"{'3':<8} {'-θ3':<12} {'0':<12} {f'{l3_val:.3f}':<12} {'0':<12}")
    print(f"{'4':<8} {'-θ4':<12} {'0':<12} {f'{l4_val:.3f}':<12} {'-π/2':<12}")
    print(f"{'5':<8} {'0':<12} {f'{lambda5_val:.3f}':<12} {f'{l5_val:.3f}':<12} {'α5':<12}")
    print("-" * 60)
    
    # Geometria końcówki (l5, λ5 tworzą trójkąt prostokątny)
    L3_horizontal = l4_val + l5_val
    L3_wypadkowa = math.sqrt(L3_horizontal**2 + lambda5_val**2)
    gamma_deg = math.degrees(math.atan2(lambda5_val, L3_horizontal))
    
    print(f"\nGeometria końcówki robota:")
    print(f"  l4 + l5 (horizontal): {L3_horizontal:.3f} mm")
    print(f"  λ5 (vertical):        {lambda5_val:.3f} mm")
    print(f"  L3 (wypadkowa):       {L3_wypadkowa:.3f} mm")
    print(f"  γ (kąt offsetu):      {gamma_deg:.3f}°")
    print(f"\n  Relacja: L3² = (l4+l5)² + λ5²")
    print(f"           {L3_wypadkowa:.3f}² = {L3_horizontal:.3f}² + {lambda5_val:.3f}²")
    print(f"           {L3_wypadkowa**2:.2f} = {L3_horizontal**2:.2f} + {lambda5_val**2:.2f}")
    
    # Oblicz maksymalny zasięg
    max_reach = l1_val + l2_val + l3_val + L3_wypadkowa
    print(f"\nMaksymalny zasięg teoretyczny: {max_reach:.2f} mm")
    print(f"Wysokość podstawy (λ1): {lambda1_val:.2f} mm")
    print(f"{'='*60}\n")

def forward_kinematics(th1_val, th2_val, th3_val, th4_val, alpha5_val=0.0):
    """
    Oblicza kinematykę prostą dla robota 5-DOF zgodnie z parametrami D-H.
    
    Z tabeli D-H:
    Joint 1: θ1, λ1,      l1,  π/2
    Joint 2: θ2, 0,       L2,  0
    Joint 3: -θ3, 0,      L3,  0
    Joint 4: -θ4, 0,      L4,  -π/2
    Joint 5: 0,   λ5,     L5,  α5
    
    Zwraca:
        Macierz 4x4 reprezentującą pozycję i orientację końcówki robota
    """
    alpha1_val = np.pi / 2    # Z tabeli D-H
    alpha4_val = -np.pi / 2   # Z tabeli D-H
    
    # Macierze transformacji D-H zgodne z tabelą
    # A_i = RotZ(θ_i) * TransZ(d_i) * TransX(a_i) * RotX(α_i)
    
    A1 = RotZ(th1_val) * TransZ(lambda1_val) * TransX(l1_val) * RotX(alpha1_val)
    A2 = RotZ(th2_val) * TransX(l2_val)  # d=0, α=0
    A3 = RotZ(-th3_val) * TransX(l3_val)  # Uwaga: -θ3, d=0, α=0
    A4 = RotZ(-th4_val) * TransX(l4_val) * RotX(alpha4_val)  # Uwaga: -θ4, d=0
    A5 = TransZ(lambda5_val) * TransX(l5_val) * RotX(alpha5_val)  # θ=0
    
    # Wynikowa macierz transformacji
    A_total = A1 * A2 * A3 * A4 * A5
    
    # Konwersja do wartości numerycznych
    A_num = np.array(A_total.evalf().tolist()).astype(float)
    
    return A_num

def forward_kinematics_detailed(th1_val, th2_val, th3_val, th4_val, alpha5_val=0.0):
    """
    Oblicza kinematykę prostą z wyświetlaniem poszczególnych kroków.
    """
    alpha1_val = np.pi / 2
    alpha4_val = -np.pi / 2
    
    print(f"\n--- Forward Kinematics - Szczegóły ---")
    print(f"Kąty wejściowe (stopnie):")
    print(f"  θ1={math.degrees(th1_val):.2f}°, θ2={math.degrees(th2_val):.2f}°, " +
          f"θ3={math.degrees(th3_val):.2f}°, θ4={math.degrees(th4_val):.2f}°")
    
    A1 = RotZ(th1_val) * TransZ(lambda1_val) * TransX(l1_val) * RotX(alpha1_val)
    A1_num = np.array(A1.evalf().tolist()).astype(float)
    print(f"\nA1 (po θ1): pos = [{A1_num[0,3]:.2f}, {A1_num[1,3]:.2f}, {A1_num[2,3]:.2f}]")
    
    A12 = A1 * RotZ(th2_val) * TransX(l2_val)
    A12_num = np.array(A12.evalf().tolist()).astype(float)
    print(f"A1*A2 (po θ2): pos = [{A12_num[0,3]:.2f}, {A12_num[1,3]:.2f}, {A12_num[2,3]:.2f}]")
    
    A123 = A12 * RotZ(-th3_val) * TransX(l3_val)
    A123_num = np.array(A123.evalf().tolist()).astype(float)
    print(f"A1*A2*A3 (po θ3): pos = [{A123_num[0,3]:.2f}, {A123_num[1,3]:.2f}, {A123_num[2,3]:.2f}]")
    
    A1234 = A123 * RotZ(-th4_val) * TransX(l4_val) * RotX(alpha4_val)
    A1234_num = np.array(A1234.evalf().tolist()).astype(float)
    print(f"A1*A2*A3*A4 (po θ4): pos = [{A1234_num[0,3]:.2f}, {A1234_num[1,3]:.2f}, {A1234_num[2,3]:.2f}]")
    
    A_total = A1234 * TransZ(lambda5_val) * TransX(l5_val) * RotX(alpha5_val)
    A_num = np.array(A_total.evalf().tolist()).astype(float)
    print(f"A1*...*A5 (końcowa): pos = [{A_num[0,3]:.2f}, {A_num[1,3]:.2f}, {A_num[2,3]:.2f}]")
    
    return A_num

# =====================================================================
# CZĘŚĆ 3: KINEMATYKA ODWROTNA (Inverse Kinematics)
# =====================================================================

def inverse_kinematics(x_target, y_target, z_target, phi_deg=0.0, elbow_up=True):
    print(f"\n{'='*60}")
    print(f"KINEMATYKA ODWROTNA - Obliczanie kątów")
    print(f"{'='*60}")
    print(f"Cel: X={x_target:.2f}, Y={y_target:.2f}, Z={z_target:.2f} mm")
    print(f"Orientacja φ={phi_deg:.2f}°")
    
    # --- KROK 1: θ1 (obrót podstawy) ---
    if math.sqrt(x_target**2 + y_target**2) < 0.01:
        th1 = 0.0 
    else:
        th1 = math.atan2(y_target, x_target)
    
    print(f"\n[Krok 1] θ1 = {math.degrees(th1):.2f}°")
    
    # --- KROK 2: Przejście do płaszczyzny R-Z ---
    R = math.sqrt(x_target**2 + y_target**2)
    Z = z_target
    print(f"[Krok 2] Rzut R-Z: R={R:.2f}, Z={Z:.2f}")
    
    # --- KROK 3: Parametry dla IK 3-DOF (tylko θ2, θ3, θ4) ---
    L1 = l2_val  # 149.006
    L2 = l3_val  # 120.3
    L3 = math.sqrt((l4_val + l5_val)**2 + lambda5_val**2) # 111.3
    
    print(f"[Krok 3] Łańcuch 3-DOF: L1={L1:.1f}, L2={L2:.1f}, L3={L3:.1f}")
    
    # Pozycja "nadgarstka" (punkt przed l4)
    phi_rad = math.radians(phi_deg)
    phi_corr = phi_rad + math.atan2(lambda5_val, l4_val + l5_val)
    print(f"[Krok 3] phi_rad={math.degrees(phi_rad):.2f}, phi_corr={math.degrees(phi_corr):.2f}")

    R_wrist = R - l1_val - L3 * math.cos(phi_corr)
    Z_wrist = Z - lambda1_val - L3 * math.sin(phi_corr)
    
    print(f"[Krok 3] Nadgarstek: R_w={R_wrist:.2f}, Z_w={Z_wrist:.2f}")

    
    # --- KROK 4: Rozwiązanie dla θ2, θ3 ---
    D = math.sqrt(R_wrist**2 + Z_wrist**2)
    print(f"[Krok 4] Odległość do nadgarstka D={D:.2f} mm")
    
    if D > (L1 + L2) or D < abs(L1 - L2):
        print(f"[BŁĄD] Nieosiągalne! Wymagane: {abs(L1-L2):.1f} <= D <= {L1+L2:.1f}")
        return None
    
    cos_th3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_th3 = np.clip(cos_th3, -1.0, 1.0)
    
    th3_ik = math.acos(cos_th3) if elbow_up else -math.acos(cos_th3)
    
    alpha = math.atan2(Z_wrist, R_wrist)
    beta = math.atan2(L2 * math.sin(th3_ik), L1 + L2 * math.cos(th3_ik))
    th2 = alpha - beta
    
    print(f"[Krok 4] θ2={math.degrees(th2):.2f}°, θ3_ik={math.degrees(th3_ik):.2f}°")
    
    # --- KROK 5: θ4 ---
    th4_ik = phi_rad - th2 - th3_ik
    print(f"[Krok 5] θ4_ik={math.degrees(th4_ik):.2f}°")
    
    # --- KROK 6: Mapowanie D-H ---
    th3 = -th3_ik
    th4 = -th4_ik
    
    print(f"\n[Krok 6] Kąty D-H:")
    print(f"  θ1={math.degrees(th1):7.2f}°, θ2={math.degrees(th2):7.2f}°")
    print(f"  θ3={math.degrees(th3):7.2f}°, θ4={math.degrees(th4):7.2f}°")
    
    # Normalizacja
    th1 = math.atan2(math.sin(th1), math.cos(th1))
    th2 = math.atan2(math.sin(th2), math.cos(th2))
    th3 = math.atan2(math.sin(th3), math.cos(th3))
    th4 = math.atan2(math.sin(th4), math.cos(th4))
    
    return (th1, th2, th3, th4)

# =====================================================================
# CZĘŚĆ 4: EKSPORT DO EXCELA
# =====================================================================

def export_to_excel(th1_val, th2_val, th3_val, th4_val, alpha5_val=0.0):
    """
    Eksportuje macierze transformacji D-H do pliku Excel.
    """
    filename = "macierze_numeryczne_Ai.xlsx"
    
    alpha1_val = np.pi / 2
    alpha4_val = -np.pi / 2
    
    # Definicja macierzy symbolicznych
    A1 = RotZ(th1) * TransZ(lambda1_val) * TransX(l1_val) * RotX(alpha1_val)
    A2 = RotZ(th2) * TransX(l2_val)
    A3 = RotZ(-th3) * TransX(l3_val)
    A4 = RotZ(-th4) * TransX(l4_val) * RotX(alpha4_val)
    A5 = TransZ(lambda5_val) * TransX(l5_val) * RotX(alpha5)
    A_total = A1 * A2 * A3 * A4 * A5
    
    # Podstawienie wartości
    subs_dict = {th1: th1_val, th2: th2_val, th3: th3_val, 
                 th4: th4_val, alpha5: alpha5_val}
    
    matrices = {
        'A1': A1.subs(subs_dict).evalf(),
        'A2': A2.subs(subs_dict).evalf(),
        'A3': A3.subs(subs_dict).evalf(),
        'A4': A4.subs(subs_dict).evalf(),
        'A5': A5.subs(subs_dict).evalf(),
        'A_total': A_total.subs(subs_dict).evalf()
    }
    
    try:
        with pd.ExcelWriter(filename, engine='openpyxl') as writer:
            for sheet_name, matrix in matrices.items():
                matrix_list = [[float(v) for v in row] for row in matrix.tolist()]
                df = pd.DataFrame(matrix_list)
                df.to_excel(writer, sheet_name=sheet_name, header=False, index=False)
        
        print(f"\n✓ Plik '{filename}' zapisany pomyślnie.")
        return np.array(matrices['A_total'].tolist()).astype(float)
    
    except Exception as e:
        print(f"\n✗ Błąd zapisu: {e}")
        return None

# =====================================================================
# CZĘŚĆ 5: GŁÓWNA FUNKCJA
# =====================================================================

def main():
    """
    Główna funkcja programu - oblicza IK i weryfikuje wynik.
    """
    
    print("\n" + "="*60)
    print("SOLVER KINEMATYKI ODWROTNEJ - ROBOT 5-DOF")
    print("="*60)
    
    # Wyświetl parametry D-H
    verify_dh_parameters()
    
    # --- WPROWADZENIE DANYCH ---
    print("\nPodaj współrzędne docelowe:")
    
    try:
        x_target = float(input("  X [mm]: "))
        y_target = float(input("  Y [mm]: "))
        z_target = float(input("  Z [mm]: "))
        
        # Opcjonalnie: orientacja
        use_orientation = input("\nCzy określić orientację końcówki? (t/n): ").lower()
        if use_orientation == 't':
            phi_deg = float(input("  Orientacja φ [stopnie]: "))
        else:
            phi_deg = 0.0
            
        # Opcjonalnie: konfiguracja łokcia
        elbow_choice = input("\nKonfiguracja łokcia - góra/dół? (g/d): ").lower()
        elbow_up = (elbow_choice != 'd')
        
    except ValueError:
        print("\n✗ Błąd: Wprowadź poprawne wartości liczbowe!")
        return
    
    # --- OBLICZANIE KINEMATYKI ODWROTNEJ ---
    result = inverse_kinematics(x_target, y_target, z_target, phi_deg, elbow_up)
    
    if result is None:
        print("\n✗ Nie udało się znaleźć rozwiązania IK.")
        return
    
    th1, th2, th3, th4 = result
    alpha5 = 0.0  # Brak obrotu wokół osi końcówki
    
    # --- WERYFIKACJA ---
    print(f"\n{'='*60}")
    print("WERYFIKACJA - Kinematyka Prosta")
    print(f"{'='*60}")
    
    # Opcja szczegółowej weryfikacji
    detailed = input("\nPokazać szczegółowe kroki FK? (t/n): ").lower()
    
    if detailed == 't':
        A_calculated = forward_kinematics_detailed(th1, th2, th3, th4, alpha5)
    else:
        A_calculated = forward_kinematics(th1, th2, th3, th4, alpha5)
    
    print("\nMacierz transformacji (obliczona z FK):")
    print(np.array2string(A_calculated, precision=3, suppress_small=True))
    
    # Pozycja obliczona
    x_calc, y_calc, z_calc = A_calculated[0, 3], A_calculated[1, 3], A_calculated[2, 3]
    
    print(f"\nPozycja docelowa:   X={x_target:8.2f}, Y={y_target:8.2f}, Z={z_target:8.2f}")
    print(f"Pozycja obliczona:  X={x_calc:8.2f}, Y={y_calc:8.2f}, Z={z_calc:8.2f}")
    
    # Błąd pozycji
    error_x = abs(x_target - x_calc)
    error_y = abs(y_target - y_calc)
    error_z = abs(z_target - z_calc)
    error_total = math.sqrt(error_x**2 + error_y**2 + error_z**2)
    
    print(f"\nBłąd pozycji:       ΔX={error_x:8.3f}, ΔY={error_y:8.3f}, ΔZ={error_z:8.3f}")
    print(f"Błąd całkowity:     {error_total:.3f} mm")
    
    # Dodatkowa analiza błędu
    print(f"\n--- Analiza błędu ---")
    print(f"Błąd w λ5 (offset Z): {abs(z_calc - z_target):.3f} mm")
    print(f"Błąd w płaszczyźnie XY: {math.sqrt(error_x**2 + error_y**2):.3f} mm")
    
    if error_total < 0.1:
        print("\n✓ Weryfikacja POZYTYWNA (błąd < 0.1 mm)")
    elif error_total < 1.0:
        print("\n✓ Weryfikacja DOBRA (błąd < 1 mm)")
    elif error_total < 10.0:
        print("\n⚠ Weryfikacja CZĘŚCIOWA (błąd < 10 mm)")
    else:
        print("\n✗ Weryfikacja NEGATYWNA (duży błąd)")
        print("   Sprawdź parametry D-H i logikę offsetów!")
    
    # --- EKSPORT DO EXCELA ---
    export_choice = input("\nCzy wyeksportować macierze do Excel? (t/n): ").lower()
    if export_choice == 't':
        export_to_excel(th1, th2, th3, th4, alpha5)
    
    print(f"\n{'='*60}")
    print("Koniec obliczeń")
    print(f"{'='*60}\n")

# =====================================================================
# URUCHOMIENIE PROGRAMU
# =====================================================================

if __name__ == "__main__":
    main()