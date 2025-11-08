import math

# --- Stałe (długości członów w mm) ---
# Na podstawie Pana schematu
L1 = 149.0  # Długość od osi theta2 do osi theta3
L2 = 120.3  # Długość od osi theta3 do osi theta4
L3 = 188.1  # Długość od osi theta4 do końcówki

def calculate_ik(x, y, phi_rad):
    """
    Oblicza kinematykę odwrotną dla manipulatora 3-DOF (RRR).

    Argumenty:
        x (float): Docelowa pozycja X efektora.
        y (float): Docelowa pozycja Y efektora.
        phi_rad (float): Docelowa orientacja efektora (kąt absolutny) W RADIANACH.

    Zwraca:
        Lista tupli: [(theta2, theta3, theta4), ...], gdzie każda tupla to jedno
        ważne rozwiązanie (zazwyczaj 0, 1 lub 2 rozwiązania).
        Kąty są W RADIANACH.
        Zwraca pustą listę [], jeśli cel jest nieosiągalny.
    """

    # --- Krok 1: Obliczenie pozycji "nadgarstka" W(xw, yw) ---
    # W to punkt osi obrotu theta4
    xw = x - L3 * math.cos(phi_rad)
    yw = y - L3 * math.sin(phi_rad)

    # --- Krok 2: Obliczenie kąta theta3 (łokieć) ---

    # Kwadrat odległości od bazy (0,0) do nadgarstka W
    r_squared = xw**2 + yw**2
    r = math.sqrt(r_squared)

    # Sprawdzenie osiągalności (czy W jest w zasięgu ramion L1+L2)
    if r > (L1 + L2) or r < abs(L1 - L2):
        # Cel W jest nieosiągalny (zbyt daleko lub zbyt blisko)
        return []
        
    # Twierdzenie cosinusów dla trójkąta (Baza, Łokieć, Nadgarstek W)
    # cos(theta3) = (r^2 - L1^2 - L2^2) / (2 * L1 * L2)
    cos_theta3 = (r_squared - L1**2 - L2**2) / (2 * L1 * L2)

    # Zabezpieczenie przed błędami precyzji zmiennoprzecinkowej (np. 1.000000001)
    if cos_theta3 > 1.0:
        cos_theta3 = 1.0
    elif cos_theta3 < -1.0:
        cos_theta3 = -1.0

    # Dwa rozwiązania dla theta3 (łokieć w górę / w dół)
    # theta3 = acos(cos_theta3)
    theta3_up = math.atan2(math.sqrt(1 - cos_theta3**2), cos_theta3)
    theta3_down = math.atan2(-math.sqrt(1 - cos_theta3**2), cos_theta3)

    solutions = []
    
    # Przetwarzamy oba przypadki (up i down)
    for theta3 in [theta3_up, theta3_down]:
        
        # --- Krok 3: Obliczenie kąta theta2 (baza) ---
        # theta2 = atan2(yw, xw) - atan2(k2, k1)
        k1 = L1 + L2 * math.cos(theta3)
        k2 = L2 * math.sin(theta3)
        
        theta2 = math.atan2(yw, xw) - math.atan2(k2, k1)

        # --- Krok 4: Obliczenie kąta theta4 (nadgarstek) ---
        # phi = theta2 + theta3 + theta4  =>  theta4 = phi - theta2 - theta3
        theta4 = phi_rad - theta2 - theta3

        # Normalizacja kątów do zakresu [-pi, pi] (dobra praktyka)
        theta2 = math.atan2(math.sin(theta2), math.cos(theta2))
        theta3 = math.atan2(math.sin(theta3), math.cos(theta3))
        theta4 = math.atan2(math.sin(theta4), math.cos(theta4))
        
        solution_tuple = (theta2, theta3, theta4)
        
        # Unikaj duplikatów, jeśli theta3_up == theta3_down (osobliwość)
        if solution_tuple not in solutions:
            solutions.append(solution_tuple)
            
    return solutions

# ---
# === PRZYKŁAD UŻYCIA ===
# ---
if __name__ == "__main__":
    
    # --- Cel (współrzędne i orientacja) ---
    # Załóżmy, że chcemy, aby końcówka była w (x=250, y=100)
    # i była skierowana poziomo (phi = 0 stopni)
    
    TARGET_X = 250.0
    TARGET_Y = 100.0
    TARGET_PHI_DEGREES = 0.0
    
    # Konwersja phi na radiany dla funkcji
    target_phi_radians = math.radians(TARGET_PHI_DEGREES)
    
    # Obliczenie
    solutions = calculate_ik(TARGET_X, TARGET_Y, target_phi_radians)
    
    # Prezentacja wyników
    print(f"--- Kinematyka Odwrotna ---")
    print(f"Cel: X={TARGET_X}, Y={TARGET_Y}, Phi={TARGET_PHI_DEGREES}°")
    print(f"Stałe: L1={L1}, L2={L2}, L3={L3}")
    print("-------------------------------")
    
    if not solutions:
        print("WYNIK: Cel nieosiągalny.")
    else:
        print(f"Znaleziono {len(solutions)} rozwiązań:")
        for i, (theta2_rad, theta3_rad, theta4_rad) in enumerate(solutions):
            # Konwersja wyników na stopnie dla czytelności
            theta2_deg = math.degrees(theta2_rad)
            theta3_deg = math.degrees(theta3_rad)
            theta4_deg = math.degrees(theta4_rad)
            
            print(f"\nRozwiązanie {i+1} ('{ 'łokieć w górę' if i == 0 else 'łokieć w dół'}'):")
            print(f"  Theta 2: {theta2_deg:.2f}°")
            print(f"  Theta 3: {theta3_deg:.2f}°")
            print(f"  Theta 4: {theta4_deg:.2f}°")
            
            # Weryfikacja (Kinematyka Prosta) - sprawdzenie poprawności obliczeń
            x_check = (L1 * math.cos(theta2_rad) + 
                       L2 * math.cos(theta2_rad + theta3_rad) + 
                       L3 * math.cos(theta2_rad + theta3_rad + theta4_rad))
            
            y_check = (L1 * math.sin(theta2_rad) + 
                       L2 * math.sin(theta2_rad + theta3_rad) + 
                       L3 * math.sin(theta2_rad + theta3_rad + theta4_rad))
            
            phi_check_rad = theta2_rad + theta3_rad + theta4_rad
            phi_check_deg = math.degrees(phi_check_rad)
            
            print(f"  > Weryfikacja (FK):")
            print(f"    X = {x_check:.2f} (Cel: {TARGET_X})")
            print(f"    Y = {y_check:.2f} (Cel: {TARGET_Y})")
            print(f"    Phi = {phi_check_deg:.2f}° (Cel: {TARGET_PHI_DEGREES})")