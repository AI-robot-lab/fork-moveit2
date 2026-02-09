# Dokumentacja dla prowadzących - Przygotowanie repozytorium MoveIt2

## Podsumowanie wykonanej pracy

To repozytorium zostało przygotowane dla studentów Politechniki Rzeszowskiej uczących się robotyki z wykorzystaniem robota humanoidalnego Unitree G1 EDU. Wszystkie materiały są w języku polskim, z zachowaniem angielskich nazw technicznych (klasy, funkcje, API).

## Struktura dodanych materiałów

### 1. Dokumentacja główna

#### README_PL.md (159 linii)
**Cel:** Polskie wprowadzenie do MoveIt2 dla studentów

**Zawartość:**
- Wyjaśnienie czym jest MoveIt 2 w przystępny sposób
- Struktura repozytorium z opisami
- Kluczowe koncepcje (Planning Scene, Move Group, Planning Request)
- Instrukcje instalacji
- Przykład podstawowego kodu Python
- Linki do wszystkich innych materiałów polskich

**Dla kogo:** Wszyscy studenci rozpoczynający pracę z MoveIt 2

#### STUDENT_GUIDE_PL.md (691 linii)
**Cel:** Szczegółowy przewodnik krok po kroku

**Zawartość:**
1. **Podstawy teoretyczne**
   - ROS 2 (node, topic, service, action)
   - Architektura MoveIt 2
   - Kluczowe komponenty (Planning Scene Monitor, Move Group, Kinematic Solver)

2. **Środowisko pracy**
   - Instalacja i konfiguracja
   - Struktura workspace
   
3. **Ćwiczenia praktyczne** 
   - Ćwiczenie 1: Uruchomienie demo i eksploracja
   - Ćwiczenie 2: Pierwszy program Python (z pełnym kodem)
   - Ćwiczenie 3: Planowanie do pozycji kartezjańskiej (z kodem)
   - Ćwiczenie 4: MoveIt Servo (odniesienia do demo)

4. **Praca z kodem**
   - Struktura pakietu MoveIt
   - Najważniejsze pliki konfiguracyjne (URDF, SRDF)

5. **Debugowanie**
   - Najczęstsze problemy i rozwiązania
   - Narzędzia debugowania

6. **Zadania do wykonania**
   - 4 zadania o rosnącej trudności

**Dla kogo:** Studenci chcący dogłębnie zrozumieć MoveIt 2

#### UNITREE_G1_GUIDE_PL.md (983 linie)
**Cel:** Specjalizowany przewodnik dla robota Unitree G1 EDU

**Zawartość:**
1. **Wprowadzenie do Unitree G1 EDU**
   - Szczegółowa specyfikacja (DOF, parametry fizyczne)
   - Czujniki
   - Dlaczego MoveIt dla G1

2. **Architektura systemu**
   - Schemat połączeń
   - Grupy planowania specyficzne dla G1

3. **Konfiguracja MoveIt dla G1**
   - Struktura pakietu konfiguracyjnego
   - Limity stawów (z przykładami)
   - Konfiguracja kinematyki
   - Parametry planowania

4. **Podstawowe operacje**
   - Inicjalizacja (z pełnym kodem Python)
   - Ruch ramienia (z kodem)
   - Koordynacja obu ramion (z kodem)

5. **Zaawansowane zastosowania**
   - Teleoperacja z Servo
   - Integracja z percepcją

6. **Projekty przykładowe**
   - Kompletny Pick-and-Place dla G1 (200+ linii kodu z komentarzami)

7. **Najlepsze praktyki**
   - Bezpieczeństwo
   - Optymalizacja wydajności
   - Debugowanie

**Dla kogo:** Studenci pracujący bezpośrednio z robotem Unitree G1

#### EXERCISES_PL.md (655 linii)
**Cel:** Zbiór praktycznych ćwiczeń z rozwiązaniami

**Zawartość:**
- Ćwiczenie 1: Pierwsze kroki (2 zadania)
- Ćwiczenie 2: Manipulacja obiektami (2 zadania z kodem)
- Ćwiczenie 3: Planowanie trajektorii (2 zadania z kodem)
- Ćwiczenie 4: Sterowanie w czasie rzeczywistym (kompletny kod teleoperacji)
- Projekt końcowy: Pick-and-Place (kompletna specyfikacja)

**Dla kogo:** Wszystkie laboratoria i projekty studenckie

### 2. Kod demonstracyjny

#### demo_student_tutorial.py (460 linii, executable)
**Cel:** Interaktywny skrypt przeprowadzający przez podstawy MoveIt

**Funkcjonalności:**
- Demo 1: Planowanie do nazwanej pozycji
- Demo 2: Planowanie do pozycji kartezjańskiej  
- Demo 3: Praca z przeszkodami
- Demo 4: Podsumowanie i dalsze kroki

**Cechy:**
- Interaktywny (czeka na Enter przed każdym krokiem)
- Szczegółowe wyjaśnienia w logach
- Krok po kroku przez proces
- Czytelne komunikaty sukcesu/błędu

**Uruchomienie:**
```bash
# Terminal 1
ros2 launch moveit2_tutorials demo.launch.py

# Terminal 2
python3 demo_student_tutorial.py
```

### 3. Annotated demo files (C++)

Dodano polskie komentarze wyjaśniające do przykładów C++:

#### moveit_ros/moveit_servo/demos/cpp_interface/demo_twist.cpp
- **Nagłówek:** Wyjaśnienie celu programu, kluczowych koncepcji, zastosowań
- **Kod:** 13 kroków z szczegółowymi komentarzami
- **Podsumowanie:** Porównanie z innymi trybami sterowania

**Kluczowe wyjaśnienia:**
- Czym jest Servo i TwistCommand
- Krok po kroku przez inicjalizację
- Wyjaśnienie parametrów twist (vx, vy, vz, ωx, ωy, ωz)
- Pętla sterowania

#### moveit_ros/moveit_servo/demos/cpp_interface/demo_pose.cpp
- **Nagłówek:** Wyjaśnienie pose tracking i multi-threading
- **Kod:** 10 kroków z komentarzami o synchronizacji wątków
- **Koncepcje:** Mutex, atomic, lambda functions

**Kluczowe wyjaśnienia:**
- Różnica między twist a pose tracking
- Synchronizacja wątków (mutex, atomic)
- Dynamiczne aktualizowanie celu
- Tolerancje pozycyjne i kątowe

#### moveit_ros/moveit_servo/demos/cpp_interface/demo_joint_jog.cpp
- **Nagłówek:** Porównanie 3 trybów sterowania (JOINT_JOG, TWIST, POSE)
- **Kod:** 9 kroków z wyjaśnieniami
- **Podsumowanie:** Zalety/wady, kiedy używać

**Kluczowe wyjaśnienia:**
- Najprostsza forma sterowania
- Bezpośrednie prędkości stawów
- Zastosowania diagnostyczne

## Statystyki

- **Łączna liczba linii dokumentacji:** ~2950 linii
- **Liczba plików dodanych:** 8
- **Języki:** Polski (dokumentacja) + Python (kod) + C++ (komentarze)
- **Liczba przykładów kodu:** 15+
- **Liczba diagramów/schematów ASCII:** 5

## Jak używać tych materiałów

### Dla prowadzącego zajęcia:

1. **Pierwsze zajęcia (Wprowadzenie):**
   - Rozpocznij od README_PL.md
   - Uruchom demo_student_tutorial.py na projektorze
   - Pokaż live w RViz jak działa MoveIt

2. **Zajęcia teoretyczne:**
   - Użyj STUDENT_GUIDE_PL.md sekcja "Podstawy teoretyczne"
   - Wyjaśnij architekturę na diagramach
   - Omów kluczowe komponenty

3. **Laboratoria:**
   - EXERCISES_PL.md jako podstawa zajęć
   - Studenci pracują w parach
   - Każde ćwiczenie = 1-2 godziny

4. **Projekt z Unitree G1:**
   - UNITREE_G1_GUIDE_PL.md jako główny materiał
   - Pick-and-place example jako punkt wyjścia
   - Studenci rozszerzają o własne funkcje

### Dla studenta samodzielnie uczącego się:

**Ścieżka rekomendowana:**
1. README_PL.md (30 min) - overview
2. demo_student_tutorial.py (1h) - hands-on
3. STUDENT_GUIDE_PL.md (3-4h) - pogłębiona teoria
4. EXERCISES_PL.md Ćwiczenie 1-2 (2-3h) - praktyka
5. Annotated C++ demos (2h) - zaawansowane koncepcje
6. EXERCISES_PL.md Ćwiczenie 3-4 (3-4h) - więcej praktyki
7. UNITREE_G1_GUIDE_PL.md (4-5h) - specjalizacja
8. Projekt końcowy (10-20h) - integracja

**Łącznie:** ~30-40 godzin materiału

## Dostosowanie do innych robotów

Jeśli chcesz dostosować materiały do innego robota (nie G1):

1. **W demo_student_tutorial.py:**
   ```python
   # Linia ~53
   self.arm_group_name = "panda_arm"  # Zmień na nazwę grupy twojego robota
   ```

2. **W EXERCISES_PL.md:**
   - Zamień wszystkie odwołania do "panda_arm" / "panda_link8"
   - Dostosuj workspace (zasięg robota)

3. **Utwórz nowy przewodnik:**
   - Skopiuj UNITREE_G1_GUIDE_PL.md jako szablon
   - Dostosuj DOF, parametry, przykłady do twojego robota

## Kluczowe pedagogiczne decyzje

### 1. Język polski + angielskie terminy techniczne
**Dlaczego:** Studenci muszą znać angielskie terminy (dokumentacja, forum), ale wyjaśnienia w polskim ułatwiają zrozumienie.

**Przykład:**
```
"Planning Scene Monitor śledzi aktualny stan robota..."
nie
"Monitor Sceny Planowania śledzi aktualny stan robota..."
```

### 2. Progresja "łatwe → trudne"
- README_PL.md: Bardzo podstawowe wprowadzenie
- STUDENT_GUIDE_PL.md: Szczegółowe wyjaśnienia
- UNITREE_G1_GUIDE_PL.md: Zaawansowane zastosowania

### 3. Kod z komentarzami krok-po-kroku
Każdy fragment kodu ma:
- Nagłówek "KROK N: Co robimy"
- Inline komentarze wyjaśniające "Dlaczego"
- Konkluzje "Co osiągnęliśmy"

### 4. Hands-on od początku
- demo_student_tutorial.py można uruchomić w 5 minut
- Natychmiastowy feedback (robot się rusza!)
- Motywacja przez sukces

### 5. Teoria + Praktyka
Każda koncepcja ma:
1. Wyjaśnienie teoretyczne
2. Diagram/schemat (jeśli możliwe)
3. Przykład kodu
4. Ćwiczenie do wykonania

## Potencjalne rozszerzenia

Sugerowane dodatkowe materiały (do przyszłej pracy):

1. **Wideo tutoriale**
   - Screencast uruchamiania demo
   - Wyjaśnienie RViz interface
   - Live debugging sesja

2. **Gotowe pakiety do testów**
   - Prekonfigurowany workspace
   - Docker image z wszystkim zainstalowanym
   - Przykładowe światy Gazebo

3. **Testy/quizy**
   - Pytania wielokrotnego wyboru po każdej sekcji
   - Automatyczna ocena ćwiczeń

4. **Integracja z prawdziwym sprzętem**
   - Przewodnik podłączenia do prawdziwego G1
   - Kalibracja
   - Procedury bezpieczeństwa

5. **Zaawansowane projekty**
   - Vision-based manipulation
   - Force control
   - Dual-arm coordination
   - Whole-body motion planning

## Kontakt i feedback

Ten materiał jest "living document" - powinien być aktualizowany na podstawie:
- Feedbacku studentów
- Nowych wersji MoveIt 2
- Nowych funkcjonalności G1
- Znalezionych błędów

Zachęcam do:
- Zbierania pytań studentów (FAQ)
- Notowania problemów
- Sugestii ulepszeń

## Licencja i attribution

Materiały są zgodne z licencją BSD 3-Clause repozytorium MoveIt2.
Można je swobodnie używać, modyfikować i dystrybuować w celach edukacyjnych.

---

**Autor:** Przygotowano dla studentów Politechniki Rzeszowskiej
**Data:** Luty 2026
**Wersja:** 1.0

Dziękujemy społeczności MoveIt za stworzenie tak potężnego narzędzia edukacyjnego! 🤖
