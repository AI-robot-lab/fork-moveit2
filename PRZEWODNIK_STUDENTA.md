# Przewodnik Studenta - MoveIt 2 dla Robotyki

## Wprowadzenie

Witaj w przewodniku po MoveIt 2! Ten dokument poprowadzi Cię krok po kroku przez proces nauki i wykorzystania tego potężnego frameworka do programowania robotów.

## Spis treści
1. [Podstawy teoretyczne](#podstawy-teoretyczne)
2. [Konfiguracja środowiska](#konfiguracja-środowiska)
3. [Pierwsze programy](#pierwsze-programy)
4. [Zaawansowane koncepcje](#zaawansowane-koncepcje)
5. [Debugowanie i rozwiązywanie problemów](#debugowanie)
6. [Projekty praktyczne](#projekty-praktyczne)

---

## Podstawy teoretyczne

### Co musisz wiedzieć przed rozpoczęciem?

#### 1. ROS 2 (Robot Operating System)

**Czym jest ROS 2?**
ROS 2 to middleware - warstwa pośrednia, która umożliwia komunikację między różnymi komponentami systemu robotycznego.

**Podstawowe pojęcia:**

- **Node (węzeł)** - pojedynczy program/proces wykonujący określone zadanie
  - *Przykład:* węzeł kamery publikuje obrazy, węzeł planera oblicza trajektorię
  
- **Topic (temat)** - kanał komunikacji jednokierunkowej (publikuj/subskrybuj)
  - *Przykład:* topic `/joint_states` zawiera bieżące pozycje stawów robota
  
- **Service (serwis)** - komunikacja dwukierunkowa (request/response)
  - *Przykład:* zapytanie o rozwiązanie kinematyki odwrotnej
  
- **Action** - długotrwałe zadania z możliwością anulowania i statusem
  - *Przykład:* planowanie i wykonanie ruchu (może trwać kilka sekund)

**Dlaczego to ważne?**
MoveIt 2 działa w ekosystemie ROS 2 - wszystkie jego komponenty komunikują się przez topics, services i actions.

#### 2. Kinematyka robota

**Kinematyka prosta (Forward Kinematics - FK)**
- **Co to jest:** Obliczanie pozycji efektora końcowego na podstawie kątów stawów
- **Kierunek:** Kąty stawów → Pozycja efektora
- **Kiedy używamy:** Gdy znamy konfigurację stawów i chcemy wiedzieć gdzie jest efektor

**Kinematyka odwrotna (Inverse Kinematics - IK)**
- **Co to jest:** Obliczanie kątów stawów potrzebnych do osiągnięcia docelowej pozycji efektora
- **Kierunek:** Pozycja efektora → Kąty stawów
- **Kiedy używamy:** Gdy chcemy aby efektor był w konkretnym miejscu
- **Trudność:** Może nie mieć rozwiązania lub mieć wiele rozwiązań

**Przykład:**
```
Robot ma 6 stawów. Chcemy aby chwytak był w pozycji (X=0.5m, Y=0.3m, Z=0.4m).
FK: [θ1, θ2, θ3, θ4, θ5, θ6] → (X, Y, Z, Roll, Pitch, Yaw)
IK: (X, Y, Z, Roll, Pitch, Yaw) → [θ1, θ2, θ3, θ4, θ5, θ6]
```

#### 3. Planowanie trajektorii

**Czym jest trajektoria?**
Trajektoria to sekwencja pozycji robota w czasie, która prowadzi od stanu początkowego do docelowego.

**Dlaczego nie możemy po prostu przeskoczyć do celu?**
- Robot musi poruszać się płynnie (ograniczenia prędkości/przyspieszenia)
- Musimy unikać kolizji po drodze
- Trzeba uwzględnić dynamikę robota

**Algorytmy planowania w MoveIt 2:**

1. **Sampling-based (OMPL)**
   - Losowo próbkuje przestrzeń konfiguracji
   - Buduje graf możliwych ruchów
   - Znajduje ścieżkę w grafie
   - **Zaleta:** Działa w złożonych scenariach
   - **Wada:** Ścieżki mogą być nieoptymalne

2. **Optimization-based (STOMP)**
   - Zaczyna od początkowej trajektorii
   - Iteracyjnie optymalizuje, minimalizując koszt
   - **Zaleta:** Płynniejsze trajektorie
   - **Wada:** Może utknąć w lokalnym minimum

3. **Industrial (Pilz)**
   - Proste ruchy: liniowe, kołowe, blend
   - **Zaleta:** Przewidywalne, szybkie
   - **Wada:** Tylko dla prostych ruchów

#### 4. Wykrywanie kolizji

**Po co wykrywać kolizje?**
- Ochrona robota przed samouszkodzeniem
- Unikanie uszkodzenia środowiska
- Bezpieczeństwo operatora

**Jak MoveIt 2 wykrywa kolizje?**
1. Każdy link robota ma geometrię kolizji (mesh, prymitywy)
2. Planning Scene zawiera przeszkody
3. Algorytmy szybko sprawdzają przecięcia geometrii
4. Kolizje sprawdzane są w każdym punkcie trajektorii

---

## Konfiguracja środowiska

### Krok 1: Instalacja Ubuntu i ROS 2

```bash
# Ubuntu 22.04 LTS z ROS 2 Humble (zalecane)

# Instalacja ROS 2 Humble
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Dodanie repozytoriów ROS 2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalacja ROS 2 i narzędzi
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
sudo apt install ros-humble-moveit
```

### Krok 2: Utworzenie workspace

```bash
# Utworzenie folderu workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Sklonowanie przykładowych tutoriali MoveIt 2
git clone https://github.com/ros-planning/moveit2_tutorials.git -b humble

# Powrót do głównego folderu workspace
cd ~/ros2_ws

# Instalacja wszystkich zależności
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y

# Kompilacja workspace
colcon build

# Dodanie do .bashrc dla automatycznego source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Krok 3: Weryfikacja instalacji

```bash
# Sprawdzenie instalacji ROS 2
ros2 pkg list | grep moveit

# Powinieneś zobaczyć listę pakietów moveit_*
```

---

## Pierwsze programy

### Przykład 1: Wizualizacja robota

**Cel:** Nauczyć się wizualizować robota w RViz

**Kod:**
```bash
# Terminal 1: Uruchomienie demo z robotem Panda
ros2 launch moveit2_tutorials demo.launch.py
```

**Co się dzieje?**
1. Ładowany jest model robota Panda (7-DOF manipulator)
2. Uruchamia się węzeł `move_group` - główny serwer planowania
3. Otwiera się RViz z konfiguracją MoveIt

**Interfejs RViz:**
- **Planning tab** - zakładka planowania
  - *Planning Group* - wybór grupy stawów (np. "panda_arm")
  - *Goal State* - ustawienie celu
  - *Plan* - zaplanuj trajektorię
  - *Execute* - wykonaj ruch
  
- **Scene** - wizualizacja środowiska
  - Robot (kolorowy model)
  - Przeszkody (jeśli dodane)
  - Planning requests (wizualizacja celów)

**Zadanie praktyczne:**
1. Wybierz "panda_arm" jako Planning Group
2. Przeciągnij interaktywny marker (kolorowe osie) aby ustawić cel
3. Kliknij "Plan" - zobaczysz zaplanowaną trajektorię
4. Kliknij "Execute" - robot się poruszy

### Przykład 2: Pierwszy program Python

**Cel:** Napisać program, który planuje i wykonuje prosty ruch

**Plik:** `~/ros2_ws/src/my_moveit_scripts/simple_move.py`

```python
#!/usr/bin/env python3
"""
Prosty program demonstracyjny MoveIt 2

Ten skrypt pokazuje podstawowe kroki planowania ruchu:
1. Inicjalizacja węzła ROS
2. Utworzenie interfejsu MoveGroup
3. Ustawienie celu w przestrzeni kartezjańskiej
4. Zaplanowanie trajektorii
5. Wykonanie ruchu
"""

import rclpy
from rclpy.node import Node
from moveit_py import MoveItPy
from moveit_py.planning import PlanRequestParameters
import numpy as np

def main():
    # Krok 1: Inicjalizacja ROS 2
    # ------------------------------------------------
    # Tworzymy węzeł ROS - podstawową jednostkę programu w ROS
    rclpy.init()
    node = Node('simple_moveit_example')
    node.get_logger().info("Węzeł uruchomiony! Inicjalizacja MoveIt...")
    
    # Krok 2: Utworzenie interfejsu MoveIt
    # ------------------------------------------------
    # MoveItPy to główny interfejs do planowania i wykonywania ruchów
    moveit = MoveItPy(node_name="moveit_py_node")
    
    # Wybór grupy planowania - zdefiniowane w konfiguracji robota
    # Dla robota Panda: "panda_arm" to główne ramię (7 stawów)
    planning_group = "panda_arm"
    
    # Pobranie interfejsu planning_component dla wybranej grupy
    panda_arm = moveit.get_planning_component(planning_group)
    node.get_logger().info(f"Używam grupy planowania: {planning_group}")
    
    # Krok 3: Ustawienie stanu początkowego
    # ------------------------------------------------
    # Zawsze ustawiamy stan początkowy jako bieżący stan robota
    # To zapewnia że planowanie rozpocznie się od rzeczywistej pozycji
    panda_arm.set_start_state_to_current_state()
    
    # Krok 4: Definicja celu w przestrzeni kartezjańskiej
    # ------------------------------------------------
    # Chcemy aby efektor końcowy (chwytak) znalazł się w konkretnym miejscu
    # Definicja: [X, Y, Z, orientacja jako kwaternion qx, qy, qz, qw]
    
    # Pozycja celu: 40cm do przodu (X), 20cm w lewo (Y), 50cm w górę (Z)
    # Orientacja: chwytak skierowany w dół (standardowa pozycja chwytania)
    goal_pose = {
        'position': [0.4, 0.2, 0.5],  # X, Y, Z w metrach
        'orientation': [0.0, 0.707, 0.0, 0.707]  # qx, qy, qz, qw (kwaternion)
    }
    
    node.get_logger().info(f"Cel: pozycja={goal_pose['position']}")
    
    # Ustawienie celu w MoveIt
    panda_arm.set_goal_state(
        pose_stamped_msg=create_pose_stamped(goal_pose),
        pose_link="panda_hand"  # Link efektora końcowego
    )
    
    # Krok 5: Planowanie trajektorii
    # ------------------------------------------------
    # MoveIt użyje algorytmu planowania (np. OMPL) aby znaleźć
    # bezkolizyjną ścieżkę od stanu początkowego do celu
    
    node.get_logger().info("Planuję trajektorię...")
    
    # Parametry planowania
    plan_params = PlanRequestParameters(
        planning_time=10.0,  # Maksymalnie 10 sekund na planowanie
        planning_attempts=5,  # Maksymalnie 5 prób
        max_velocity_scaling_factor=0.5,  # Zmniejsz prędkość do 50% maksymalnej
        max_acceleration_scaling_factor=0.5  # Zmniejsz przyspieszenie do 50%
    )
    
    # Wywołanie planera
    plan_result = panda_arm.plan(plan_params)
    
    # Krok 6: Weryfikacja wyniku planowania
    # ------------------------------------------------
    if plan_result.error_code.val != 1:  # 1 = SUCCESS
        node.get_logger().error(
            f"Planowanie nie powiodło się! Kod błędu: {plan_result.error_code.val}"
        )
        # Możliwe przyczyny: cel nieosiągalny, kolizja, timeout
        return
    
    node.get_logger().info("Trajektoria zaplanowana pomyślnie!")
    node.get_logger().info(
        f"Trajektoria zawiera {len(plan_result.trajectory.joint_trajectory.points)} punktów"
    )
    
    # Krok 7: Wykonanie ruchu
    # ------------------------------------------------
    # Wyślij zaplanowaną trajektorię do kontrolera robota
    # W symulacji: robot w RViz się poruszy
    # Na prawdziwym robocie: silniki wykonają ruch
    
    node.get_logger().info("Wykonuję ruch...")
    panda_arm.execute()
    
    node.get_logger().info("Ruch zakończony!")
    
    # Krok 8: Cleanup
    # ------------------------------------------------
    rclpy.shutdown()

def create_pose_stamped(goal_dict):
    """
    Funkcja pomocnicza: tworzenie wiadomości PoseStamped
    
    Args:
        goal_dict: Słownik z kluczami 'position' i 'orientation'
    
    Returns:
        PoseStamped message
    """
    from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
    from std_msgs.msg import Header
    
    pose_stamped = PoseStamped()
    pose_stamped.header = Header()
    pose_stamped.header.frame_id = "panda_link0"  # Bazowy frame robota
    
    pose_stamped.pose = Pose()
    pose_stamped.pose.position = Point(
        x=goal_dict['position'][0],
        y=goal_dict['position'][1],
        z=goal_dict['position'][2]
    )
    pose_stamped.pose.orientation = Quaternion(
        x=goal_dict['orientation'][0],
        y=goal_dict['orientation'][1],
        z=goal_dict['orientation'][2],
        w=goal_dict['orientation'][3]
    )
    
    return pose_stamped

if __name__ == '__main__':
    main()
```

**Jak uruchomić:**

```bash
# Terminal 1: Uruchom demo (jeśli nie jest już uruchomione)
ros2 launch moveit2_tutorials demo.launch.py

# Terminal 2: Uruchom nasz skrypt
cd ~/ros2_ws/src/my_moveit_scripts
chmod +x simple_move.py
python3 simple_move.py
```

**Co powinieneś zobaczyć:**
- Robot w RViz porusza się do nowej pozycji
- W terminalu logi opisujące każdy krok

---

## Zaawansowane koncepcje

### 1. Planning Scene - Zarządzanie środowiskiem

**Czym jest Planning Scene?**
Planning Scene to reprezentacja środowiska robota w MoveIt 2, zawierająca:
- Model robota
- Przeszkody (statyczne i dynamiczne)
- Obiekty do manipulacji (Attached Collision Objects)

**Przykład: Dodawanie przeszkody**

```python
#!/usr/bin/env python3
"""
Dodawanie przeszkód do Planning Scene

Demonstracja:
- Tworzenie obiektu kolizji (pudełko)
- Dodanie do sceny
- Planowanie z unikaniem przeszkody
"""

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def add_box_obstacle(moveit_interface, name, position, size):
    """
    Dodaje prostopadłościan jako przeszkodę
    
    Args:
        moveit_interface: Interfejs MoveItPy
        name: Nazwa przeszkody (string)
        position: [x, y, z] pozycja środka pudełka
        size: [długość, szerokość, wysokość] wymiary
    
    Wyjaśnienie:
    Ta funkcja tworzy obiekt kolizji typu BOX i dodaje go do Planning Scene.
    MoveIt automatycznie uwzględni tę przeszkodę podczas planowania.
    """
    
    # Krok 1: Utworzenie obiektu kolizji
    collision_object = CollisionObject()
    collision_object.header.frame_id = "panda_link0"  # Frame bazowy robota
    collision_object.id = name  # Unikalna nazwa przeszkody
    
    # Krok 2: Definicja geometrii - BOX (prostopadłościan)
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = size  # [długość x, szerokość y, wysokość z]
    
    # Krok 3: Pozycja przeszkody
    box_pose = Pose()
    box_pose.position.x = position[0]
    box_pose.position.y = position[1]
    box_pose.position.z = position[2]
    box_pose.orientation.w = 1.0  # Brak rotacji
    
    # Krok 4: Przypisanie geometrii i pozycji do obiektu
    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(box_pose)
    
    # Krok 5: Operacja ADD - dodaj do sceny
    collision_object.operation = CollisionObject.ADD
    
    # Krok 6: Wysłanie do Planning Scene
    moveit_interface.apply_planning_scene(collision_object)
    
    print(f"Dodano przeszkodę '{name}' w pozycji {position}")

# Przykład użycia:
# add_box_obstacle(moveit, "table", [0.5, 0, 0.2], [0.8, 1.2, 0.4])
# Tworzy stół (80x120x40cm) w odległości 50cm przed robotem
```

**Dlaczego to ważne?**
- W prawdziwym świecie robot musi unikać mebli, ścian, ludzi
- Planning Scene pozwala modelować te przeszkody
- MoveIt automatycznie znajdzie ścieżkę omijającą przeszkody

### 2. Constraints (Ograniczenia)

**Czym są ograniczenia?**
Constraints to dodatkowe warunki, które musi spełniać trajektoria:
- **Orientation Constraints** - utrzymuj określoną orientację
- **Position Constraints** - zostań w określonym regionie
- **Joint Constraints** - ogranicz zakres stawu

**Przykład: Utrzymywanie poziomego chwytaka**

```python
def add_orientation_constraint(planning_component):
    """
    Dodaje ograniczenie orientacji - chwytak zawsze poziomy
    
    Przypadek użycia:
    Noszenie płynu w kubku - nie możemy przechylić chwytaka,
    bo płyn by się wylał.
    """
    from moveit_msgs.msg import OrientationConstraint
    
    constraint = OrientationConstraint()
    constraint.header.frame_id = "panda_link0"
    constraint.link_name = "panda_hand"  # Link efektora
    
    # Docelowa orientacja (poziomy chwytak)
    constraint.orientation.w = 1.0  # Kwaternion identyczności
    
    # Tolerancja (w radianach) - jak bardzo może odbiegać
    constraint.absolute_x_axis_tolerance = 0.1  # ~5.7 stopni
    constraint.absolute_y_axis_tolerance = 0.1
    constraint.absolute_z_axis_tolerance = 0.1
    
    constraint.weight = 1.0  # Ważność ograniczenia (0-1)
    
    planning_component.set_path_constraints([constraint])
    print("Dodano ograniczenie: chwytak musi pozostać poziomy")
```

### 3. Cartesian Path Planning

**Co to jest?**
Ruch w linii prostej w przestrzeni kartezjańskiej (X, Y, Z).

**Kiedy używamy?**
- Zbliżanie się do obiektu przed chwyceniem
- Rysowanie/spawanie w linii prostej
- Nalewanie płynu

**Przykład:**

```python
def plan_cartesian_path(moveit_interface, waypoints):
    """
    Planuje trajektorię przez sekwencję punktów kartezjańskich
    
    Args:
        waypoints: Lista pozycji [x, y, z] przez które ma przejść efektor
    
    Wyjaśnienie:
    W przeciwieństwie do zwykłego planowania, tutaj efektor
    porusza się w linii prostej między punktami.
    """
    from geometry_msgs.msg import Pose
    
    # Utworzenie listy Pose messages
    poses = []
    for wp in waypoints:
        pose = Pose()
        pose.position.x = wp[0]
        pose.position.y = wp[1]
        pose.position.z = wp[2]
        pose.orientation.w = 1.0
        poses.append(pose)
    
    # Planowanie ścieżki kartezjańskiej
    # eef_step: 1cm - rozdzielczość interpolacji
    # jump_threshold: 0 - wyłącz detekcję skoków w joint space
    (plan, fraction) = moveit_interface.compute_cartesian_path(
        poses,
        eef_step=0.01,  # 1cm między kolejnymi punktami
        jump_threshold=0.0
    )
    
    print(f"Zaplanowano {fraction*100:.1f}% ścieżki kartezjańskiej")
    
    if fraction < 0.95:
        print("UWAGA: Nie udało się zaplanować pełnej ścieżki!")
        print("Możliwe przyczyny: kolizja, ograniczenia stawów")
    
    return plan

# Przykład: Ruch w dół o 10cm, potem w bok o 15cm
waypoints = [
    [0.4, 0.2, 0.5],   # Punkt startowy
    [0.4, 0.2, 0.4],   # W dół o 10cm
    [0.4, 0.35, 0.4]   # W bok o 15cm
]
plan = plan_cartesian_path(moveit, waypoints)
```

---

## Debugowanie

### Typowe problemy i rozwiązania

#### Problem 1: "No IK solution found"

**Objawy:**
```
[ERROR] No IK solution found for pose
```

**Przyczyny:**
- Cel jest poza zasięgiem robota (workspace)
- Cel wymaga niemożliwej konfiguracji stawów
- Kolizja w stanie docelowym

**Rozwiązanie:**
```python
# Sprawdź czy cel jest osiągalny
def check_reachability(moveit_interface, target_pose):
    """Sprawdza czy robot może osiągnąć daną pozycję"""
    
    # Spróbuj obliczyć IK
    ik_result = moveit_interface.solve_ik(target_pose)
    
    if ik_result.error_code.val == 1:
        print("✓ Cel osiągalny!")
        print(f"  Konfiguracja stawów: {ik_result.joint_state.position}")
        return True
    else:
        print("✗ Cel nieosiągalny!")
        print(f"  Kod błędu: {ik_result.error_code.val}")
        
        # Wskazówki
        print("\nSprawdź:")
        print("- Czy cel jest w zasięgu robota?")
        print("- Czy orientacja jest prawidłowa?")
        print("- Czy nie ma kolizji w tej pozycji?")
        return False
```

#### Problem 2: Planer nie znajduje rozwiązania

**Objawy:**
```
[WARN] Unable to find valid plan within allocated time
```

**Rozwiązanie:**
1. Zwiększ czas planowania
2. Zmień algorytm planowania
3. Uprość scenę (usuń niepotrzebne przeszkody)

```python
# Użyj bardziej agresywnych parametrów
plan_params = PlanRequestParameters(
    planning_time=30.0,  # Więcej czasu
    planning_attempts=10,  # Więcej prób
    planner_id="RRTConnectkConfigDefault"  # Inny algorytm
)
```

#### Problem 3: Robot porusza się zbyt szybko

**Rozwiązanie:**
```python
# Zmniejsz skalowanie prędkości i przyspieszenia
plan_params.max_velocity_scaling_factor = 0.2  # 20% max prędkości
plan_params.max_acceleration_scaling_factor = 0.2  # 20% max przyspieszenia
```

### Narzędzia debugowania

#### 1. Wizualizacja w RViz

```bash
# Dodaj wizualizację trajektorii:
# RViz -> Add -> MotionPlanning
# Włącz:
#   - Planned Path (zaplanowana trajektoria)
#   - Trail (ślad ruchu)
```

#### 2. Logi ROS

```bash
# Włącz szczegółowe logi MoveIt
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
ros2 run moveit_ros_move_group move_group --ros-args --log-level debug
```

---

## Projekty praktyczne

### Projekt 1: Pick and Place (Podnieś i odłóż)

**Cel:** Napisz program, który:
1. Identyfikuje obiekt na stole (symulowany jako known object)
2. Planuje ruch do obiektu
3. Chwyta obiekt
4. Przenosi w nowe miejsce
5. Odkłada obiekt

**Szkielet programu:**

```python
def pick_and_place_demo():
    """
    Kompletny przykład pick-and-place
    
    Kroki:
    1. Ruch nad obiektem
    2. Zjazd w dół (Cartesian)
    3. Zamknięcie chwytaka
    4. Podniesienie obiektu
    5. Ruch nad miejscem docelowym
    6. Zjazd
    7. Otwarcie chwytaka
    8. Podniesienie
    """
    
    # TODO: Implementacja dla studenta
    pass

# Wskazówki:
# - Użyj Cartesian paths dla zbliżania/oddalania
# - Dodaj obiekt jako AttachedCollisionObject po chwyceniu
# - Sprawdzaj powodzenie każdego kroku
```

### Projekt 2: Obstacle Avoidance Challenge

**Cel:** Robot musi przejść przez "labirynt" przeszkód

```python
def obstacle_course():
    """
    Dodaj kilka przeszkód i zaplanuj ruch przez nie
    """
    
    # Przeszkoda 1: Ściana lewa
    add_box_obstacle(moveit, "wall_left", [-0.5, 0.3, 0.5], [0.1, 0.02, 1.0])
    
    # Przeszkoda 2: Ściana prawa
    add_box_obstacle(moveit, "wall_right", [0.5, -0.3, 0.5], [0.1, 0.02, 1.0])
    
    # Przeszkoda 3: Sufit
    add_box_obstacle(moveit, "ceiling", [0, 0, 0.8], [1.0, 1.0, 0.02])
    
    # Zadanie: Zaplanuj ruch z punktu A do B unikając przeszkód
    # ...
```

---

## Podsumowanie

Gratulacje! Przeszedłeś przez podstawy MoveIt 2. Teraz powinieneś:

✓ Rozumieć architekturę MoveIt 2
✓ Umieć konfigurować środowisko ROS 2
✓ Pisać podstawowe programy planowania ruchu
✓ Debugować typowe problemy
✓ Mieć bazę do dalszej nauki

### Kolejne kroki:

1. **Przeczytaj:** [UNITREE_G1_INTEGRATION.md](UNITREE_G1_INTEGRATION.md) - integracja z robotem G1
2. **Eksperymentuj:** Modyfikuj przykłady, testuj różne scenariusze
3. **Zaawansowane tematy:** 
   - Dual-arm manipulation (współpraca dwóch ramion)
   - Visual servoing (sterowanie oparte na obrazie)
   - Mobile manipulation (robot mobilny z ramieniem)

### Zasoby dodatkowe:

- **MoveIt 2 Tutorials:** https://moveit.picknik.ai/
- **ROS 2 Documentation:** https://docs.ros.org/
- **Forum:** https://robotics.stackexchange.com/

---

**Powodzenia w nauce robotyki! 🤖**
