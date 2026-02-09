# Przewodnik studenta - MoveIt 2 dla Politechniki Rzeszowskiej

## Spis treści
1. [Wprowadzenie](#wprowadzenie)
2. [Podstawy teoretyczne](#podstawy-teoretyczne)
3. [Środowisko pracy](#środowisko-pracy)
4. [Ćwiczenia praktyczne](#ćwiczenia-praktyczne)
5. [Praca z kodem](#praca-z-kodem)
6. [Debugowanie](#debugowanie)
7. [Zadania do wykonania](#zadania-do-wykonania)

## Wprowadzenie

Ten przewodnik poprowadzi Cię krok po kroku przez naukę MoveIt 2. Zakładamy, że masz podstawową znajomość:
- Programowania w C++ lub Python
- Systemów Linux (Ubuntu)
- Podstaw robotyki (kinematyka, DH parameters)

## Podstawy teoretyczne

### 1. Robot Operating System (ROS 2)

**ROS 2** to framework do tworzenia aplikacji robotycznych. Kluczowe koncepcje:

#### Node (węzeł)
- Podstawowa jednostka programu w ROS 2
- Każdy node wykonuje konkretne zadanie (np. sterowanie, planowanie, wizualizacja)
- Node'y komunikują się przez **topic'i**, **serwisy** i **akcje**

```bash
# Lista aktywnych node'ów
ros2 node list

# Informacje o node'dzie
ros2 node info /move_group
```

#### Topic (temat)
- Kanał komunikacji między node'ami
- Publisher wysyła wiadomości, Subscriber je odbiera
- Asynchroniczna komunikacja (fire-and-forget)

```bash
# Lista topic'ów
ros2 topic list

# Nasłuchiwanie na topic
ros2 topic echo /joint_states
```

#### Service (serwis)
- Synchroniczna komunikacja request-response
- Klient wysyła żądanie, serwer odpowiada
- Używane do operacji, które wymagają potwierdzenia

```bash
# Lista serwisów
ros2 service list

# Wywołanie serwisu
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene
```

#### Action (akcja)
- Długotrwałe operacje z feedback'iem
- Można anulować w trakcie wykonywania
- MoveIt używa akcji do planowania i wykonywania trajektorii

### 2. MoveIt 2 - Architektura

```
┌─────────────────────────────────────────────────────────────┐
│                    Aplikacja użytkownika                     │
│              (Twój kod Python/C++)                          │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ↓
┌─────────────────────────────────────────────────────────────┐
│                   MoveIt Planning Interface                  │
│  (move_group_interface, planning_scene_interface)           │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ↓
┌─────────────────────────────────────────────────────────────┐
│                      Move Group Node                         │
│  • Zarządzanie żądaniami planowania                         │
│  • Integracja z plannerami                                  │
│  • Wykrywanie kolizji                                       │
└─────────────────────┬───────────────────────────────────────┘
                      │
         ┌────────────┼────────────┐
         ↓            ↓            ↓
    ┌─────────┐  ┌─────────┐  ┌─────────┐
    │  OMPL   │  │  Pilz   │  │ STOMP   │
    │ Planner │  │ Planner │  │ Planner │
    └─────────┘  └─────────┘  └─────────┘
```

### 3. Kluczowe komponenty MoveIt 2

#### Planning Scene Monitor
- Śledzi aktualny stan robota i otoczenia
- Aktualizuje się na podstawie:
  - `joint_states` - aktualne pozycje stawów
  - `tf` - transformacje między układami współrzędnych
  - Sensor'ów (kamery głębi, lidar)

**Dlaczego jest ważny?**
Planning Scene Monitor zapewnia, że planner zawsze ma aktualne informacje o:
- Gdzie robot się znajduje
- Jakie są przeszkody w otoczeniu
- Jakie części robota mogą się ze sobą kolidować

#### Move Group
- Centralny node koordynujący planowanie
- Odbiera żądania planowania
- Wywołuje odpowiedni planner
- Sprawdza kolizje
- Zwraca trajektorię do wykonania

**Przykładowy workflow:**
1. Aplikacja wysyła żądanie: "Przesuń end-effector do pozycji X,Y,Z"
2. Move Group wybiera planner (np. OMPL)
3. Planner oblicza trajektorię unikającą przeszkód
4. Move Group sprawdza czy trajektoria jest bezpieczna
5. Zwraca trajektorię do aplikacji

#### Kinematic Solver
- Rozwiązuje kinematykę odwrotną (IK - Inverse Kinematics)
- Problem: mamy pozycję end-effectora, szukamy kątów stawów
- MoveIt 2 używa różnych solverów:
  - **KDL** - szybki, numeryczny
  - **TracIK** - bardziej niezawodny
  - **IKFast** - bardzo szybki, generowany dla konkretnego robota

**Przykład problemu IK:**
```
Dane: Pozycja chwytaka (x=0.5m, y=0.3m, z=0.8m)
Szukane: Kąty stawów [θ1, θ2, θ3, θ4, θ5, θ6]
```

## Środowisko pracy

### Instalacja i konfiguracja

#### 1. Instalacja ROS 2 Humble
```bash
# Dodaj repozytorium ROS 2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Instalacja
sudo apt update
sudo apt install ros-humble-desktop
```

#### 2. Konfiguracja workspace
```bash
# Źródło ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Utworzenie workspace
mkdir -p ~/moveit2_ws/src
cd ~/moveit2_ws/src

# Pobranie MoveIt 2
git clone https://github.com/ros-planning/moveit2.git -b humble

# Instalacja zależności
cd ~/moveit2_ws
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y

# Kompilacja
cd ~/moveit2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 3. Konfiguracja dla Unitree G1
```bash
# Dodaj workspace Unitree G1 (jeśli dostępne)
cd ~/moveit2_ws/src
git clone <unitree_g1_moveit_config_repo>

# Ponowna kompilacja
cd ~/moveit2_ws
colcon build
source install/setup.bash
```

### Struktura workspace

```
~/moveit2_ws/
├── build/           # Pliki kompilacji (nie edytować)
├── install/         # Zainstalowane pakiety (nie edytować)
├── log/            # Logi kompilacji
└── src/            # Kod źródłowy (TU PRACUJESZ)
    ├── moveit2/    # Główne repozytorium MoveIt
    └── (inne pakiety)
```

## Ćwiczenia praktyczne

### Ćwiczenie 1: Uruchomienie demo i eksploracja

**Cel:** Zapoznanie się z interfejsem RViz i podstawowymi operacjami.

```bash
# Terminal 1: Uruchom demo
source ~/moveit2_ws/install/setup.bash
ros2 launch moveit2_tutorials demo.launch.py

# Terminal 2: Sprawdź działające node'y
ros2 node list
# Powinieneś zobaczyć /move_group, /rviz, itp.

# Terminal 3: Monitoruj stan stawów
ros2 topic echo /joint_states
```

**Zadania w RViz:**
1. W zakładce "Planning" ustaw nowy cel (przeciągnij interaktywny marker)
2. Kliknij "Plan" - zobaczysz wizualizację planowanej trajektorii
3. Kliknij "Execute" - robot wykona ruch
4. Eksperymentuj z dodawaniem przeszkód (zakładka "Scene Objects")

**Co obserwować:**
- Jak planner omija przeszkody
- Jak zmienia się trajektoria przy różnych celach
- Komunikaty w terminalu - sukces/porażka planowania

### Ćwiczenie 2: Pierwszy program Python

**Cel:** Napisanie prostego programu planującego ruch.

Utwórz plik `my_first_motion.py`:

```python
#!/usr/bin/env python3
"""
Prosty program planujący ruch ramienia robota.
Ten skrypt demonstruje podstawowe użycie MoveIt 2 Python API.
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import time

class SimpleMotionPlanner(Node):
    """
    Node ROS 2 do prostego planowania ruchu.
    
    Ten node demonstruje:
    - Inicjalizację MoveItPy
    - Pobieranie planning component
    - Planowanie ruchu do nazwanej pozycji
    - Wykonywanie zaplanowanej trajektorii
    """
    
    def __init__(self):
        super().__init__('simple_motion_planner')
        self.get_logger().info('Inicjalizacja Simple Motion Planner...')
        
        # Utworzenie obiektu MoveItPy
        # Ten obiekt zapewnia dostęp do wszystkich funkcjonalności MoveIt
        self.moveit = MoveItPy(node_name="moveit_py_node")
        
        # Pobranie planning component dla grupy "panda_arm"
        # Planning component pozwala na:
        # - Ustawianie stanów początkowych i docelowych
        # - Planowanie trajektorii
        # - Nie wykonuje ruchu - tylko planuje!
        self.arm = self.moveit.get_planning_component("panda_arm")
        
        self.get_logger().info('Inicjalizacja zakończona!')
    
    def plan_to_named_target(self, target_name):
        """
        Planuje ruch do nazwanej pozycji.
        
        Args:
            target_name: Nazwa pozycji zdefiniowana w SRDF (np. "ready", "home")
        
        Returns:
            bool: True jeśli planowanie się powiodło
        """
        self.get_logger().info(f'Planowanie ruchu do pozycji: {target_name}')
        
        # Krok 1: Ustaw stan początkowy jako aktualną pozycję robota
        # To mówi plannerowi "zacznij planowanie od tego, gdzie robot jest teraz"
        self.arm.set_start_state_to_current_state()
        
        # Krok 2: Ustaw stan docelowy jako nazwaną pozycję
        # Nazwane pozycje są zdefiniowane w pliku SRDF robota
        self.arm.set_goal_state(configuration_name=target_name)
        
        # Krok 3: Zaplanuj trajektorię
        # Tutaj dzieje się magia - planner oblicza ścieżkę
        plan_result = self.arm.plan()
        
        # Krok 4: Sprawdź wynik planowania
        if plan_result:
            self.get_logger().info('✓ Planowanie zakończone sukcesem!')
            return True
        else:
            self.get_logger().error('✗ Planowanie nie powiodło się!')
            return False
    
    def execute_plan(self):
        """
        Wykonuje ostatnio zaplanowaną trajektorię.
        
        UWAGA: To faktycznie porusza robotem!
        """
        self.get_logger().info('Wykonywanie trajektorii...')
        
        # Pobierz ostatnio zaplanowaną trajektorię
        trajectory = self.arm.get_plan_trajectory()
        
        if trajectory:
            # Wykonaj trajektorię na prawdziwym robocie
            success = self.moveit.execute(trajectory, blocking=True)
            
            if success:
                self.get_logger().info('✓ Trajektoria wykonana!')
            else:
                self.get_logger().error('✗ Wykonanie nie powiodło się!')
        else:
            self.get_logger().error('Brak trajektorii do wykonania!')

def main():
    """Główna funkcja programu."""
    # Inicjalizacja ROS 2
    rclpy.init()
    
    # Utworzenie node'a
    planner = SimpleMotionPlanner()
    
    # Daj czas na załadowanie planning scene
    time.sleep(2)
    
    # Sekwencja ruchów
    targets = ["ready", "home", "ready"]
    
    for target in targets:
        # Zaplanuj ruch
        if planner.plan_to_named_target(target):
            # Jeśli planowanie się powiodło, wykonaj ruch
            planner.execute_plan()
            # Poczekaj przed następnym ruchem
            time.sleep(2)
        else:
            planner.get_logger().error(f'Przerwanie - nie można zaplanować do {target}')
            break
    
    # Zakończenie
    planner.get_logger().info('Program zakończony!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Uruchomienie:**
```bash
# Terminal 1: Uruchom Move Group
ros2 launch moveit2_tutorials demo.launch.py

# Terminal 2: Uruchom swój program
cd ~/moveit2_ws
chmod +x my_first_motion.py
python3 my_first_motion.py
```

### Ćwiczenie 3: Planowanie do pozycji w przestrzeni kartezjańskiej

**Cel:** Nauczenie się planowania do konkretnej pozycji end-effectora.

```python
#!/usr/bin/env python3
"""
Planowanie ruchu do pozycji w przestrzeni kartezjańskiej.
Pokazuje jak używać pose goals zamiast joint goals.
"""

import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit.planning import MoveItPy

def create_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """
    Pomocnicza funkcja do tworzenia Pose.
    
    Args:
        x, y, z: Pozycja w metrach
        qx, qy, qz, qw: Orientacja jako kwaternion
    
    Returns:
        Pose: Obiekt reprezentujący pozę w przestrzeni 3D
    """
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose

def main():
    rclpy.init()
    
    # Inicjalizacja MoveIt
    moveit = MoveItPy(node_name="cartesian_planning")
    arm = moveit.get_planning_component("panda_arm")
    
    # Ustaw stan początkowy
    arm.set_start_state_to_current_state()
    
    # Zdefiniuj cel jako pozycję w przestrzeni
    # Współrzędne w układzie base robota
    target_pose = create_pose(
        x=0.4,   # 40 cm przed robotem
        y=0.2,   # 20 cm w prawo
        z=0.5,   # 50 cm w górę
        qw=1.0   # Orientacja: chwytka skierowana w dół
    )
    
    # Ustaw cel jako pose goal
    arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="panda_hand")
    
    # Zaplanuj i wykonaj
    plan_result = arm.plan()
    if plan_result:
        print("✓ Trajektoria zaplanowana!")
        trajectory = arm.get_plan_trajectory()
        moveit.execute(trajectory, blocking=True)
        print("✓ Trajektoria wykonana!")
    else:
        print("✗ Nie udało się zaplanować trajektorii")
        print("Możliwe przyczyny:")
        print("  - Pozycja poza workspace robota")
        print("  - Brak rozwiązania kinematyki odwrotnej")
        print("  - Kolizja z przeszkodami")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Ćwiczenie 4: Praca z MoveIt Servo (sterowanie w czasie rzeczywistym)

**Cel:** Nauka sterowania robotem przez komendy prędkości.

Zobacz szczegółowe przykłady z komentarzami w:
- `moveit_ros/moveit_servo/demos/cpp_interface/demo_twist.cpp`
- `moveit_ros/moveit_servo/demos/cpp_interface/demo_pose.cpp`
- `moveit_ros/moveit_servo/demos/cpp_interface/demo_joint_jog.cpp`

**Kiedy używać Servo:**
- ✓ Sterowanie joystickiem/teleoperacja
- ✓ Reagowanie na input w czasie rzeczywistym (siły, wizja)
- ✓ Delikatne pozycjonowanie (assembly tasks)
- ✗ Długie, złożone trajektorie (użyj Move Group)
- ✗ Gdy potrzebujesz gwarancji osiągnięcia celu

## Praca z kodem

### Struktura typowego pakietu MoveIt

```
my_robot_moveit_config/
├── config/                      # Pliki konfiguracyjne
│   ├── moveit.rviz             # Konfiguracja RViz
│   ├── joint_limits.yaml       # Limity prędkości i przyspieszeń
│   ├── kinematics.yaml         # Konfiguracja solverów IK
│   ├── ompl_planning.yaml      # Parametry plannera OMPL
│   └── servo_config.yaml       # Parametry Servo
├── launch/                      # Pliki launch
│   ├── demo.launch.py          # Uruchomienie w symulacji
│   └── real_robot.launch.py    # Uruchomienie z prawdziwym robotem
├── config/                      # Modele robota
│   ├── my_robot.srdf           # Semantic Robot Description Format
│   └── my_robot.urdf           # Unified Robot Description Format
└── CMakeLists.txt / package.xml # Konfiguracja pakietu
```

### Najważniejsze pliki konfiguracyjne

#### URDF (Unified Robot Description Format)
Opisuje fizyczną strukturę robota:
- Linki (części sztywne)
- Stawy (połączenia ruchome)
- Geometrię kolizji i wizualizacji
- Parametry inercji

#### SRDF (Semantic Robot Description Format)
Dodaje semantyczne informacje:
- **Groups** - grupy stawów (np. "left_arm", "right_hand")
- **End Effectors** - definicje chwytaków
- **Virtual Joints** - połączenia z zewnętrznym światem
- **Disabled Collisions** - pary linków, które mogą się kolidować
- **Poses** - nazwane pozycje robota

#### Przykładowy fragment SRDF:
```xml
<group name="left_arm">
    <joint name="l_shoulder_pitch" />
    <joint name="l_shoulder_roll" />
    <joint name="l_shoulder_yaw" />
    <joint name="l_elbow" />
</group>

<group_state name="left_arm_home" group="left_arm">
    <joint name="l_shoulder_pitch" value="0.0" />
    <joint name="l_shoulder_roll" value="0.0" />
    <joint name="l_shoulder_yaw" value="0.0" />
    <joint name="l_elbow" value="0.0" />
</group_state>
```

## Debugowanie

### Najczęstsze problemy i rozwiązania

#### Problem 1: "No planning scene monitor"
```
[ERROR] [move_group]: Unable to get planning scene
```

**Diagnoza:**
```bash
# Sprawdź czy działa robot_state_publisher
ros2 node list | grep robot_state_publisher

# Sprawdź topic joint_states
ros2 topic hz /joint_states
```

**Rozwiązanie:**
- Upewnij się, że `robot_state_publisher` jest uruchomiony
- Sprawdź czy `/joint_states` publikuje dane
- Zweryfikuj URDF robota

#### Problem 2: "Planning failed: INVALID_START_STATE"
```
[ERROR] [move_group]: Planning failed: INVALID_START_STATE
```

**Przyczyny:**
- Aktualna pozycja robota jest w kolizji
- Stan stawów jest poza limitami
- Planning Scene nie jest zsynchronizowany

**Rozwiązanie:**
```python
# Sprawdź czy stan jest poprawny
robot_state = moveit.get_robot_state()
print(f"Joint values: {robot_state.joint_positions}")

# Sprawdź kolizje
planning_scene = moveit.get_planning_scene()
if planning_scene.is_state_colliding(robot_state):
    print("Robot is in collision!")
```

#### Problem 3: "No IK solution found"
```
[ERROR] [move_group]: No IK solution found for pose
```

**Przyczyny:**
- Pozycja docelowa poza workspace robota
- Orientacja niemożliwa do osiągnięcia
- Za mało iteracji solvera IK

**Rozwiązanie:**
- Zweryfikuj czy pozycja jest osiągalna
- Zwiększ `timeout` dla IK solvera w `kinematics.yaml`
- Użyj `setApproximateJointValueTarget()` zamiast dokładnej pozy

### Narzędzia debugowania

```bash
# Wizualizacja TF frames
ros2 run tf2_tools view_frames

# Monitoring topic'ów
ros2 topic list
ros2 topic hz /topic_name
ros2 topic echo /topic_name

# Inspekcja parametrów
ros2 param list /move_group
ros2 param get /move_group planning_plugin

# Logi
ros2 node info /move_group
```

## Zadania do wykonania

### Zadanie 1: Podstawowe planowanie (★☆☆)
Napisz program, który:
1. Planuje ruch do pozycji "ready"
2. Dodaje prostopadłościenną przeszkodę przed robotem
3. Planuje ruch omijający przeszkodę
4. Usuwa przeszkodę
5. Wraca do pozycji "home"

**Wskazówki:**
```python
from moveit.planning import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

# Dodawanie przeszkody
planning_scene = PlanningSceneInterface()
box_pose = PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.position.x = 0.5
box_pose.pose.position.y = 0.0
box_pose.pose.position.z = 0.5
planning_scene.add_box("obstacle", box_pose, size=(0.1, 0.1, 0.1))
```

### Zadanie 2: Trajektoria kartezjańska (★★☆)
Napisz program rysujący kwadrat w przestrzeni:
- Wielkość kwadratu: 10cm x 10cm
- Płaszczyzna: pionowa przed robotem
- Używaj `compute_cartesian_path()` do generowania gładkiej trajektorii

**Struktura:**
1. Zdefiniuj 4 punkty narożników kwadratu
2. Dla każdego punktu oblicz trajektorię kartezjańską
3. Wykonaj trajektorię
4. Przejdź do następnego punktu

### Zadanie 3: Servo + Joystick (★★★)
Zaprogramuj sterowanie robotem joystickiem:
1. Wykorzystaj `moveit_servo`
2. Mapuj osie joysticka na komendy twist
3. Dodaj przycisk "deadman switch" (bezpieczeństwo)
4. Ogranicz prędkość do bezpiecznych wartości

**Rozszerzenie:**
- Dodaj tryby sterowania (world frame / end-effector frame)
- Wizualizuj aktualny target w RViz

### Zadanie 4: Projekt końcowy - Manipulacja obiektem (★★★)
Zaimplementuj kompletny cykl pick-and-place:
1. Wykryj obiekt (symuluj pozycję)
2. Zaplanuj podejście do obiektu
3. Otwórz chwytaka
4. Zbliż się do obiektu
5. Zamknij chwytaka
6. Podnieś obiekt
7. Przenieś do miejsca docelowego
8. Odłóż obiekt

**Wymagania:**
- Dodaj obiekt do planning scene (attached/detached)
- Obsłuż sytuacje błędów (brak IK, kolizje)
- Loguj wszystkie kroki operacji

## Zasoby dodatkowe

### Dokumentacja online
- [MoveIt 2 API Documentation](https://moveit.picknik.ai/main/api/html/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Simulation](https://gazebosim.org/docs)

### Polecane tutoriale wideo
- MoveIt 2 Basics (YouTube: PickNik Robotics)
- ROS 2 for Beginners (YouTube: The Construct)

### Literatura
- "A Gentle Introduction to ROS" - Jason M. O'Kane
- "Programming Robots with ROS" - Morgan Quigley et al.

### Kontakt i pomoc
W razie problemów:
1. Sprawdź [GitHub Issues](https://github.com/ros-planning/moveit2/issues)
2. Zadaj pytanie na [ROS Discourse](https://discourse.ros.org/)
3. Skontaktuj się z prowadzącym zajęcia

---

**Powodzenia w nauce! 🤖**
