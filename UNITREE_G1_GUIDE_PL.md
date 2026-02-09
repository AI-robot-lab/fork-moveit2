# MoveIt 2 dla Robota Unitree G1 EDU - Przewodnik Praktyczny

## Spis treści
1. [Wprowadzenie do Unitree G1 EDU](#wprowadzenie-do-unitree-g1-edu)
2. [Architektura systemu](#architektura-systemu)
3. [Konfiguracja MoveIt dla G1](#konfiguracja-moveit-dla-g1)
4. [Podstawowe operacje](#podstawowe-operacje)
5. [Zaawansowane zastosowania](#zaawansowane-zastosowania)
6. [Projekty przykładowe](#projekty-przykładowe)
7. [Najlepsze praktyki](#najlepsze-praktyki)

## Wprowadzenie do Unitree G1 EDU

### Specyfikacja robota

**Unitree G1 EDU** to zaawansowany robot humanoidalny zaprojektowany do badań i edukacji. Kluczowe parametry:

#### Stopnie swobody (Degrees of Freedom - DOF)
```
Całkowita liczba DOF: 23+
├── Głowa: 3 DOF (pitch, roll, yaw)
├── Tułów: 3 DOF (pitch, roll, yaw)
├── Lewe ramię: 7 DOF
│   ├── Bark: 3 DOF (shoulder pitch, roll, yaw)
│   ├── Łokieć: 1 DOF
│   └── Nadgarstek: 3 DOF (wrist pitch, roll, yaw)
├── Prawe ramię: 7 DOF (symetrycznie do lewego)
├── Lewa noga: 6 DOF
│   ├── Biodro: 3 DOF (hip pitch, roll, yaw)
│   ├── Kolano: 1 DOF
│   └── Staw skokowy: 2 DOF (ankle pitch, roll)
└── Prawa noga: 6 DOF (symetrycznie do lewej)
```

#### Parametry fizyczne
- **Wysokość:** ~130 cm
- **Waga:** ~35 kg
- **Udźwig ramienia:** ~3 kg
- **Zakres ruchu ramion:** ~360° (niektóre stawy)
- **Prędkość maksymalna stawów:** ~180°/s

#### Czujniki
- **IMU** (Inertial Measurement Unit) - orientacja i przyspieszenie tułowia
- **Enkodery stawów** - precyzyjna pozycja każdego stawu
- **Czujniki momentu** - detekcja sił w stawach
- **Kamery** (opcjonalnie) - percepcja otoczenia
- **LiDAR** (opcjonalnie) - skanowanie 3D otoczenia

### Dlaczego MoveIt 2 dla G1?

Robot humanoidalny jak G1 ma 23+ stopni swobody. Oznacza to:

**Bez MoveIt:**
- Musisz ręcznie obliczać kinematykę odwrotną dla 7 DOF ramienia
- Trudno jest uniknąć samokoli (robot może uderzyć ręką w nogę)
- Planowanie trajektorii wymaga rozważenia milionów możliwych konfiguracji
- Każda operacja wymaga zaawansowanej matematyki

**Z MoveIt:**
- ✓ Automatyczne obliczenia kinematyki
- ✓ Wykrywanie kolizji między częściami robota
- ✓ Inteligentne planowanie trajektorii
- ✓ Prosty interfejs programistyczny
- ✓ Bezpieczeństwo wbudowane w system

## Architektura systemu

### Schemat połączeń

```
┌────────────────────────────────────────────────────────────┐
│                    Twoja Aplikacja                          │
│            (Python/C++ kod sterujący)                       │
└──────────────────┬─────────────────────────────────────────┘
                   │ MoveIt Planning Interface
                   ↓
┌────────────────────────────────────────────────────────────┐
│                    MoveIt Move Group                        │
│  • Planowanie trajektorii dla ramion                       │
│  • Koordynacja ruchu całego ciała                          │
│  • Wykrywanie kolizji                                      │
└──────────────────┬─────────────────────────────────────────┘
                   │ Joint Trajectory Commands
                   ↓
┌────────────────────────────────────────────────────────────┐
│              Unitree G1 Controller Node                     │
│  • Konwersja trajektorii MoveIt → komendy G1               │
│  • Monitorowanie stanu robota                              │
│  • Bezpieczeństwo i limity                                 │
└──────────────────┬─────────────────────────────────────────┘
                   │ CAN / Ethernet Protocol
                   ↓
┌────────────────────────────────────────────────────────────┐
│                  Unitree G1 EDU Robot                       │
│              (Fizyczny robot / Symulacja)                   │
└────────────────────────────────────────────────────────────┘
```

### Grupy planowania dla G1

W konfiguracji MoveIt dla G1 definiujemy kilka **Planning Groups**:

```yaml
# Przykładowe grupy w G1 SRDF

# Lewe ramię (7 DOF)
left_arm:
  joints:
    - l_shoulder_pitch
    - l_shoulder_roll  
    - l_shoulder_yaw
    - l_elbow
    - l_wrist_roll
    - l_wrist_pitch
    - l_wrist_yaw

# Prawe ramię (7 DOF)
right_arm:
  joints:
    - r_shoulder_pitch
    - r_shoulder_roll
    - r_shoulder_yaw
    - r_elbow
    - r_wrist_roll
    - r_wrist_pitch
    - r_wrist_yaw

# Górna część ciała (ramiona + tułów)
upper_body:
  groups:
    - left_arm
    - right_arm
    - torso

# Całe ciało
whole_body:
  groups:
    - upper_body
    - left_leg
    - right_leg
```

**Dlaczego różne grupy?**
- `left_arm` / `right_arm` - gdy sterujemy tylko jedną ręką
- `upper_body` - dla zadań wymagających koordynacji obu rąk
- `whole_body` - dla złożonych zadań mobilnych (chodzenie + manipulacja)

## Konfiguracja MoveIt dla G1

### Struktura pakietu konfiguracyjnego

```
unitree_g1_moveit_config/
├── config/
│   ├── g1.srdf                    # Definicja grup i kolizji
│   ├── joint_limits.yaml          # Limity stawów
│   ├── kinematics.yaml            # Konfiguracja IK solverów
│   ├── ompl_planning.yaml         # Parametry planowania OMPL
│   ├── servo_config.yaml          # Konfiguracja Servo
│   └── controllers.yaml           # Konfiguracja kontrolerów
├── launch/
│   ├── demo.launch.py             # Demo w RViz
│   ├── g1_moveit.launch.py        # Główny launch dla G1
│   └── g1_sim.launch.py           # Uruchomienie z symulacją Gazebo
└── urdf/
    └── g1.urdf.xacro              # Model kinematyczny robota
```

### Kluczowe parametry konfiguracji

#### 1. Limity stawów (joint_limits.yaml)
```yaml
# Przykładowe limity dla prawego ramienia
joint_limits:
  r_shoulder_pitch:
    has_position_limits: true
    min_position: -3.14        # -180° (rad)
    max_position: 3.14         # +180° (rad)
    has_velocity_limits: true
    max_velocity: 3.14         # 180°/s (rad/s)
    has_acceleration_limits: true
    max_acceleration: 6.28     # Przyspieszenie (rad/s²)
  
  r_elbow:
    has_position_limits: true
    min_position: 0.0          # Łokieć nie może się wyginać "na drugą stronę"
    max_position: 2.79         # ~160° (rad)
    max_velocity: 2.5
    max_acceleration: 5.0
```

**Dlaczego to jest ważne?**
- Chroni fizyczny robot przed uszkodzeniem
- Zapewnia realistyczne trajektorie (robot nie próbuje niemożliwych ruchów)
- Ogranicza prędkości dla bezpieczeństwa ludzi w pobliżu

#### 2. Konfiguracja kinematyki (kinematics.yaml)
```yaml
# Solver kinematyki odwrotnej dla prawego ramienia
right_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005  # Precyzja [m]
  kinematics_solver_timeout: 0.05             # Timeout [s]
  kinematics_solver_attempts: 3               # Liczba prób

# Dla bardziej złożonych grup używamy TracIK (bardziej niezawodny)
upper_body:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.1
  solve_type: Speed                           # Speed / Distance / Manipulation
```

**Wybór solvera:**
- **KDL** - szybki, dobry dla prostych grup (pojedyncze ramię)
- **TracIK** - wolniejszy, ale bardziej niezawodny dla trudnych pozycji
- **IKFast** - najszybszy, ale wymaga wygenerowania dla konkretnego modelu

#### 3. Parametry planowania (ompl_planning.yaml)
```yaml
# Konfiguracja plannerów OMPL
planner_configs:
  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0                    # Auto
    
  RRTstar:
    type: geometric::RRTstar
    range: 0.0
    goal_bias: 0.05               # Szansa kierowania się do celu
    
# Ustawienia dla grupy ramienia
right_arm:
  planner_configs:
    - RRTConnect                  # Szybki, pierwszy wybór
    - RRTstar                     # Wolniejszy, ale optymalizuje trajektorię
  projection_evaluator: joints(r_shoulder_pitch,r_shoulder_roll,r_elbow)
  longest_valid_segment_fraction: 0.01
```

**Różne plannery dla różnych zadań:**
- **RRTConnect** - szybkie planowanie, dobry dla prostych zadań
- **RRTstar** - optymalizuje długość trajektorii
- **PRM** - dobrze dla wielokrotnych zapytań w tym samym środowisku

## Podstawowe operacje

### 1. Inicjalizacja systemu

```python
#!/usr/bin/env python3
"""
Inicjalizacja MoveIt dla robota Unitree G1.
Ten skrypt pokazuje podstawową konfigurację i weryfikację systemu.
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from sensor_msgs.msg import JointState
import time

class G1MoveItInitializer(Node):
    """Node inicjalizujący i weryfikujący MoveIt dla G1."""
    
    def __init__(self):
        super().__init__('g1_moveit_initializer')
        self.get_logger().info('🤖 Inicjalizacja MoveIt dla Unitree G1...')
        
        # Krok 1: Inicjalizacja MoveItPy
        # To może zająć kilka sekund - ładuje model robota, planning scene, itp.
        try:
            self.moveit = MoveItPy(node_name="g1_moveit_node")
            self.get_logger().info('✓ MoveItPy zainicjalizowany')
        except Exception as e:
            self.get_logger().error(f'✗ Błąd inicjalizacji MoveItPy: {e}')
            return
        
        # Krok 2: Pobierz dostępne grupy planowania
        available_groups = self.moveit.get_group_names()
        self.get_logger().info(f'Dostępne grupy: {available_groups}')
        
        # Krok 3: Inicjalizuj planning components dla obu ramion
        self.left_arm = None
        self.right_arm = None
        
        if 'left_arm' in available_groups:
            self.left_arm = self.moveit.get_planning_component('left_arm')
            self.get_logger().info('✓ Lewe ramię gotowe')
        else:
            self.get_logger().warn('⚠ Grupa "left_arm" nie znaleziona')
            
        if 'right_arm' in available_groups:
            self.right_arm = self.moveit.get_planning_component('right_arm')
            self.get_logger().info('✓ Prawe ramię gotowe')
        else:
            self.get_logger().warn('⚠ Grupa "right_arm" nie znaleziona')
        
        # Krok 4: Weryfikacja stanu robota
        self.verify_robot_state()
        
        self.get_logger().info('✓ Inicjalizacja zakończona!')
    
    def verify_robot_state(self):
        """Sprawdza czy stan robota jest poprawny."""
        self.get_logger().info('Weryfikacja stanu robota...')
        
        try:
            # Pobierz aktualny stan robota
            robot_state = self.moveit.get_robot_state()
            
            # Sprawdź czy stan jest w dozwolonych granicach
            # (czy żaden staw nie jest poza limitami)
            if robot_state:
                joint_names = robot_state.joint_positions.keys()
                self.get_logger().info(f'Robot ma {len(joint_names)} stawów')
                
                # Przykład: wyświetl pozycje stawów prawego ramienia
                right_arm_joints = [j for j in joint_names if j.startswith('r_')]
                self.get_logger().info(f'Stawy prawego ramienia: {right_arm_joints}')
                
                for joint in right_arm_joints[:3]:  # Pierwsze 3 stawy
                    pos = robot_state.joint_positions.get(joint, 0.0)
                    self.get_logger().info(f'  {joint}: {pos:.3f} rad')
                
                self.get_logger().info('✓ Stan robota poprawny')
            else:
                self.get_logger().error('✗ Nie można pobrać stanu robota')
                
        except Exception as e:
            self.get_logger().error(f'✗ Błąd weryfikacji: {e}')

def main():
    rclpy.init()
    initializer = G1MoveItInitializer()
    
    # Trzymaj node aktywny
    try:
        rclpy.spin(initializer)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Podstawowy ruch ramienia

```python
#!/usr/bin/env python3
"""
Podstawowe sterowanie ramieniem robota G1.
Demonstruje planowanie i wykonywanie prostych ruchów.
"""

import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose, Point, Quaternion
import time

def create_reach_pose(x, y, z):
    """
    Tworzy pose dla chwytania obiektów.
    
    W robocie humanoidalnym G1, orientacja chwytaka jest kluczowa.
    Ta funkcja tworzy pose z chwytakiem skierowanym w dół (naturalna
    orientacja do chwytania obiektów na stole).
    
    Args:
        x, y, z: Współrzędne w układzie base robota [metry]
        
    Returns:
        Pose: Pozycja i orientacja end-effectora
    """
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    
    # Quaternion dla orientacji "chwytka w dół"
    # To jest uproszczenie - w rzeczywistości może wymagać dostosowania
    pose.orientation = Quaternion(x=0.707, y=0.0, z=0.0, w=0.707)
    
    return pose

def main():
    rclpy.init()
    
    # Inicjalizacja
    moveit = MoveItPy(node_name="g1_arm_control")
    right_arm = moveit.get_planning_component("right_arm")
    
    print("🤖 Unitree G1 - Sterowanie prawym ramieniem")
    print("=" * 50)
    
    # Scenariusz 1: Ruch do nazwanej pozycji
    print("\n📍 Scenariusz 1: Ruch do pozycji 'home'")
    right_arm.set_start_state_to_current_state()
    right_arm.set_goal_state(configuration_name="home")
    
    print("  ⏳ Planowanie...")
    plan_result = right_arm.plan()
    
    if plan_result:
        print("  ✓ Trajektoria zaplanowana!")
        trajectory = right_arm.get_plan_trajectory()
        print(f"  📊 Długość trajektorii: {len(trajectory.joint_trajectory.points)} punktów")
        
        # Wykonaj ruch
        print("  🏃 Wykonywanie ruchu...")
        success = moveit.execute(trajectory, blocking=True)
        
        if success:
            print("  ✓ Ruch wykonany!")
        else:
            print("  ✗ Błąd wykonania!")
    else:
        print("  ✗ Planowanie nie powiodło się!")
    
    time.sleep(2)
    
    # Scenariusz 2: Ruch do pozycji w przestrzeni
    print("\n📍 Scenariusz 2: Sięgnięcie do punktu przed robotem")
    
    # Pozycja: 40cm przed robotem, 20cm w prawo, 80cm w górę
    target_pose = create_reach_pose(x=0.4, y=-0.2, z=0.8)
    
    right_arm.set_start_state_to_current_state()
    right_arm.set_goal_state(
        pose_stamped_msg=target_pose,
        pose_link="r_hand_link"  # Nazwa end-effectora z URDF
    )
    
    print(f"  🎯 Cel: x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}")
    print("  ⏳ Planowanie (może zająć chwilę - rozwiązywanie IK)...")
    
    plan_result = right_arm.plan()
    
    if plan_result:
        print("  ✓ IK rozwiązana! Trajektoria zaplanowana!")
        trajectory = right_arm.get_plan_trajectory()
        
        print("  🏃 Wykonywanie ruchu...")
        success = moveit.execute(trajectory, blocking=True)
        
        if success:
            print("  ✓ Ramię osiągnęło cel!")
        else:
            print("  ✗ Błąd wykonania!")
    else:
        print("  ✗ Nie udało się zaplanować trajektorii")
        print("  💡 Możliwe przyczyny:")
        print("     - Pozycja poza zasięgiem ramienia")
        print("     - Brak rozwiązania IK dla tej orientacji")
        print("     - Kolizja z ciałem robota")
    
    print("\n" + "=" * 50)
    print("✓ Program zakończony")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Koordynacja obu ramion

```python
#!/usr/bin/env python3
"""
Koordynacja ruchu obu ramion robota G1.
Pokazuje jak planować i wykonywać zsynchronizowane ruchy.
"""

import rclpy
from moveit.planning import MoveItPy
import time

def plan_dual_arm_motion(moveit):
    """
    Planuje ruch obu ramion jednocześnie.
    
    To jest bardziej skomplikowane niż planowanie dla jednego ramienia,
    ponieważ musimy uwzględnić:
    - Kolizje między ramionami
    - Synchronizację czasową
    - Wspólny workspace
    
    Args:
        moveit: Instancja MoveItPy
        
    Returns:
        tuple: (left_trajectory, right_trajectory) lub (None, None) w przypadku błędu
    """
    
    # Opcja 1: Planowanie niezależne
    # Planujemy dla każdego ramienia osobno, a potem synchronizujemy
    
    print("📍 Planowanie dla lewego ramienia...")
    left_arm = moveit.get_planning_component("left_arm")
    left_arm.set_start_state_to_current_state()
    left_arm.set_goal_state(configuration_name="left_arm_extended")
    
    left_plan = left_arm.plan()
    
    print("📍 Planowanie dla prawego ramienia...")
    right_arm = moveit.get_planning_component("right_arm")
    right_arm.set_start_state_to_current_state()
    right_arm.set_goal_state(configuration_name="right_arm_extended")
    
    right_plan = right_arm.plan()
    
    if left_plan and right_plan:
        left_traj = left_arm.get_plan_trajectory()
        right_traj = right_arm.get_plan_trajectory()
        
        print("✓ Obie trajektorie zaplanowane!")
        print(f"  Lewe ramię: {len(left_traj.joint_trajectory.points)} punktów")
        print(f"  Prawe ramię: {len(right_traj.joint_trajectory.points)} punktów")
        
        return left_traj, right_traj
    else:
        print("✗ Błąd planowania!")
        if not left_plan:
            print("  - Nie udało się zaplanować dla lewego ramienia")
        if not right_plan:
            print("  - Nie udało się zaplanować dla prawego ramienia")
        return None, None

def main():
    rclpy.init()
    
    moveit = MoveItPy(node_name="g1_dual_arm_control")
    
    print("🤖 Unitree G1 - Koordynacja obu ramion")
    print("=" * 60)
    
    # Scenariusz: Rozłóż ramiona na boki (klasyczna poza "T-pose")
    print("\n📍 Scenariusz: T-pose (rozłożenie ramion)")
    print("  ⏳ Planowanie...")
    
    left_traj, right_traj = plan_dual_arm_motion(moveit)
    
    if left_traj and right_traj:
        # Opcja A: Wykonaj sekwencyjnie (bezpieczniejsze)
        print("\n  🏃 Wykonywanie sekwencyjne (jedno po drugim)...")
        print("    → Lewe ramię...")
        moveit.execute(left_traj, blocking=True)
        time.sleep(1)
        print("    → Prawe ramię...")
        moveit.execute(right_traj, blocking=True)
        print("  ✓ Gotowe!")
        
        # Opcja B: Wykonaj równolegle (wymaga wsparcia kontrolera)
        # W prawdziwej aplikacji musisz użyć grupy "upper_body"
        # aby wykonać ruch obu ramion jednocześnie
        
        """
        print("\n  🏃 Wykonywanie równoległe...")
        upper_body = moveit.get_planning_component("upper_body")
        # Połącz trajektorie w jedną
        combined_trajectory = merge_trajectories(left_traj, right_traj)
        moveit.execute(combined_trajectory, blocking=True)
        print("  ✓ Gotowe!")
        """
    else:
        print("✗ Nie można wykonać ruchu - błąd planowania")
    
    print("\n" + "=" * 60)
    print("✓ Program zakończony")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Zaawansowane zastosowania

### 1. Teleoperacja z MoveIt Servo

**MoveIt Servo** pozwala na sterowanie robotem w czasie rzeczywistym poprzez komendy prędkości. Jest idealny do:
- Sterowania joystickiem
- VR/AR teleoperacji
- Zdalnego sterowania
- Śledzenia ruchu człowieka

Zobacz szczegółowe przykłady w `moveit_ros/moveit_servo/demos/` z polskimi komentarzami.

### 2. Integracja z percepcją

```python
"""
Integracja MoveIt z systemem percepcji (kamery, LiDAR).
"""

from moveit.planning import PlanningSceneInterface
from sensor_msgs.msg import PointCloud2

class PerceptionIntegration:
    """
    Integruje dane z czujników do Planning Scene.
    
    W robocie humanoidalnym percepcja jest kluczowa do:
    - Wykrywania przeszkód w otoczeniu
    - Lokalizacji obiektów do manipulacji
    - Nawigacji w dynamicznym środowisku
    """
    
    def __init__(self, moveit):
        self.moveit = moveit
        self.planning_scene = PlanningSceneInterface()
        
    def add_detected_obstacles(self, point_cloud):
        """
        Dodaje wykryte przeszkody do Planning Scene.
        
        Args:
            point_cloud: Chmura punktów z LiDAR/kamery głębi
        """
        # W prawdziwej aplikacji: przetwórz chmurę punktów
        # i wyekstrahuj przeszkody jako prostopadłościany/cylindry
        
        # Przykład: dodanie stołu wykrytego przez percepcję
        from geometry_msgs.msg import PoseStamped
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0.7
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.4
        table_pose.pose.orientation.w = 1.0
        
        self.planning_scene.add_box(
            "detected_table",
            table_pose,
            size=(1.0, 0.8, 0.02)  # szerokość, głębokość, wysokość
        )
        
        print("✓ Dodano wykryty stół do Planning Scene")
```

## Projekty przykładowe

### Projekt 1: Pick-and-Place dla obiektu na stole

```python
#!/usr/bin/env python3
"""
Kompletny cykl pick-and-place dla robota G1.
Robot podnosi obiekt ze stołu i przenosi go w inne miejsce.
"""

import rclpy
from moveit.planning import MoveItPy, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
import time

class G1PickAndPlace:
    """Implementacja operacji pick-and-place dla Unitree G1."""
    
    def __init__(self):
        rclpy.init()
        
        # Inicjalizacja MoveIt
        self.moveit = MoveItPy(node_name="g1_pick_place")
        self.right_arm = self.moveit.get_planning_component("right_arm")
        self.planning_scene = PlanningSceneInterface()
        
        # Parametry chwytaka (specyficzne dla G1)
        self.gripper_open = 0.08    # 8cm otwarcie
        self.gripper_closed = 0.01  # prawie zamknięty
        
        print("✓ G1 Pick-and-Place gotowy")
    
    def setup_scene(self):
        """Konfiguruje scenę: stół + obiekt do podniesienia."""
        
        # Dodaj stół
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0.6
        table_pose.pose.position.z = 0.4
        table_pose.pose.orientation.w = 1.0
        
        self.planning_scene.add_box(
            "table",
            table_pose,
            size=(0.8, 1.0, 0.02)
        )
        
        # Dodaj obiekt (np. kubek)
        object_pose = PoseStamped()
        object_pose.header.frame_id = "base_link"
        object_pose.pose.position.x = 0.6
        object_pose.pose.position.y = 0.2
        object_pose.pose.position.z = 0.5  # Na stole
        object_pose.pose.orientation.w = 1.0
        
        self.planning_scene.add_cylinder(
            "object",
            object_pose,
            height=0.15,
            radius=0.04
        )
        
        time.sleep(0.5)  # Daj czas na update Planning Scene
        print("✓ Scena skonfigurowana (stół + obiekt)")
    
    def approach_object(self, object_x, object_y, object_z):
        """
        Podejdź do obiektu (pozycja pre-grasp).
        
        Najpierw podchodzimy z góry, aby uniknąć kolizji.
        """
        print("  → Podejście do obiektu...")
        
        # Pozycja 10cm nad obiektem
        approach_pose = Pose()
        approach_pose.position.x = object_x
        approach_pose.position.y = object_y
        approach_pose.position.z = object_z + 0.1  # 10cm wyżej
        approach_pose.orientation.x = 0.707
        approach_pose.orientation.w = 0.707
        
        self.right_arm.set_start_state_to_current_state()
        self.right_arm.set_goal_state(
            pose_stamped_msg=approach_pose,
            pose_link="r_hand_link"
        )
        
        plan_result = self.right_arm.plan()
        if plan_result:
            trajectory = self.right_arm.get_plan_trajectory()
            self.moveit.execute(trajectory, blocking=True)
            print("    ✓ W pozycji podejścia")
            return True
        else:
            print("    ✗ Nie można zaplanować podejścia")
            return False
    
    def grasp_object(self, object_x, object_y, object_z):
        """Zbliż się i chwyć obiekt."""
        print("  → Chwytanie obiektu...")
        
        # Pozycja chwytania (wysokość obiektu)
        grasp_pose = Pose()
        grasp_pose.position.x = object_x
        grasp_pose.position.y = object_y
        grasp_pose.position.z = object_z
        grasp_pose.orientation.x = 0.707
        grasp_pose.orientation.w = 0.707
        
        # Otwórz chwytaka przed zbliżeniem
        self.control_gripper(self.gripper_open)
        
        # Zbliż się do obiektu
        self.right_arm.set_start_state_to_current_state()
        self.right_arm.set_goal_state(
            pose_stamped_msg=grasp_pose,
            pose_link="r_hand_link"
        )
        
        plan_result = self.right_arm.plan()
        if plan_result:
            trajectory = self.right_arm.get_plan_trajectory()
            self.moveit.execute(trajectory, blocking=True)
            
            # Zamknij chwytaka
            time.sleep(0.5)
            self.control_gripper(self.gripper_closed)
            time.sleep(0.5)
            
            # Dołącz obiekt do robota w Planning Scene
            # Od teraz obiekt porusza się z ręką robota
            touch_links = ["r_hand_link", "r_finger_1", "r_finger_2"]
            self.planning_scene.attach_box(
                "r_hand_link",
                "object",
                touch_links=touch_links
            )
            
            print("    ✓ Obiekt schwytany!")
            return True
        else:
            print("    ✗ Nie można zbliżyć się do obiektu")
            return False
    
    def place_object(self, target_x, target_y, target_z):
        """Przenieś i odłóż obiekt w nowej lokalizacji."""
        print("  → Przenoszenie obiektu...")
        
        # Pozycja nad miejscem docelowym
        place_approach = Pose()
        place_approach.position.x = target_x
        place_approach.position.y = target_y
        place_approach.position.z = target_z + 0.1
        place_approach.orientation.x = 0.707
        place_approach.orientation.w = 0.707
        
        self.right_arm.set_start_state_to_current_state()
        self.right_arm.set_goal_state(
            pose_stamped_msg=place_approach,
            pose_link="r_hand_link"
        )
        
        plan_result = self.right_arm.plan()
        if plan_result:
            trajectory = self.right_arm.get_plan_trajectory()
            self.moveit.execute(trajectory, blocking=True)
            
            # Opuść do wysokości docelowej
            place_pose = Pose()
            place_pose.position.x = target_x
            place_pose.position.y = target_y
            place_pose.position.z = target_z
            place_pose.orientation.x = 0.707
            place_pose.orientation.w = 0.707
            
            self.right_arm.set_start_state_to_current_state()
            self.right_arm.set_goal_state(
                pose_stamped_msg=place_pose,
                pose_link="r_hand_link"
            )
            
            plan_result = self.right_arm.plan()
            if plan_result:
                trajectory = self.right_arm.get_plan_trajectory()
                self.moveit.execute(trajectory, blocking=True)
                
                # Otwórz chwytaka i odłącz obiekt
                time.sleep(0.5)
                self.control_gripper(self.gripper_open)
                self.planning_scene.detach_object("r_hand_link", "object")
                
                print("    ✓ Obiekt odłożony!")
                return True
        
        print("    ✗ Nie można przenieść obiektu")
        return False
    
    def control_gripper(self, opening):
        """
        Steruje chwytakiem robota.
        
        W prawdziwej implementacji wyślesz komendę do kontrolera chwytaka.
        """
        # Placeholder - zaimplementuj według specyfikacji G1
        print(f"    🤏 Chwytaka: {opening*1000:.0f}mm")
        # Przykład: publikuj na topic kontrolera chwytaka
        # self.gripper_pub.publish(GripperCommand(position=opening))
    
    def execute_pick_and_place(self):
        """Główna sekwencja pick-and-place."""
        print("\n🤖 Start: Pick-and-Place")
        print("=" * 60)
        
        # Krok 1: Przygotuj scenę
        self.setup_scene()
        
        # Krok 2: Pozycja obiektu (w rzeczywistości z percepcji)
        object_x, object_y, object_z = 0.6, 0.2, 0.5
        
        # Krok 3: Podejdź
        if not self.approach_object(object_x, object_y, object_z):
            print("✗ Operacja przerwana")
            return False
        
        # Krok 4: Chwyć
        if not self.grasp_object(object_x, object_y, object_z):
            print("✗ Operacja przerwana")
            return False
        
        # Krok 5: Przenieś
        target_x, target_y, target_z = 0.6, -0.3, 0.5
        if not self.place_object(target_x, target_y, target_z):
            print("✗ Operacja przerwana")
            return False
        
        # Krok 6: Wróć do pozycji domowej
        print("  → Powrót do pozycji home...")
        self.right_arm.set_start_state_to_current_state()
        self.right_arm.set_goal_state(configuration_name="home")
        plan_result = self.right_arm.plan()
        if plan_result:
            trajectory = self.right_arm.get_plan_trajectory()
            self.moveit.execute(trajectory, blocking=True)
        
        print("\n✓ Pick-and-Place zakończony sukcesem!")
        print("=" * 60)
        return True

def main():
    picker = G1PickAndPlace()
    picker.execute_pick_and_place()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Najlepsze praktyki

### Bezpieczeństwo

1. **Zawsze testuj w symulacji najpierw**
   ```bash
   # Uruchom Gazebo symulację przed testami na prawdziwym robocie
   ros2 launch unitree_g1_moveit_config g1_sim.launch.py
   ```

2. **Używaj limitów prędkości i przyspieszeń**
   - Nigdy nie przekraczaj limitów zdefiniowanych w `joint_limits.yaml`
   - Dla pracy z ludźmi: zmniejsz `max_velocity` o 50%

3. **Monitoruj siły w stawach**
   ```python
   # Sprawdzaj obciążenie stawów
   robot_state = moveit.get_robot_state()
   if robot_state.joint_efforts['r_shoulder_pitch'] > MAX_TORQUE:
       # Stop robot!
       pass
   ```

### Optymalizacja wydajności

1. **Wybieraj odpowiedni planner dla zadania**
   - RRTConnect - dla szybkiego planowania
   - RRTstar - gdy jakość trajektorii jest kluczowa
   - Pilz - dla prostych, przemysłowych ruchów

2. **Używaj named targets gdzie to możliwe**
   - Szybsze niż rozwiązywanie IK
   - Bardziej przewidywalne

3. **Cachuj rozwiązania IK**
   - Dla powtarzalnych zadań zapisuj rozwiązania

### Debugowanie

```python
# Pomocnicze narzędzia debugowania

def visualize_target_pose(pose, moveit):
    """Wizualizuj cel w RViz przed planowaniem."""
    # Dodaj marker do RViz
    from visualization_msgs.msg import Marker
    # ... kod wizualizacji ...

def log_planning_time(func):
    """Dekorator mierzący czas planowania."""
    import time
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        elapsed = time.time() - start
        print(f"⏱️ Planowanie zajęło: {elapsed:.2f}s")
        return result
    return wrapper
```

## Podsumowanie

MoveIt 2 z robotem Unitree G1 EDU to potężna kombinacja do:
- ✓ Nauki zaawansowanej robotyki
- ✓ Badań nad manipulacją humanoidalną
- ✓ Prototypowania aplikacji service robotics
- ✓ Rozwijania algorytmów AI dla robotów

**Kluczowe wnioski:**
1. Używaj odpowiednich grup planowania (left_arm, right_arm, upper_body)
2. Testuj w symulacji przed uruchomieniem na prawdziwym robocie
3. Zawsze monitoruj stan robota i wykrywaj kolizje
4. Optymalizuj parametry planowania dla swojego zastosowania

**Następne kroki:**
- Eksperymentuj z przykładami w tym przewodniku
- Przeczytaj [Przewodnik studenta](./STUDENT_GUIDE_PL.md)
- Dołącz do społeczności MoveIt i G1

---

**Powodzenia w projektach! 🤖🚀**
