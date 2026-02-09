# Integracja MoveIt 2 z Robotem Unitree G1 EDU

## Wprowadzenie do Unitree G1 EDU

**Unitree G1 EDU** to humanoidalny robot badawczy zaprojektowany dla edukacji i rozwoju algorytmów robotyki. Robot posiada:
- **Wysokość:** ~130 cm
- **Ramiona:** 2 ramiona po 7 stopni swobody każde (podobne do Franka Panda)
- **Dłonie:** Chwytaki wielopalcowe z możliwością precyzyjnej manipulacji
- **Nogi:** 2 nogi z 6 stopniami swobody każda (dla lokomocji)
- **Głowa:** Z kamerami stereowizyjnymi i sensorami głębi
- **Tułów:** Z IMU i możliwością pochylania/rotacji

## Dlaczego MoveIt 2 dla Unitree G1?

### 1. Złożoność kinematyczna
Humanoid ma **ponad 20 stopni swobody**. Ręczne obliczanie kinematyki byłoby praktycznie niemożliwe.

**MoveIt 2 rozwiązuje:**
- Automatyczne rozwiązywanie IK dla ramion
- Koordynacja wielu grup planowania (lewe ramię, prawe ramię, nogi)
- Planowanie całego ciała (whole-body planning)

### 2. Bezpieczeństwo
Robot humanoidalny może łatwo uszkodzić sam siebie (self-collision) przy złej koordynacji.

**MoveIt 2 zapewnia:**
- Detekcję samo-kolizji w czasie rzeczywistym
- Sprawdzanie limitów stawów
- Bezpieczne planowanie trajektorii

### 3. Aplikacje badawcze
Typowe zadania dla G1 EDU współpracujące z MoveIt 2:
- Manipulacja obiektami obiema rękami (bimanual manipulation)
- Interakcja człowiek-robot (podawanie obiektów)
- Zadania manipulacji z mobilnością (mobile manipulation)
- Gestykulacja i komunikacja niewerbalna

---

## Architektura systemu

```
┌─────────────────────────────────────────────────────────┐
│                    Użytkownik/Aplikacja                  │
│                  (Twój kod Python/C++)                   │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│                   MoveIt 2 Framework                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Move Group   │  │ Planning     │  │ Collision    │  │
│  │ Interface    │  │ Algorithms   │  │ Detection    │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              ROS 2 Control (ros2_control)                │
│  ┌──────────────────────────────────────────────────┐  │
│  │     Controller Manager & Joint Controllers        │  │
│  └──────────────────────────────────────────────────┘  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│            Unitree G1 Hardware Interface                 │
│  (Komunikacja z firmware robota przez SDK/ROS drivers)  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│               Unitree G1 EDU - Sprzęt                    │
│  (Silniki, enkodery, czujniki, kamery)                  │
└─────────────────────────────────────────────────────────┘
```

**Przepływ informacji:**
1. **W górę (sensing):** Joint states → Hardware Interface → ROS 2 topics → MoveIt
2. **W dół (actuation):** MoveIt commands → Controller Manager → Hardware Interface → Robot

---

## Konfiguracja: Krok po kroku

### Krok 1: Przygotowanie modelu robota

**Co jest potrzebne:**
- **URDF (Unified Robot Description Format)** - plik XML opisujący kinematykę robota
- **Meshes** - modele 3D dla wizualizacji
- **SRDF (Semantic Robot Description Format)** - definicje grup planowania, self-collision matrix

**Przykład struktury URDF dla G1:**

```xml
<!-- unitree_g1_description/urdf/g1.urdf.xacro -->
<robot name="unitree_g1" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- 
  URDF definiuje strukturę robota:
  - Links (części sztywne): torso, upper_arm_left, forearm_left, etc.
  - Joints (połączenia): shoulder_pitch, elbow_roll, etc.
  -->
  
  <!-- Tułów (base) -->
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://unitree_g1_description/meshes/torso.stl"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://unitree_g1_description/meshes/torso_collision.stl"/>
      </geometry>
    </collision>
    <inertial>
      <!-- Parametry inercji: masa, środek masy, moment bezwładności -->
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3"/>
    </inertial>
  </link>
  
  <!-- Lewe ramię - 7 DOF -->
  <!-- Staw 1: Shoulder Pitch (góra-dół) -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.0 0.25 0.4" rpy="0 0 0"/>  <!-- Pozycja ramienia na tułowiu -->
    <axis xyz="0 1 0"/>  <!-- Oś rotacji: Y -->
    <limit lower="-2.8" upper="2.8" effort="50" velocity="2.0"/>
    <!-- Limity: zakres [-160°, 160°], max moment 50Nm, max prędkość 2 rad/s -->
  </joint>
  
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <mesh filename="package://unitree_g1_description/meshes/upper_arm.stl"/>
      </geometry>
    </visual>
    <!-- ... collision i inertial ... -->
  </link>
  
  <!-- Staw 2: Shoulder Roll (lewo-prawo) -->
  <joint name="left_shoulder_roll" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_shoulder"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.5" upper="1.5" effort="50" velocity="2.0"/>
  </joint>
  
  <!-- Powtórz dla pozostałych 5 stawów lewego ramienia -->
  <!-- Oraz dla prawego ramienia (symetrycznie) -->
  
  <!-- Efektor końcowy - chwytak -->
  <link name="left_hand_tcp">
    <!-- TCP = Tool Center Point - punkt referencyjny dla chwytaka -->
  </link>
  
  <joint name="left_hand_tcp_joint" type="fixed">
    <parent link="left_wrist"/>
    <child link="left_hand_tcp"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>  <!-- 15cm od nadgarstka -->
  </joint>
  
</robot>
```

**Dlaczego to ważne:**
- URDF zawiera **wszystkie informacje kinematyczne** potrzebne MoveIt do obliczeń
- Prawidłowe **limity stawów** zapobiegają uszkodzeniu robota
- **Collision meshes** używane są do detekcji kolizji

### Krok 2: Konfiguracja SRDF (Semantic Robot Description)

SRDF dodaje semantyczne informacje do URDF:

```xml
<!-- unitree_g1_moveit_config/config/g1.srdf -->
<robot name="unitree_g1">
  
  <!--
  Grupy planowania (Planning Groups):
  Definiują które stawy pracują razem jako jednostka
  -->
  
  <!-- Lewe ramię - 7 stawów -->
  <group name="left_arm">
    <chain base_link="base_link" tip_link="left_hand_tcp"/>
    <!-- 
    Chain: łańcuch kinematyczny od base do efektora
    MoveIt automatycznie znajdzie wszystkie stawy w tym łańcuchu
    -->
  </group>
  
  <!-- Prawe ramię - 7 stawów -->
  <group name="right_arm">
    <chain base_link="base_link" tip_link="right_hand_tcp"/>
  </group>
  
  <!-- Obie ramiona razem - dla bimanual manipulation -->
  <group name="both_arms">
    <group name="left_arm"/>
    <group name="right_arm"/>
  </group>
  
  <!-- Nogi - dla whole-body planning -->
  <group name="legs">
    <joint name="left_hip_yaw"/>
    <joint name="left_hip_roll"/>
    <joint name="left_hip_pitch"/>
    <joint name="left_knee"/>
    <joint name="left_ankle_pitch"/>
    <joint name="left_ankle_roll"/>
    <joint name="right_hip_yaw"/>
    <!-- ... pozostałe stawy prawej nogi ... -->
  </group>
  
  <!--
  Pozycje nazwane (Named Poses):
  Predefiniowane konfiguracje dla wygody
  -->
  <group_state name="home" group="left_arm">
    <!-- Pozycja domowa: ramię przy boku -->
    <joint name="left_shoulder_pitch" value="0.0"/>
    <joint name="left_shoulder_roll" value="0.0"/>
    <joint name="left_elbow" value="-1.57"/>  <!-- 90° zgięte -->
    <!-- ... pozostałe stawy ... -->
  </group_state>
  
  <group_state name="ready_to_grasp" group="left_arm">
    <!-- Pozycja gotowa do chwycenia: ramię wyciągnięte do przodu -->
    <joint name="left_shoulder_pitch" value="1.57"/>
    <joint name="left_elbow" value="-0.5"/>
    <!-- ... -->
  </group_state>
  
  <!--
  Wyłączenia kolizji (Disable Collisions):
  Pary linków, które NIGDY nie kolidują (są sąsiednie lub zawsze daleko)
  Optymalizacja: MoveIt nie musi sprawdzać tych par
  -->
  <disable_collisions link1="left_upper_arm" link2="base_link" reason="Adjacent"/>
  <disable_collisions link1="left_forearm" link2="left_upper_arm" reason="Adjacent"/>
  <!-- MoveIt Setup Assistant generuje automatycznie bazując na geometrii -->
  
  <!--
  Efektory końcowe (End Effectors):
  Definicja chwytaków/narzędzi
  -->
  <end_effector name="left_gripper" parent_link="left_hand_tcp" group="left_hand"/>
  <end_effector name="right_gripper" parent_link="right_hand_tcp" group="right_hand"/>
  
</robot>
```

**Kluczowe pojęcia:**

- **Planning Group:** Zestaw stawów planowanych razem
  - *Przykład:* "left_arm" = lewe ramię, "both_arms" = koordynacja obu ramion
  
- **Named Pose:** Zapisana konfiguracja stawów
  - *Korzyść:* Szybkie przejście do często używanych pozycji
  
- **Disable Collisions:** Lista par linków do ignorowania w detekcji kolizji
  - *Korzyść:* Przyspiesza obliczenia o ~50%

### Krok 3: Konfiguracja kontrolerów (ros2_control)

**Plik:** `unitree_g1_moveit_config/config/ros2_controllers.yaml`

```yaml
# ros2_controllers.yaml
#
# Konfiguracja kontrolerów dla ros2_control
# Kontrolery to "mosty" między MoveIt a sprzętem robota

controller_manager:
  ros__parameters:
    update_rate: 100  # Hz - częstotliwość aktualizacji kontrolerów
    
    # Lista kontrolerów do załadowania
    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

# Konfiguracja kontrolera lewego ramienia
left_arm_controller:
  ros__parameters:
    # Stawy kontrolowane przez ten kontroler
    joints:
      - left_shoulder_pitch
      - left_shoulder_roll
      - left_shoulder_yaw
      - left_elbow
      - left_wrist_yaw
      - left_wrist_pitch
      - left_wrist_roll
    
    # Interfejsy sprzętowe
    command_interfaces:
      - position  # Wysyłamy docelowe pozycje stawów
    
    state_interfaces:
      - position  # Odbieramy aktualne pozycje
      - velocity  # Odbieramy prędkości
    
    # Tolerancje śledzenia trajektorii
    constraints:
      stopped_velocity_tolerance: 0.01  # Prędkość < 0.01 rad/s = zatrzymany
      goal_time: 0.5  # Dopuszczalne opóźnienie celu [s]
      
      # Tolerancje dla każdego stawu
      left_shoulder_pitch: {trajectory: 0.05, goal: 0.01}
      left_shoulder_roll: {trajectory: 0.05, goal: 0.01}
      # ... pozostałe stawy ...
    
    # Akcja ROS - interfejs dla MoveIt
    action_monitor_rate: 20.0  # Hz - publikacja statusu akcji

# Analogicznie dla prawego ramienia
right_arm_controller:
  ros__parameters:
    joints:
      - right_shoulder_pitch
      # ...
```

**Wyjaśnienie:**

1. **Controller Manager:** Zarządza wszystkimi kontrolerami
   - Ładuje/wyładowuje kontrolery
   - Aktualizuje je z określoną częstotliwością

2. **JointTrajectoryController:** Wykonuje trajektorie planowane przez MoveIt
   - Otrzymuje: trajectory (sekwencja punktów: pozycja, prędkość, czas)
   - Wysyła: docelowe pozycje do hardware interface
   - Monitoruje: rzeczywiste pozycje, sprawdza błędy śledzenia

3. **JointStateBroadcaster:** Publikuje stan stawów na topic `/joint_states`
   - MoveIt odczytuje aktualne pozycje robota z tego topicu

### Krok 4: Konfiguracja MoveIt

**Plik:** `unitree_g1_moveit_config/config/moveit.yaml`

```yaml
# moveit.yaml - główna konfiguracja MoveIt dla G1

# ========================================
# Planning Pipeline Configuration
# ========================================
planning_pipelines:
  # Lista dostępnych pipeline'ów planowania
  pipeline_names: ["ompl", "pilz_industrial_motion_planner", "stomp"]
  
  # Domyślny pipeline
  default_planning_pipeline: "ompl"

# ========================================
# OMPL Planning Pipeline
# ========================================
ompl:
  planning_plugin: ompl_interface/OMPLPlanner
  
  # Parametry globalnego plannera
  request_adapters: >-
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/FixWorkspaceBounds
    default_planner_request_adapters/FixStartStateBounds
    default_planner_request_adapters/FixStartStateCollision
    default_planner_request_adapters/FixStartStatePathConstraints
  
  start_state_max_bounds_error: 0.1
  
  # Planery dla lewego ramienia
  left_arm:
    # Planer domyślny
    default_planner_config: RRTConnectkConfigDefault
    
    # Dostępne plannery (algorytmy)
    planner_configs:
      - RRTConnectkConfigDefault  # Szybki, dla prostych scenariuszy
      - RRTstarkConfigDefault      # Optymalizuje koszt trajektorii
      - PRMkConfigDefault          # Dobry dla powtarzalnych zadań
      - BKPIECEkConfigDefault      # Dla wąskich przejść
    
    # Rzutowanie: wymuszenie planowania w określonej podprzestrzeni
    projection_evaluator: joints(left_shoulder_pitch, left_shoulder_roll, left_elbow)
    
    # Parametry szczegółowe
    longest_valid_segment_fraction: 0.005  # Rozdzielczość sprawdzania kolizji
  
  # Analogicznie dla prawego ramienia i both_arms
  right_arm:
    # ...
  
  both_arms:
    default_planner_config: RRTConnectkConfigDefault
    projection_evaluator: joints(
      left_shoulder_pitch, left_shoulder_roll,
      right_shoulder_pitch, right_shoulder_roll
    )

# ========================================
# Pilz Industrial Planner
# ========================================
# Planowanie przemysłowe: proste, przewidywalne ruchy
pilz_industrial_motion_planner:
  planning_plugin: pilz_industrial_motion_planner/CommandPlanner
  
  capabilities: >
    pilz_industrial_motion_planner/MoveGroupSequenceAction
    pilz_industrial_motion_planner/MoveGroupSequenceService
  
  # Dostępne typy ruchów
  default_planner_config: PTP  # Point-to-Point
  
  # PTP: Ruch w joint space (wszystkie stawy jednocześnie)
  # LIN: Ruch liniowy w Cartesian space
  # CIRC: Ruch kołowy w Cartesian space

# ========================================
# Collision Detection
# ========================================
collision_detection:
  plugin: collision_detection/CollisionPlugin
  
  # Parametry detekcji kolizji
  collision_detector: "bullet"  # Opcje: fcl, bullet
  
  # Self-collision checking
  check_for_self_collision: true
  
  # Margines bezpieczeństwa (padding)
  default_robot_padding: 0.01  # 1cm bufora wokół robota
  default_robot_scale: 1.0
  
  # Specyficzne marginesy dla wrażliwych części
  robot_link_padding:
    left_hand_tcp: 0.02  # 2cm bufora dla chwytaka (wrażliwa część)
    right_hand_tcp: 0.02

# ========================================
# Kinematics Solvers
# ========================================
# Rozwiązywanie kinematyki odwrotnej (IK)
kinematics:
  left_arm:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    # KDL (Kinematics and Dynamics Library) - niezawodny solver
    
    kinematics_solver_search_resolution: 0.005  # Rozdzielczość próbkowania
    kinematics_solver_timeout: 0.05  # 50ms na rozwiązanie IK
    kinematics_solver_attempts: 3  # Liczba prób z różnymi seed states
    
  right_arm:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.05
    kinematics_solver_attempts: 3
  
  both_arms:
    # Dla coordinated bimanual manipulation
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_timeout: 0.1  # Dłuższy timeout dla więcej DOF

# ========================================
# Trajectory Execution
# ========================================
trajectory_execution:
  # Interfejs do wykonywania trajektorii
  allowed_execution_duration_scaling: 1.2  # Dopuszcz 20% przekroczenie czasu
  allowed_goal_duration_margin: 0.5  # 500ms marginesu na cel
  allowed_start_tolerance: 0.01  # Tolerancja różnicy stan początkowy
  
  # Controllery
  execution_duration_monitoring: true  # Monitoruj czy wykonanie trwa zbyt długo
  
  # Mapowanie grup planowania na kontrolery
  moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

# ========================================
# Sensors (3D Perception)
# ========================================
sensors:
  # Konfiguracja czujników 3D dla percepcji środowiska
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth/points  # Topic z chmurą punktów
    max_range: 5.0  # Maksymalny zasięg czujnika [m]
    frame_subsample: 1  # Próbkowanie co N-tą ramkę (1 = każda)
    point_subsample: 1  # Próbkowanie punktów w chmurze
    
    # Parametry occupancy map (mapa zajętości przestrzeni)
    padding_offset: 0.02  # Rozszerzenie przeszkód o 2cm
    padding_scale: 1.0
    
    # Filtrowanie szumu
    filtered_cloud_topic: /filtered_cloud
```

**Kluczowe decyzje konfiguracyjne:**

1. **Wybór plannera:**
   - **RRTConnect:** Szybki, dla większości zadań
   - **RRT\*:** Optymalizuje jakość trajektorii
   - **Pilz:** Dla prostych ruchów przemysłowych

2. **Kinematics solver:**
   - **KDL:** Stabilny, średnio szybki
   - **TRAC-IK:** Szybszy, lepsze coverage
   - **Wybór:** Dla G1 zalecamy KDL (sprawdzony)

3. **Collision detection:**
   - **FCL:** Precyzyjny, wolniejszy
   - **Bullet:** Szybszy, używany w grach/symulacjach
   - **Wybór:** Bullet dla real-time performance

---

## Uruchomienie systemu

### Launch file dla symulacji Gazebo

**Plik:** `unitree_g1_moveit_config/launch/demo_gazebo.launch.py`

```python
#!/usr/bin/env python3
"""
Launch file dla demonstracji Unitree G1 z MoveIt 2 w symulacji Gazebo

Komponenty uruchamiane:
1. Gazebo simulator - symulacja fizyki robota
2. Robot State Publisher - publikacja TF (transformacje między linkami)
3. Joint State Publisher - publikacja stanów stawów
4. MoveIt Move Group - główny węzeł planowania
5. RViz - wizualizacja i interfejs użytkownika

Użycie:
  ros2 launch unitree_g1_moveit_config demo_gazebo.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Generuje deskrypcję launch - lista węzłów i parametrów do uruchomienia
    """
    
    # ============================================
    # KROK 1: Deklaracja argumentów launch
    # ============================================
    # Argumenty pozwalają użytkownikowi dostosować launch bez edycji pliku
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Użyj czasu symulacji Gazebo (true) lub czasu systemowego (false)'
    )
    
    # ============================================
    # KROK 2: Wyszukiwanie pakietów i plików
    # ============================================
    # FindPackageShare znajduje ścieżkę do zainstalowanych pakietów ROS
    
    moveit_config_pkg = FindPackageShare('unitree_g1_moveit_config')
    description_pkg = FindPackageShare('unitree_g1_description')
    
    # Ścieżki do plików konfiguracyjnych
    urdf_file = PathJoinSubstitution([description_pkg, 'urdf', 'g1.urdf.xacro'])
    srdf_file = PathJoinSubstitution([moveit_config_pkg, 'config', 'g1.srdf'])
    moveit_config_file = PathJoinSubstitution([moveit_config_pkg, 'config', 'moveit.yaml'])
    
    # ============================================
    # KROK 3: Robot State Publisher
    # ============================================
    # Publikuje transformacje (TF) między wszystkimi linkami robota
    # TF tree: base_link -> shoulder -> upper_arm -> forearm -> hand -> tcp
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_file,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # ============================================
    # KROK 4: Gazebo Simulator
    # ============================================
    # Symulator fizyki - emuluje dynamikę robota
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'false',
            'pause': 'false'  # Rozpocznij symulację od razu
        }.items()
    )
    
    # Spawn robota w Gazebo (umieszczenie modelu w świecie symulacji)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'unitree_g1',  # Nazwa encji w Gazebo
            '-file', urdf_file,
            '-x', '0.0',  # Pozycja startowa X
            '-y', '0.0',  # Pozycja startowa Y
            '-z', '1.0'   # Pozycja startowa Z (1m nad podłożem)
        ],
        output='screen'
    )
    
    # ============================================
    # KROK 5: MoveIt Move Group Node
    # ============================================
    # Główny węzeł MoveIt - planowanie, IK, collision checking
    
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            # Załaduj wszystkie pliki konfiguracyjne
            {'robot_description': urdf_file},
            {'robot_description_semantic': srdf_file},
            moveit_config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_robot_description': True,
                'publish_robot_description_semantic': True,
                
                # Planning
                'planning_pipelines': ['ompl'],
                'default_planning_pipeline': 'ompl',
                
                # Trajectory execution
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                
                # Sensor-based planning (opcjonalne - jeśli mamy kamery)
                'sensors_3d.point_cloud_topic': '/camera/depth/points'
            }
        ]
    )
    
    # ============================================
    # KROK 6: RViz Visualization
    # ============================================
    # Graficzny interfejs użytkownika MoveIt
    
    rviz_config = PathJoinSubstitution([moveit_config_pkg, 'config', 'moveit.rviz'])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # ============================================
    # KROK 7: Złożenie LaunchDescription
    # ============================================
    # Określa kolejność uruchamiania węzłów
    
    return LaunchDescription([
        # Argumenty
        use_sim_time,
        
        # Węzły - uruchamiane sekwencyjnie
        robot_state_publisher,
        gazebo,
        spawn_entity,
        move_group,
        rviz
    ])
```

**Uruchomienie:**

```bash
# Terminal 1: Uruchom symulację
ros2 launch unitree_g1_moveit_config demo_gazebo.launch.py

# Terminal 2: Sprawdź czy wszystko działa
ros2 topic list  # Powinieneś zobaczyć topiki /joint_states, /tf, itp.
ros2 topic echo /joint_states  # Podgląd bieżących pozycji stawów
```

---

## Przykład: Bimanual Manipulation

**Zadanie:** Chwyć obiekt obiema rękami i przenieś go.

```python
#!/usr/bin/env python3
"""
Demonstracja manipulacji obiema rękami (bimanual manipulation)

Scenariusz:
Robot G1 chwyta długi obiekt (np. patyk, deska) obiema rękami,
podnosi go i przenosi w nowe miejsce.

Wymagania:
- Synchronizacja obu ramion
- Utrzymanie względnej pozycji chwytaków
- Unikanie kolizji między ramionami
"""

import rclpy
from rclpy.node import Node
from moveit_py import MoveItPy
from moveit_py.planning import PlanRequestParameters
from geometry_msgs.msg import PoseStamped
import numpy as np

class BimanualManipulation(Node):
    """Klasa demonstracyjna manipulacji dwuręcznej"""
    
    def __init__(self):
        super().__init__('bimanual_manipulation_demo')
        self.get_logger().info("Inicjalizacja MoveIt dla G1...")
        
        # Inicjalizacja MoveIt
        self.moveit = MoveItPy(node_name='moveit_py_bimanual')
        
        # Interfejsy planowania dla obu ramion
        self.left_arm = self.moveit.get_planning_component('left_arm')
        self.right_arm = self.moveit.get_planning_component('right_arm')
        self.both_arms = self.moveit.get_planning_component('both_arms')
        
        self.get_logger().info("MoveIt gotowy!")
    
    def demo_synchronized_movement(self):
        """
        Demo 1: Zsynchronizowany ruch obu ramion
        
        Strategia:
        1. Planuj ruch dla obu ramion jednocześnie (grupa "both_arms")
        2. Wykonaj trajektorię synchronicznie
        
        Zastosowanie:
        - Noszenie dużych/ciężkich obiektów
        - Zadania wymagające precyzyjnej koordynacji
        """
        
        self.get_logger().info("=== Demo 1: Zsynchronizowany ruch ===")
        
        # Definicja celów dla obu chwytaków
        # Oba chwytaki mają być w tej samej wysokości, rozstawione na 40cm
        
        left_goal = PoseStamped()
        left_goal.header.frame_id = 'base_link'
        left_goal.pose.position.x = 0.5  # 50cm do przodu
        left_goal.pose.position.y = 0.2  # 20cm w lewo
        left_goal.pose.position.z = 0.8  # 80cm w górę
        left_goal.pose.orientation.w = 1.0
        
        right_goal = PoseStamped()
        right_goal.header.frame_id = 'base_link'
        right_goal.pose.position.x = 0.5   # 50cm do przodu (jak lewy)
        right_goal.pose.position.y = -0.2  # 20cm w prawo
        right_goal.pose.position.z = 0.8   # 80cm w górę (jak lewy)
        right_goal.pose.orientation.w = 1.0
        
        # Ustaw cele dla grupy "both_arms"
        # MoveIt zaplanuje trajektorię dla obu ramion jednocześnie
        self.both_arms.set_start_state_to_current_state()
        
        # Dla both_arms musimy ustawić cele dla obu efektorów
        self.both_arms.set_goal_state(
            pose_stamped_msg=left_goal,
            pose_link='left_hand_tcp'
        )
        self.both_arms.set_goal_state(
            pose_stamped_msg=right_goal,
            pose_link='right_hand_tcp',
            append=True  # APPEND=True: dodaj drugi cel bez usuwania pierwszego
        )
        
        # Planowanie
        self.get_logger().info("Planowanie zsynchronizowanego ruchu...")
        plan = self.both_arms.plan(PlanRequestParameters(
            planning_time=15.0,  # Dłuższy czas - trudniejsze zadanie
            planner_id='RRTConnectkConfigDefault'
        ))
        
        if plan.error_code.val == 1:
            self.get_logger().info("Plan gotowy - wykonuję...")
            self.both_arms.execute()
            self.get_logger().info("✓ Zsynchronizowany ruch zakończony")
        else:
            self.get_logger().error("✗ Planowanie nie powiodło się")
    
    def demo_sequential_movement(self):
        """
        Demo 2: Sekwencyjny ruch - najpierw lewe, potem prawe
        
        Strategia:
        1. Planuj i wykonaj ruch lewego ramienia
        2. Następnie planuj i wykonaj ruch prawego ramienia
        
        Zastosowanie:
        - Prostsze zadania
        - Gdy synchronizacja nie jest krytyczna
        - Debugging (łatwiej zidentyfikować problemy)
        """
        
        self.get_logger().info("=== Demo 2: Sekwencyjny ruch ===")
        
        # Ruch 1: Lewe ramię
        self.get_logger().info("Krok 1/2: Ruch lewego ramienia...")
        self.left_arm.set_start_state_to_current_state()
        
        left_pose = PoseStamped()
        left_pose.header.frame_id = 'base_link'
        left_pose.pose.position.x = 0.6
        left_pose.pose.position.y = 0.3
        left_pose.pose.position.z = 0.7
        left_pose.pose.orientation.w = 1.0
        
        self.left_arm.set_goal_state(
            pose_stamped_msg=left_pose,
            pose_link='left_hand_tcp'
        )
        
        plan_left = self.left_arm.plan()
        if plan_left.error_code.val == 1:
            self.left_arm.execute()
            self.get_logger().info("✓ Lewe ramię w pozycji")
        
        # Ruch 2: Prawe ramię
        self.get_logger().info("Krok 2/2: Ruch prawego ramienia...")
        self.right_arm.set_start_state_to_current_state()
        
        right_pose = PoseStamped()
        right_pose.header.frame_id = 'base_link'
        right_pose.pose.position.x = 0.6
        right_pose.pose.position.y = -0.3
        right_pose.pose.position.z = 0.7
        right_pose.pose.orientation.w = 1.0
        
        self.right_arm.set_goal_state(
            pose_stamped_msg=right_pose,
            pose_link='right_hand_tcp'
        )
        
        plan_right = self.right_arm.plan()
        if plan_right.error_code.val == 1:
            self.right_arm.execute()
            self.get_logger().info("✓ Prawe ramię w pozycji")
    
    def demo_constrained_bimanual(self):
        """
        Demo 3: Ruch z ograniczeniami - utrzymywanie stałej odległości
        
        Strategia:
        1. Dodaj constraint: odległość między chwytakami = const
        2. Planuj ruch respektujący to ograniczenie
        
        Zastosowanie:
        - Trzymanie sztywnego obiektu (patyk, deska)
        - Zapobieganie rozciąganiu elastycznego obiektu
        """
        
        self.get_logger().info("=== Demo 3: Ruch z ograniczeniem odległości ===")
        
        # TODO: Implementacja constraint planning
        # To zaawansowana funkcjonalność - wymaga custom constraint sampler
        
        self.get_logger().info("Funkcjonalność w przygotowaniu - wymaga custom plannera")
        
        # Ogólna idea:
        # 1. Zdefiniuj constraint: distance(left_tcp, right_tcp) = 0.4m ± 2cm
        # 2. Użyj constraint-aware plannera (np. OMPL CBiRRT)
        # 3. Planer znajdzie trajektorię spełniającą constraint

def main():
    """Główna funkcja demonstracyjna"""
    
    rclpy.init()
    demo = BimanualManipulation()
    
    try:
        # Uruchom kolejne demonstracje
        demo.get_logger().info("\n" + "="*60)
        demo.get_logger().info("DEMONSTRACJA MANIPULACJI DWURĘCZNEJ - UNITREE G1")
        demo.get_logger().info("="*60 + "\n")
        
        input("Naciśnij Enter aby rozpocząć Demo 1 (synchronizacja)...")
        demo.demo_synchronized_movement()
        
        input("\nNaciśnij Enter aby rozpocząć Demo 2 (sekwencja)...")
        demo.demo_sequential_movement()
        
        input("\nNaciśnij Enter aby rozpocząć Demo 3 (constraint)...")
        demo.demo_constrained_bimanual()
        
        demo.get_logger().info("\n✓ Wszystkie demonstracje zakończone!")
        
    except KeyboardInterrupt:
        demo.get_logger().info("Przerwano przez użytkownika")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Zaawansowane tematy

### 1. Whole-Body Planning

**Czym jest?**
Planowanie uwzględniające wszystkie stopnie swobody robota jednocześnie (ramiona + nogi + tułów).

**Po co?**
- Osiąganie celów poza podstawowym zasięgiem (wykorzystanie pochylenia)
- Utrzymywanie balansu podczas manipulacji
- Optymalizacja manipulability

**Przykład zadania:**
Robot musi dosięgnąć obiektu na podłodze - musi się schylić używając nóg i tułowia.

### 2. Visual Servoing

**Czym jest?**
Sterowanie oparte na informacji z kamery - robot reaguje na obraz w czasie rzeczywistym.

**Integracja z MoveIt:**
1. Kamera detektuje obiekt → pozycja 3D
2. MoveIt planuje ruch do obiektu
3. W trakcie ruchu: kamera aktualizuje pozycję → MoveIt replannuje
4. Finalnie: precyzyjne chwyty nie wymagające doskonałej kalibracji

### 3. Mobile Manipulation

**Czym jest?**
Robot G1 porusza się (chodzi) i manipuluje jednocześnie.

**Wyzwania:**
- Koordynacja lokomocji i manipulacji
- Dynamiczny balans
- Planning w wysokowymiarowej przestrzeni

**Podejście MoveIt:**
- Grupa planowania: `whole_body` (nogi + ramiona + tułów)
- Custom plannery uwzględniające dynamikę chodzenia
- Replanowanie w czasie rzeczywistym

---

## Podsumowanie

### Czego się nauczyłeś:

✓ Jak skonfigurować MoveIt 2 dla robota humanoidalnego
✓ Struktura pakietu moveit_config (URDF, SRDF, controllers)
✓ Bimanual manipulation - koordynacja obu ramion
✓ Launch files i konfiguracja systemu
✓ Zaawansowane tematy: whole-body, visual servoing

### Następne kroki dla projektów z G1:

1. **Podstawowe:** Zaimplementuj pick-and-place jedną ręką
2. **Średnie:** Koordynacja obu ramion do przenoszenia obiektów
3. **Zaawansowane:** Manipulation podczas chodzenia (mobile manipulation)
4. **Expert:** Whole-body optimization z balansowaniem

### Zasoby dodatkowe:

- **Dokumentacja Unitree SDK:** https://www.unitree.com/
- **MoveIt 2 Tutorials:** https://moveit.picknik.ai/
- **Humanoid Robotics Research:** https://humanoidroboticsproject.com/

---

**Powodzenia w projektach z Unitree G1 EDU! 🤖🦾**
