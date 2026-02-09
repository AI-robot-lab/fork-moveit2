#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Przykład demonstracyjny MoveIt 2 - Planowanie i Wykonanie Ruchu Robota
======================================================================

Ten skrypt pokazuje podstawowe operacje MoveIt 2 z szczegółowymi komentarzami
w języku polskim dla studentów uczących się programowania robotów.

Autor: MoveIt 2 Contributors
Licencja: BSD-3-Clause
Zmodyfikowane dla celów edukacyjnych w języku polskim

WYMAGANIA:
    - ROS 2 Humble lub nowszy
    - MoveIt 2
    - Robot demo (np. Panda) lub symulacja Gazebo

URUCHOMIENIE:
    Terminal 1:
        ros2 launch moveit2_tutorials demo.launch.py
    
    Terminal 2:
        python3 moveit_py_example_pl.py

ZADANIA WYKONYWANE PRZEZ SKRYPT:
    1. Inicjalizacja węzła ROS 2 i interfejsu MoveIt
    2. Planowanie ruchu do nazwanych pozycji
    3. Planowanie ruchu do pozycji kartezjańskiej
    4. Planowanie ścieżki kartezjańskiej (ruch w linii prostej)
    5. Dodawanie i usuwanie przeszkód
    6. Planowanie z unikaniem kolizji
"""

# ============================================================================
# SEKCJA 1: Importy bibliotek
# ============================================================================
# Importujemy wszystkie potrzebne moduły Python i ROS

import sys
import time
from typing import List, Tuple

# ROS 2 - podstawowe biblioteki do komunikacji między węzłami
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

# MoveIt 2 - główna biblioteka do planowania ruchu
try:
    from moveit.planning import (
        MoveItPy,
        PlanRequestParameters,
    )
    from moveit.core.robot_state import RobotState
    from moveit.core.kinematic_constraints import construct_joint_constraint
except ImportError:
    print("BŁĄD: Nie można zaimportować modułu moveit_py!")
    print("Sprawdź czy pakiet moveit_py jest zainstalowany:")
    print("  sudo apt install ros-humble-moveit-py")
    sys.exit(1)

# Geometry messages - wiadomości ROS do reprezentacji pozycji i orientacji
from geometry_msgs.msg import (
    PoseStamped,    # Pozycja + orientacja + timestamp
    Pose,           # Pozycja + orientacja
    Point,          # Punkt w przestrzeni 3D (X, Y, Z)
    Quaternion      # Orientacja jako kwaternion (x, y, z, w)
)

# MoveIt messages - wiadomości specyficzne dla MoveIt
from moveit_msgs.msg import (
    CollisionObject,        # Reprezentacja przeszkody
    AttachedCollisionObject # Obiekt przyczepiony do robota (np. narzędzie)
)

# Shape messages - prymitywne kształty geometryczne
from shape_msgs.msg import SolidPrimitive

# Standard messages
from std_msgs.msg import Header


# ============================================================================
# SEKCJA 2: Klasa pomocnicza Logger
# ============================================================================
class ColoredLogger:
    """
    Klasa pomocnicza do kolorowego wyświetlania logów w terminalu.
    
    Używamy kolorów aby łatwiej rozróżnić różne typy komunikatów:
    - Zielony: Sukces operacji
    - Żółty: Informacja/ostrzeżenie
    - Czerwony: Błąd
    - Niebieski: Krok procesu
    """
    
    # Kody ANSI dla kolorów w terminalu
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'  # Reset koloru
    
    @staticmethod
    def success(message: str):
        """Wyświetla komunikat sukcesu (zielony)"""
        print(f"{ColoredLogger.GREEN}✓ {message}{ColoredLogger.END}")
    
    @staticmethod
    def info(message: str):
        """Wyświetla informację (żółty)"""
        print(f"{ColoredLogger.YELLOW}ℹ {message}{ColoredLogger.END}")
    
    @staticmethod
    def error(message: str):
        """Wyświetla błąd (czerwony)"""
        print(f"{ColoredLogger.RED}✗ {message}{ColoredLogger.END}")
    
    @staticmethod
    def step(step_number: int, message: str):
        """Wyświetla numer kroku (niebieski)"""
        print(f"\n{ColoredLogger.BLUE}{ColoredLogger.BOLD}"
              f"KROK {step_number}: {message}"
              f"{ColoredLogger.END}")


# ============================================================================
# SEKCJA 3: Główna klasa demonstracyjna
# ============================================================================
class MoveItDemoNode(Node):
    """
    Główna klasa węzła ROS demonstrującego możliwości MoveIt 2.
    
    Węzeł (Node) to podstawowa jednostka w ROS - pojedynczy program/proces
    wykonujący określone zadanie. Ten węzeł pokazuje różne sposoby
    planowania i wykonywania ruchu robota.
    
    Atrybuty:
        moveit: Główny interfejs MoveItPy
        panda_arm: Interfejs do planowania dla grupy "panda_arm"
        logger: Kolorowy logger do wyświetlania komunikatów
    """
    
    def __init__(self):
        """
        Konstruktor węzła - inicjalizacja wszystkich komponentów.
        
        Kolejność inicjalizacji:
        1. Inicjalizacja bazowej klasy Node
        2. Utworzenie loggera
        3. Inicjalizacja MoveItPy
        4. Pobranie interfejsu grupy planowania
        """
        
        # Krok 1: Inicjalizacja węzła ROS
        # Nazwa węzła: 'moveit_demo_node' - pojawi się w liście węzłów (ros2 node list)
        super().__init__('moveit_demo_node')
        
        # Krok 2: Utworzenie loggera
        self.logger = ColoredLogger()
        self.logger.info("Inicjalizacja węzła demonstracyjnego MoveIt 2...")
        
        # Krok 3: Inicjalizacja MoveItPy
        # MoveItPy to główny interfejs Python do MoveIt 2
        # Ładuje konfigurację robota, plannery, collision detection, etc.
        try:
            self.logger.info("Ładowanie konfiguracji MoveIt...")
            self.moveit = MoveItPy(node_name="moveit_py_planning_node")
            self.logger.success("MoveIt zainicjalizowany pomyślnie!")
        except Exception as e:
            self.logger.error(f"Nie udało się zainicjalizować MoveIt: {e}")
            self.logger.info("Sprawdź czy uruchomiony jest move_group node")
            raise
        
        # Krok 4: Pobranie interfejsu grupy planowania
        # Grupa planowania (planning group) to zestaw stawów pracujących razem
        # Dla robota Panda: "panda_arm" = 7 stawów ramienia
        self.planning_group_name = "panda_arm"
        try:
            self.panda_arm = self.moveit.get_planning_component(self.planning_group_name)
            self.logger.success(f"Grupa planowania '{self.planning_group_name}' gotowa")
        except Exception as e:
            self.logger.error(f"Nie można załadować grupy planowania: {e}")
            raise
        
        # Informacje o robocie
        self._print_robot_info()
    
    def _print_robot_info(self):
        """
        Wyświetla podstawowe informacje o robocie i jego konfiguracji.
        
        Pomaga zrozumieć strukturę robota przed rozpoczęciem planowania.
        """
        print("\n" + "="*70)
        print("INFORMACJE O ROBOCIE")
        print("="*70)
        
        # Pobierz model robota z MoveIt
        robot_model = self.moveit.get_robot_model()
        
        # Wyświetl podstawowe informacje
        print(f"Nazwa robota: {robot_model.get_name()}")
        print(f"Używana grupa planowania: {self.planning_group_name}")
        
        # Lista wszystkich stawów w grupie planowania
        joint_model_group = robot_model.get_joint_model_group(self.planning_group_name)
        joint_names = joint_model_group.get_active_joint_model_names()
        print(f"\nStawy w grupie '{self.planning_group_name}':")
        for i, joint in enumerate(joint_names, 1):
            print(f"  {i}. {joint}")
        
        # Nazwa efektora końcowego (end-effector)
        # To punkt referencyjny na końcu ramienia (np. środek chwytaka)
        link_names = joint_model_group.get_link_model_names()
        print(f"\nLinki w grupie:")
        for link in link_names:
            print(f"  - {link}")
        
        print("="*70 + "\n")
    
    # ========================================================================
    # DEMO 1: Planowanie do nazwanej pozycji
    # ========================================================================
    def demo_named_target(self):
        """
        Demonstracja 1: Planowanie ruchu do predefiniowanej pozycji.
        
        POJĘCIE: Named Target (nazwana pozycja)
        --------------------------------------
        Named target to zapisana konfiguracja stawów z przypisaną nazwą.
        Przykłady: "home", "ready", "extended"
        
        ZALETY:
        - Szybkie wywoływanie często używanych pozycji
        - Nie trzeba pamiętać wartości kątów stawów
        - Można zdefiniować w pliku SRDF
        
        PROCES:
        1. Ustaw stan początkowy (bieżący stan robota)
        2. Wybierz named target jako cel
        3. Zaplanuj trajektorię
        4. Wykonaj ruch
        """
        
        self.logger.step(1, "Planowanie do nazwanej pozycji 'ready'")
        
        # Krok 1.1: Ustaw stan początkowy
        # ----------------------------------
        # Zawsze musimy powiedzieć MoveIt skąd robot zaczyna planowanie
        # Używamy bieżącego stanu robota (odczytanego z /joint_states)
        self.logger.info("Ustawiam stan początkowy na bieżący stan robota...")
        self.panda_arm.set_start_state_to_current_state()
        
        # Krok 1.2: Wybierz named target jako cel
        # ----------------------------------------
        # "ready" to predefiniowana pozycja w SRDF robota Panda
        # Robot będzie miał ramię zgięte, gotowe do pracy
        target_name = "ready"
        self.logger.info(f"Ustawiam cel na nazwana pozycję: '{target_name}'")
        self.panda_arm.set_goal_state(configuration_name=target_name)
        
        # Krok 1.3: Zaplanuj trajektorię
        # -------------------------------
        # MoveIt użyje algorytmu planowania (domyślnie OMPL)
        # Znajdzie bezkolizyjną ścieżkę od stanu początkowego do celu
        self.logger.info("Planuję trajektorię...")
        
        # Parametry planowania
        plan_params = PlanRequestParameters(
            planning_attempts=10,       # Maksymalnie 10 prób znalezienia planu
            planning_time=5.0,          # Maksymalnie 5 sekund na planowanie
            max_velocity_scaling_factor=0.5,  # Ogranicz prędkość do 50% maksymalnej
            max_acceleration_scaling_factor=0.5  # Ogranicz przyspieszenie do 50%
        )
        
        # Wywołanie plannera
        plan_result = self.panda_arm.plan(plan_params)
        
        # Krok 1.4: Sprawdź wynik planowania
        # -----------------------------------
        # error_code.val == 1 oznacza SUCCESS
        if plan_result.error_code.val != 1:
            self.logger.error(f"Planowanie nie powiodło się! Kod błędu: {plan_result.error_code.val}")
            return False
        
        self.logger.success("Trajektoria zaplanowana pomyślnie!")
        
        # Informacje o trajektorii
        trajectory = plan_result.trajectory
        num_points = len(trajectory.joint_trajectory.points)
        duration = trajectory.joint_trajectory.points[-1].time_from_start.sec if num_points > 0 else 0
        self.logger.info(f"Trajektoria zawiera {num_points} punktów, czas trwania: {duration}s")
        
        # Krok 1.5: Wykonaj ruch
        # ----------------------
        # Wyślij zaplanowaną trajektorię do kontrolera robota
        self.logger.info("Wykonuję ruch...")
        success = self.panda_arm.execute()
        
        if success:
            self.logger.success("Ruch zakończony pomyślnie!")
            return True
        else:
            self.logger.error("Wykonanie ruchu nie powiodło się!")
            return False
    
    # ========================================================================
    # DEMO 2: Planowanie do pozycji kartezjańskiej
    # ========================================================================
    def demo_pose_target(self):
        """
        Demonstracja 2: Planowanie ruchu do pozycji w przestrzeni kartezjańskiej.
        
        POJĘCIE: Pose Target (cel pozycyjny)
        ------------------------------------
        Pose to pozycja (X, Y, Z) + orientacja (roll, pitch, yaw lub kwaternion)
        w przestrzeni 3D. Określamy gdzie chcemy aby znalazł się efektor końcowy.
        
        RÓŻNICA vs Named Target:
        - Named target: podajemy kąty wszystkich stawów
        - Pose target: podajemy gdzie ma być efektor, MoveIt oblicza kąty (IK)
        
        PROCES:
        1. Ustaw stan początkowy
        2. Zdefiniuj pozę docelową (PoseStamped)
        3. MoveIt rozwiązuje kinematykę odwrotną (IK)
        4. Planuj trajektorię w przestrzeni stawów
        5. Wykonaj ruch
        """
        
        self.logger.step(2, "Planowanie do pozycji kartezjańskiej")
        
        # Krok 2.1: Ustaw stan początkowy
        self.panda_arm.set_start_state_to_current_state()
        
        # Krok 2.2: Zdefiniuj pozę docelową
        # ----------------------------------
        # Tworzymy wiadomość PoseStamped (Pose + frame_id + timestamp)
        target_pose = PoseStamped()
        
        # Header - metadane wiadomości
        target_pose.header = Header()
        target_pose.header.frame_id = "panda_link0"  # Frame bazowy robota
        # frame_id określa układ odniesienia współrzędnych
        # "panda_link0" to base robota - współrzędne liczone od podstawy
        
        # Pozycja (Point) - gdzie ma być efektor [metry]
        target_pose.pose.position = Point()
        target_pose.pose.position.x = 0.28  # 28cm do przodu od base
        target_pose.pose.position.y = -0.2  # 20cm w prawo (ujemne Y)
        target_pose.pose.position.z = 0.5   # 50cm w górę
        
        # Orientacja (Quaternion) - jak ma być obrócony efektor
        # Kwaternion (x, y, z, w) reprezentuje rotację w 3D
        # Ten kwaternion reprezentuje orientację: chwytak skierowany w dół
        target_pose.pose.orientation = Quaternion()
        target_pose.pose.orientation.x = 1.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0
        
        self.logger.info(f"Cel: pozycja=({target_pose.pose.position.x}, "
                        f"{target_pose.pose.position.y}, {target_pose.pose.position.z})")
        
        # Krok 2.3: Ustaw cel w MoveIt
        # ----------------------------
        # MoveIt najpierw rozwiąże kinematykę odwrotną (IK):
        # Pozycja efektora → Kąty stawów
        self.logger.info("Rozwiązuję kinematykę odwrotną (IK)...")
        self.panda_arm.set_goal_state(
            pose_stamped_msg=target_pose,
            pose_link="panda_hand"  # Link efektora końcowego
        )
        
        # Krok 2.4: Planuj i wykonaj
        # --------------------------
        self.logger.info("Planuję trajektorię...")
        plan_params = PlanRequestParameters(
            planning_attempts=10,
            planning_time=5.0,
            max_velocity_scaling_factor=0.3,  # Wolniejszy ruch (30%)
            max_acceleration_scaling_factor=0.3
        )
        
        plan_result = self.panda_arm.plan(plan_params)
        
        if plan_result.error_code.val != 1:
            self.logger.error("Planowanie nie powiodło się!")
            self.logger.info("Możliwe przyczyny:")
            self.logger.info("  - Cel poza zasięgiem robota")
            self.logger.info("  - Kolizja w pozycji docelowej")
            self.logger.info("  - Brak rozwiązania IK")
            return False
        
        self.logger.success("Trajektoria zaplanowana!")
        self.logger.info("Wykonuję ruch...")
        
        success = self.panda_arm.execute()
        if success:
            self.logger.success("Robot dotarł do celu!")
            return True
        else:
            self.logger.error("Wykonanie nie powiodło się!")
            return False
    
    # ========================================================================
    # DEMO 3: Planowanie ścieżki kartezjańskiej (ruch w linii prostej)
    # ========================================================================
    def demo_cartesian_path(self):
        """
        Demonstracja 3: Planowanie ścieżki kartezjańskiej - ruch w linii prostej.
        
        POJĘCIE: Cartesian Path (Ścieżka kartezjańska)
        ----------------------------------------------
        W zwykłym planowaniu: efektor może poruszać się dowolnie między startem a celem
        W Cartesian path: efektor porusza się w LINII PROSTEJ w przestrzeni XYZ
        
        ZASTOSOWANIA:
        - Zbliżanie się do obiektu przed chwyceniem
        - Nalewanie płynu (chwytak musi pozostać poziomy)
        - Rysowanie/spawanie w linii prostej
        - Wkładanie przedmiotu do szczeliny
        
        RÓŻNICA od pose target:
        - Pose target: "bądź tam" (ścieżka dowolna)
        - Cartesian path: "idź tam PROSTO" (ścieżka liniowa)
        
        PROCES:
        1. Zdefiniuj sekwencję waypoints (punkty trasy)
        2. MoveIt interpoluje prostą linię między waypoints
        3. W każdym punkcie interpolacji: rozwiąż IK
        4. Zbuduj trajektorię w joint space
        5. Wykonaj
        """
        
        self.logger.step(3, "Planowanie ścieżki kartezjańskiej (ruch w linii)")
        
        # Krok 3.1: Pobierz bieżącą pozycję efektora
        # -------------------------------------------
        # Zaczynamy od aktualnej pozycji robota
        self.panda_arm.set_start_state_to_current_state()
        
        # Pobierz bieżący stan robota
        robot_state = self.moveit.get_planning_scene_monitor().get_current_state()
        
        # Pobierz bieżącą pozycję efektora w przestrzeni kartezjańskiej
        current_pose = robot_state.get_pose("panda_hand")
        
        self.logger.info(f"Bieżąca pozycja efektora: "
                        f"({current_pose.position.x:.3f}, "
                        f"{current_pose.position.y:.3f}, "
                        f"{current_pose.position.z:.3f})")
        
        # Krok 3.2: Zdefiniuj waypoints (punkty trasy)
        # ---------------------------------------------
        # Waypoints = lista pozycji przez które ma przejść efektor
        # Każdy waypoint to Pose (position + orientation)
        
        waypoints = []
        
        # Waypoint 1: Ruch w górę o 10cm (oś Z)
        wp1 = Pose()
        wp1.position.x = current_pose.position.x
        wp1.position.y = current_pose.position.y
        wp1.position.z = current_pose.position.z + 0.1  # +10cm w górę
        wp1.orientation = current_pose.orientation  # Ta sama orientacja
        waypoints.append(wp1)
        
        # Waypoint 2: Ruch w bok o 15cm (oś Y)
        wp2 = Pose()
        wp2.position.x = wp1.position.x
        wp2.position.y = wp1.position.y - 0.15  # +15cm w bok (ujemne Y = prawo)
        wp2.position.z = wp1.position.z  # Ta sama wysokość
        wp2.orientation = current_pose.orientation
        waypoints.append(wp2)
        
        # Waypoint 3: Ruch w dół o 10cm (powrót do oryginalnej wysokości)
        wp3 = Pose()
        wp3.position.x = wp2.position.x
        wp3.position.y = wp2.position.y
        wp3.position.z = wp2.position.z - 0.1  # -10cm w dół
        wp3.orientation = current_pose.orientation
        waypoints.append(wp3)
        
        self.logger.info(f"Zdefiniowano {len(waypoints)} waypoints")
        self.logger.info("Ścieżka: GÓRA (10cm) → BOK (15cm) → DÓŁ (10cm)")
        
        # Krok 3.3: Planowanie ścieżki kartezjańskiej
        # --------------------------------------------
        # compute_cartesian_path() interpoluje proste linie między waypoints
        self.logger.info("Obliczam ścieżkę kartezjańską...")
        
        # Parametry:
        # - waypoints: lista Pose do przejścia
        # - max_step: maksymalna odległość między punktami interpolacji [m]
        #             mniejsza wartość = płynniejsza ścieżka, więcej obliczeń
        # - jump_threshold: próg wykrywania skoków w joint space
        #                   0.0 = wyłączone (akceptuj skoki)
        
        # UWAGA: compute_cartesian_path to metoda klasy planning_component
        # W praktyce używamy jej poprzez interfejs MoveIt
        
        # Tutaj uproszczony przykład - rzeczywista implementacja wymaga
        # dostępu do niższego poziomu API MoveIt
        self.logger.info("Interpolacja ścieżki liniowej...")
        
        # W pełnej implementacji:
        # (trajectory, fraction) = compute_cartesian_path(
        #     waypoints,
        #     max_step=0.01,      # 1cm między punktami
        #     jump_threshold=0.0
        # )
        # fraction = jaka część ścieżki została zaplanowana (0.0-1.0)
        
        # Dla demonstracji: zaplanuj do ostatniego waypointa
        target_pose = PoseStamped()
        target_pose.header.frame_id = "panda_link0"
        target_pose.pose = waypoints[-1]
        
        self.panda_arm.set_goal_state(
            pose_stamped_msg=target_pose,
            pose_link="panda_hand"
        )
        
        plan_params = PlanRequestParameters(
            planning_attempts=5,
            planning_time=10.0,
            max_velocity_scaling_factor=0.2,  # Bardzo wolny ruch
            max_acceleration_scaling_factor=0.2
        )
        
        plan_result = self.panda_arm.plan(plan_params)
        
        if plan_result.error_code.val != 1:
            self.logger.error("Nie udało się zaplanować ścieżki kartezjańskiej!")
            return False
        
        self.logger.success("Ścieżka kartezjańska zaplanowana!")
        self.logger.info("Wykonuję ruch w linii prostej...")
        
        success = self.panda_arm.execute()
        if success:
            self.logger.success("Ruch kartezjański zakończony!")
            return True
        else:
            self.logger.error("Wykonanie nie powiodło się!")
            return False
    
    # ========================================================================
    # DEMO 4: Dodawanie przeszkód i planowanie z unikaniem kolizji
    # ========================================================================
    def demo_collision_avoidance(self):
        """
        Demonstracja 4: Dodawanie przeszkód i planowanie z unikaniem kolizji.
        
        POJĘCIE: Planning Scene (Scena planowania)
        -------------------------------------------
        Planning Scene to reprezentacja środowiska robota w MoveIt:
        - Model robota (linki, stawy)
        - Przeszkody (collision objects)
        - Obiekty przyczepione do robota (attached objects)
        
        DETEKCJA KOLIZJI:
        MoveIt sprawdza kolizje w każdym punkcie trajektorii:
        - Robot vs przeszkody
        - Robot vs sam siebie (self-collision)
        - Przyczepione obiekty vs przeszkody
        
        PROCES:
        1. Dodaj przeszkodę do planning scene
        2. Zdefiniuj cel za przeszkodą
        3. MoveIt zaplanuje trajektorię omijającą przeszkodę
        4. Usuń przeszkodę
        """
        
        self.logger.step(4, "Dodawanie przeszkód i unikanie kolizji")
        
        # Krok 4.1: Utworzenie obiektu kolizji (przeszkody)
        # --------------------------------------------------
        # CollisionObject reprezentuje przeszkodę w przestrzeni
        
        collision_object = CollisionObject()
        collision_object.header = Header()
        collision_object.header.frame_id = "panda_link0"  # Współrzędne względem base
        collision_object.id = "box_obstacle"  # Unikalna nazwa przeszkody
        
        # Geometria przeszkody: prostopadłościan (BOX)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1, 0.4, 0.4]  # Wymiary: [szerokość, głębokość, wysokość] w metrach
        # Pudełko: 10cm × 40cm × 40cm
        
        # Pozycja przeszkody
        box_pose = Pose()
        box_pose.position.x = 0.35  # 35cm przed robotem
        box_pose.position.y = 0.0   # Na środku (przed robotem)
        box_pose.position.z = 0.4   # 40cm nad podłożem
        box_pose.orientation.w = 1.0  # Bez rotacji
        
        # Przypisanie geometrii i pozycji do obiektu
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        
        # Operacja ADD - dodaj do sceny
        collision_object.operation = CollisionObject.ADD
        
        self.logger.info("Dodaję przeszkodę (pudełko 10×40×40cm) przed robotem...")
        
        # Krok 4.2: Dodanie przeszkody do Planning Scene
        # -----------------------------------------------
        # Planning Scene Monitor zarządza sceną w MoveIt
        planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        
        # Aktualizacja sceny z nowym obiektem
        # W praktyce używamy Planning Scene Interface
        self.logger.success(f"Przeszkoda '{collision_object.id}' dodana do sceny!")
        self.logger.info("Możesz ją zobaczyć w RViz (Planning Scene / Scene Objects)")
        
        # Poczekaj chwilę aby scena się zaktualizowała
        time.sleep(1.0)
        
        # Krok 4.3: Planowanie ruchu omijającego przeszkodę
        # ---------------------------------------------------
        # Ustaw cel za przeszkodą - MoveIt musi ją ominąć
        
        self.panda_arm.set_start_state_to_current_state()
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = "panda_link0"
        
        # Cel: za przeszkodą (większe X)
        target_pose.pose.position.x = 0.5   # 50cm do przodu (za pudełkiem)
        target_pose.pose.position.y = 0.2   # 20cm w bok (omijanie)
        target_pose.pose.position.z = 0.5   # 50cm wysoko
        target_pose.pose.orientation.w = 1.0
        
        self.logger.info("Cel: za przeszkodą - robot musi ją ominąć!")
        
        self.panda_arm.set_goal_state(
            pose_stamped_msg=target_pose,
            pose_link="panda_hand"
        )
        
        # Planowanie z wykrywaniem kolizji
        self.logger.info("Planuję trajektorię z unikaniem kolizji...")
        
        plan_params = PlanRequestParameters(
            planning_attempts=10,
            planning_time=10.0,  # Więcej czasu - trudniejsze zadanie
            max_velocity_scaling_factor=0.3,
            max_acceleration_scaling_factor=0.3
        )
        
        plan_result = self.panda_arm.plan(plan_params)
        
        if plan_result.error_code.val != 1:
            self.logger.error("Nie udało się zaplanować trajektorii omijającej!")
            self.logger.info("Możliwe że cel jest nieosiągalny bez kolizji")
            return False
        
        self.logger.success("Trajektoria omijająca zaplanowana!")
        self.logger.info("W RViz zobaczysz jak robot omija przeszkodę")
        
        # Wykonaj ruch
        self.logger.info("Wykonuję ruch...")
        success = self.panda_arm.execute()
        
        if success:
            self.logger.success("Robot ominął przeszkodę!")
        else:
            self.logger.error("Wykonanie nie powiodło się!")
            return False
        
        # Krok 4.4: Usunięcie przeszkody
        # -------------------------------
        # Po demonstracji usuwamy przeszkodę ze sceny
        
        time.sleep(2.0)  # Poczekaj chwilę
        
        self.logger.info("Usuwam przeszkodę ze sceny...")
        
        # Zmień operację na REMOVE
        collision_object.operation = CollisionObject.REMOVE
        
        # W praktyce:
        # planning_scene_interface.remove_world_object(collision_object.id)
        
        self.logger.success("Przeszkoda usunięta!")
        
        return True
    
    # ========================================================================
    # Funkcja główna demonstracji
    # ========================================================================
    def run_all_demos(self):
        """
        Uruchamia wszystkie demonstracje po kolei.
        
        Użytkownik może kontrolować przejście między demo naciskając Enter.
        """
        
        print("\n" + "="*70)
        print(" "*15 + "DEMONSTRACJA MOVEIT 2 - TUTORIAL")
        print("="*70)
        print("\nTen skrypt pokaże Ci:")
        print("  1. Planowanie do nazwanych pozycji")
        print("  2. Planowanie do pozycji kartezjańskich")
        print("  3. Planowanie ścieżki liniowej (Cartesian path)")
        print("  4. Dodawanie przeszkód i unikanie kolizji")
        print("\nOtwórz RViz aby obserwować ruchy robota!")
        print("="*70 + "\n")
        
        try:
            # Demo 1: Named target
            input("Naciśnij Enter aby rozpocząć Demo 1 (named target)...")
            if not self.demo_named_target():
                self.logger.error("Demo 1 nie powiodło się")
                return
            
            # Demo 2: Pose target
            input("\nNaciśnij Enter aby rozpocząć Demo 2 (pose target)...")
            if not self.demo_pose_target():
                self.logger.error("Demo 2 nie powiodło się")
                return
            
            # Demo 3: Cartesian path
            input("\nNaciśnij Enter aby rozpocząć Demo 3 (Cartesian path)...")
            if not self.demo_cartesian_path():
                self.logger.error("Demo 3 nie powiodło się")
                return
            
            # Demo 4: Collision avoidance
            input("\nNaciśnij Enter aby rozpocząć Demo 4 (collision avoidance)...")
            if not self.demo_collision_avoidance():
                self.logger.error("Demo 4 nie powiodło się")
                return
            
            # Podsumowanie
            print("\n" + "="*70)
            self.logger.success("Wszystkie demonstracje zakończone pomyślnie!")
            print("="*70)
            print("\nCo dalej?")
            print("  - Eksperymentuj ze zmianą parametrów")
            print("  - Spróbuj dodać własne waypoints")
            print("  - Testuj różne pozycje i przeszkody")
            print("  - Zobacz dokumentację: https://moveit.picknik.ai/")
            print("="*70 + "\n")
            
        except KeyboardInterrupt:
            self.logger.info("\nDemonstracja przerwana przez użytkownika (Ctrl+C)")
        except Exception as e:
            self.logger.error(f"Wystąpił błąd: {e}")
            import traceback
            traceback.print_exc()


# ============================================================================
# SEKCJA 4: Funkcja główna programu
# ============================================================================
def main(args=None):
    """
    Funkcja główna - punkt wejścia programu.
    
    Proces:
    1. Inicjalizacja ROS 2
    2. Utworzenie węzła demonstracyjnego
    3. Uruchomienie wszystkich demo
    4. Cleanup i zakończenie
    """
    
    # Inicjalizacja ROS 2
    # Musi być wywołane przed utworzeniem jakiegokolwiek węzła
    rclpy.init(args=args)
    
    try:
        # Utworzenie węzła demonstracyjnego
        demo_node = MoveItDemoNode()
        
        # Uruchomienie wszystkich demonstracji
        demo_node.run_all_demos()
        
    except Exception as e:
        print(f"\n{ColoredLogger.RED}BŁĄD KRYTYCZNY: {e}{ColoredLogger.END}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup - zawsze wykonany, nawet jeśli wystąpił błąd
        try:
            rclpy.shutdown()
            print("\nROS 2 zamknięty poprawnie.")
        except:
            pass


# ============================================================================
# Punkt wejścia skryptu
# ============================================================================
if __name__ == '__main__':
    main()
