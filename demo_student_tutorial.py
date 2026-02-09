#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
==============================================================================
SKRYPT DEMONSTRACYJNY MOVEIT 2 DLA STUDENTÓW PRz
==============================================================================

Plik: demo_student_tutorial.py
Autor: Opracowano dla studentów Politechniki Rzeszowskiej
Data: 2026

CEL:
Ten skrypt jest kompletnym przykładem użycia MoveIt 2, który przeprowadza
studenta przez wszystkie podstawowe operacje krok po kroku.

WYMAGANIA:
- ROS 2 Humble
- MoveIt 2
- Robot Panda lub Unitree G1 (skonfigurowany w MoveIt)

URUCHOMIENIE:
1. W pierwszym terminalu:
   ros2 launch moveit2_tutorials demo.launch.py
   
2. W drugim terminalu:
   python3 demo_student_tutorial.py

==============================================================================
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningSceneInterface
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import time
import sys


class StudentDemoNode(Node):
    """
    Node demonstracyjny dla studentów.
    
    Ten node przeprowadza przez podstawowe operacje MoveIt 2:
    1. Inicjalizacja systemu
    2. Planowanie do nazwanej pozycji
    3. Planowanie do pozycji w przestrzeni kartezjańskiej
    4. Praca z przeszkodami w Planning Scene
    5. Planowanie trajektorii kartezjańskiej
    """
    
    def __init__(self):
        """Inicjalizacja node'a i MoveIt."""
        super().__init__('student_demo_node')
        
        self.print_header("INICJALIZACJA SYSTEMU")
        
        try:
            # Krok 1: Inicjalizacja MoveItPy
            self.get_logger().info("⏳ Inicjalizacja MoveItPy...")
            self.moveit = MoveItPy(node_name="moveit_demo")
            self.get_logger().info("✓ MoveItPy zainicjalizowany!")
            
            # Krok 2: Pobranie grupy planowania
            # UWAGA: Zmień "panda_arm" na nazwę grupy z twojego robota
            # Dla Unitree G1: "left_arm" lub "right_arm"
            self.arm_group_name = "panda_arm"
            
            available_groups = self.moveit.get_group_names()
            self.get_logger().info(f"📋 Dostępne grupy: {available_groups}")
            
            if self.arm_group_name not in available_groups:
                self.get_logger().error(f"❌ Grupa '{self.arm_group_name}' nie istnieje!")
                self.get_logger().error(f"   Dostępne grupy: {available_groups}")
                sys.exit(1)
            
            self.arm = self.moveit.get_planning_component(self.arm_group_name)
            self.get_logger().info(f"✓ Grupa '{self.arm_group_name}' gotowa!")
            
            # Krok 3: Inicjalizacja Planning Scene Interface
            self.planning_scene = PlanningSceneInterface()
            self.get_logger().info("✓ Planning Scene Interface gotowy!")
            
            # Poczekaj chwilę na synchronizację
            time.sleep(2)
            
            self.get_logger().info("✓ System gotowy do pracy!\n")
            
        except Exception as e:
            self.get_logger().error(f"❌ Błąd inicjalizacji: {e}")
            raise
    
    def print_header(self, text):
        """Wyświetla ładny nagłówek w konsoli."""
        print("\n" + "=" * 80)
        print(f"  {text}")
        print("=" * 80 + "\n")
    
    def wait_for_user(self, message="Naciśnij Enter aby kontynuować..."):
        """Czeka na potwierdzenie użytkownika."""
        input(f"\n{message}\n")
    
    def demo_1_named_target(self):
        """
        DEMO 1: Planowanie do nazwanej pozycji
        
        KONCEPCJA:
        Nazwane pozycje (named targets) to wstępnie zdefiniowane konfiguracje
        robota zapisane w pliku SRDF. Są one bardzo wygodne, bo nie musisz
        pamiętać kątów stawów ani pozycji w przestrzeni.
        
        TYPOWE NAZWANE POZYCJE:
        - "home" - pozycja domowa/bezpieczna
        - "ready" - pozycja gotowości do pracy
        - "extended" - ramię wyprostowane
        
        ZALETY:
        + Szybkie - nie trzeba rozwiązywać IK
        + Przewidywalne - zawsze ta sama konfiguracja
        + Nazwane znacząco - łatwo zapamiętać
        """
        
        self.print_header("DEMO 1: Planowanie do nazwanej pozycji")
        
        self.get_logger().info("📖 TEORIA:")
        self.get_logger().info("   Nazwane pozycje (named targets) to wstępnie zdefiniowane")
        self.get_logger().info("   konfiguracje stawów zapisane w pliku SRDF robota.")
        self.get_logger().info("   Planowanie do nich jest szybkie, bo nie wymaga kinematyki odwrotnej.")
        
        self.wait_for_user("Naciśnij Enter aby zaplanować ruch do pozycji 'ready'...")
        
        try:
            # Krok 1: Ustaw stan początkowy
            self.get_logger().info("1️⃣  Ustawianie stanu początkowego jako aktualny...")
            self.arm.set_start_state_to_current_state()
            self.get_logger().info("   ✓ Stan początkowy ustawiony")
            
            # Krok 2: Ustaw cel jako nazwaną pozycję
            target_name = "ready"
            self.get_logger().info(f"2️⃣  Ustawianie celu: pozycja '{target_name}'...")
            self.arm.set_goal_state(configuration_name=target_name)
            self.get_logger().info(f"   ✓ Cel ustawiony: '{target_name}'")
            
            # Krok 3: Zaplanuj trajektorię
            self.get_logger().info("3️⃣  Planowanie trajektorii...")
            plan_result = self.arm.plan()
            
            if plan_result:
                trajectory = self.arm.get_plan_trajectory()
                num_points = len(trajectory.joint_trajectory.points)
                self.get_logger().info(f"   ✓ Trajektoria zaplanowana!")
                self.get_logger().info(f"   📊 Liczba punktów: {num_points}")
                
                self.wait_for_user("Naciśnij Enter aby WYKONAĆ ruch...")
                
                # Krok 4: Wykonaj trajektorię
                self.get_logger().info("4️⃣  Wykonywanie trajektorii...")
                success = self.moveit.execute(trajectory, blocking=True)
                
                if success:
                    self.get_logger().info("   ✓ Ruch wykonany pomyślnie!")
                else:
                    self.get_logger().error("   ❌ Błąd wykonania trajektorii")
            else:
                self.get_logger().error("   ❌ Nie udało się zaplanować trajektorii")
                self.get_logger().error("   💡 Możliwe przyczyny:")
                self.get_logger().error("      - Pozycja 'ready' nie jest zdefiniowana w SRDF")
                self.get_logger().error("      - Robot jest w kolizji")
                
        except Exception as e:
            self.get_logger().error(f"❌ Błąd w demo 1: {e}")
    
    def demo_2_cartesian_target(self):
        """
        DEMO 2: Planowanie do pozycji w przestrzeni kartezjańskiej
        
        KONCEPCJA:
        Planowanie do pozycji w przestrzeni (Pose Goal) pozwala określić gdzie
        dokładnie ma być end-effector w układzie współrzędnych 3D.
        
        POSE składa się z:
        - Position (x, y, z) - pozycja w metrach
        - Orientation (quaternion) - orientacja w przestrzeni
        
        MoveIt automatycznie:
        1. Rozwiązuje kinematykę odwrotną (IK) - znajduje kąty stawów
        2. Planuje bezkolizyjną trajektorię
        3. Sprawdza czy cel jest osiągalny
        
        ZASTOSOWANIA:
        - Chwytanie obiektów w znanej pozycji
        - Interakcja z otoczeniem
        - Zadania wymagające precyzyjnego pozycjonowania
        """
        
        self.print_header("DEMO 2: Planowanie do pozycji kartezjańskiej")
        
        self.get_logger().info("📖 TEORIA:")
        self.get_logger().info("   Pose Goal = Pozycja (x,y,z) + Orientacja (quaternion)")
        self.get_logger().info("   MoveIt automatycznie rozwiązuje kinematykę odwrotną (IK)")
        self.get_logger().info("   aby znaleźć kąty stawów osiągające zadaną pozę.")
        
        self.wait_for_user("Naciśnij Enter aby zaplanować ruch do pozycji w przestrzeni...")
        
        try:
            # Krok 1: Utwórz pozę docelową
            target_pose = Pose()
            
            # Pozycja: 40cm przed robotem, 10cm w prawo, 50cm w górę
            target_pose.position = Point(x=0.4, y=0.1, z=0.5)
            
            # Orientacja: quaternion dla "chwytaka skierowanego w dół"
            # Ten quaternion reprezentuje obrót o 90° wokół osi X
            target_pose.orientation = Quaternion(x=0.707, y=0.0, z=0.0, w=0.707)
            
            self.get_logger().info("1️⃣  Cel zdefiniowany:")
            self.get_logger().info(f"   📍 Pozycja: x={target_pose.position.x}m, "
                                 f"y={target_pose.position.y}m, z={target_pose.position.z}m")
            self.get_logger().info(f"   🔄 Orientacja: quaternion(x={target_pose.orientation.x:.3f}, "
                                 f"y={target_pose.orientation.y:.3f}, "
                                 f"z={target_pose.orientation.z:.3f}, "
                                 f"w={target_pose.orientation.w:.3f})")
            
            # Krok 2: Ustaw stan początkowy i docelowy
            self.arm.set_start_state_to_current_state()
            
            # UWAGA: Nazwa end-effectora może się różnić!
            # Dla Panda: "panda_hand" lub "panda_link8"
            # Dla G1: "r_hand_link" lub "l_hand_link"
            end_effector_link = "panda_link8"
            
            self.get_logger().info(f"2️⃣  Ustawianie pose goal dla '{end_effector_link}'...")
            self.arm.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link=end_effector_link
            )
            
            # Krok 3: Planuj trajektorię
            self.get_logger().info("3️⃣  Planowanie (może zająć chwilę - rozwiązywanie IK)...")
            plan_result = self.arm.plan()
            
            if plan_result:
                trajectory = self.arm.get_plan_trajectory()
                self.get_logger().info("   ✓ Kinematyka odwrotna rozwiązana!")
                self.get_logger().info("   ✓ Trajektoria zaplanowana!")
                
                self.wait_for_user("Naciśnij Enter aby WYKONAĆ ruch...")
                
                # Krok 4: Wykonaj
                self.get_logger().info("4️⃣  Wykonywanie...")
                success = self.moveit.execute(trajectory, blocking=True)
                
                if success:
                    self.get_logger().info("   ✓ End-effector osiągnął cel!")
                else:
                    self.get_logger().error("   ❌ Błąd wykonania")
            else:
                self.get_logger().error("   ❌ Nie udało się zaplanować trajektorii")
                self.get_logger().error("   💡 Możliwe przyczyny:")
                self.get_logger().error("      - Pozycja poza zasięgiem robota (workspace)")
                self.get_logger().error("      - Brak rozwiązania IK dla tej orientacji")
                self.get_logger().error("      - Kolizja z przeszkodami lub samym sobą")
                
        except Exception as e:
            self.get_logger().error(f"❌ Błąd w demo 2: {e}")
    
    def demo_3_collision_objects(self):
        """
        DEMO 3: Praca z przeszkodami w Planning Scene
        
        KONCEPCJA:
        Planning Scene to wewnętrzna reprezentacja środowiska robota.
        Zawiera:
        - Model robota
        - Przeszkody w otoczeniu
        - Załączone obiekty (np. trzymane przez chwytaka)
        
        MoveIt automatycznie unika kolizji z przeszkodami podczas planowania.
        
        TYPY PRZESZKÓD:
        - Box (prostopadłościan) - dla stołów, ścian, pudełek
        - Cylinder (cylinder) - dla słupków, butelek
        - Mesh (siatka) - dla złożonych kształtów
        
        ZASTOSOWANIA:
        - Planowanie w zagraconym środowisku
        - Symulacja rzeczywistych przeszkód
        - Testowanie bezpieczeństwa trajektorii
        """
        
        self.print_header("DEMO 3: Planowanie z przeszkodami")
        
        self.get_logger().info("📖 TEORIA:")
        self.get_logger().info("   Planning Scene przechowuje informacje o przeszkodach.")
        self.get_logger().info("   MoveIt automatycznie planuje trajektorie omijające przeszkody.")
        
        self.wait_for_user("Naciśnij Enter aby dodać przeszkodę...")
        
        try:
            # Krok 1: Dodaj przeszkodę (stół)
            self.get_logger().info("1️⃣  Dodawanie stołu przed robotem...")
            
            table_pose = PoseStamped()
            table_pose.header.frame_id = "world"
            table_pose.pose.position = Point(x=0.5, y=0.0, z=0.25)
            table_pose.pose.orientation = Quaternion(w=1.0)
            
            self.planning_scene.add_box(
                name="table",
                pose=table_pose,
                size=(0.6, 1.0, 0.02)  # 60cm x 100cm x 2cm
            )
            
            time.sleep(1)  # Daj czas na update Planning Scene
            self.get_logger().info("   ✓ Stół dodany! (Powinieneś go zobaczyć w RViz)")
            self.get_logger().info("   📦 Wymiary: 60cm x 100cm x 2cm")
            self.get_logger().info("   📍 Pozycja: 50cm przed robotem")
            
            self.wait_for_user("Sprawdź RViz. Naciśnij Enter aby dodać drugą przeszkodę...")
            
            # Krok 2: Dodaj drugą przeszkodę (pudełko)
            self.get_logger().info("2️⃣  Dodawanie pudełka na stole...")
            
            box_pose = PoseStamped()
            box_pose.header.frame_id = "world"
            box_pose.pose.position = Point(x=0.5, y=0.2, z=0.35)
            box_pose.pose.orientation = Quaternion(w=1.0)
            
            self.planning_scene.add_box(
                name="obstacle_box",
                pose=box_pose,
                size=(0.1, 0.1, 0.1)  # Kostka 10cm
            )
            
            time.sleep(1)
            self.get_logger().info("   ✓ Pudełko dodane!")
            self.get_logger().info("   📦 Wymiary: 10cm x 10cm x 10cm")
            
            self.wait_for_user("Naciśnij Enter aby zaplanować ruch omijający przeszkody...")
            
            # Krok 3: Zaplanuj ruch w otoczeniu przeszkód
            self.get_logger().info("3️⃣  Planowanie ruchu omijającego przeszkody...")
            
            # Cel: pozycja blisko pudełka (ale go nie uderzając)
            target_pose = Pose()
            target_pose.position = Point(x=0.5, y=0.3, z=0.4)
            target_pose.orientation = Quaternion(x=0.707, y=0.0, z=0.0, w=0.707)
            
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link="panda_link8"
            )
            
            plan_result = self.arm.plan()
            
            if plan_result:
                self.get_logger().info("   ✓ Trajektoria zaplanowana!")
                self.get_logger().info("   ✓ Planner znalazł ścieżkę omijającą przeszkody!")
                
                self.wait_for_user("Naciśnij Enter aby WYKONAĆ ruch...")
                
                trajectory = self.arm.get_plan_trajectory()
                success = self.moveit.execute(trajectory, blocking=True)
                
                if success:
                    self.get_logger().info("   ✓ Ruch wykonany - robot ominął przeszkody!")
                else:
                    self.get_logger().error("   ❌ Błąd wykonania")
            else:
                self.get_logger().error("   ❌ Nie udało się zaplanować trajektorii")
                self.get_logger().error("   💡 Możliwe, że cel jest nieosiągalny bez kolizji")
            
            # Krok 4: Czyszczenie sceny
            self.wait_for_user("Naciśnij Enter aby usunąć przeszkody...")
            
            self.get_logger().info("4️⃣  Usuwanie przeszkód...")
            self.planning_scene.remove_world_object("table")
            self.planning_scene.remove_world_object("obstacle_box")
            time.sleep(1)
            self.get_logger().info("   ✓ Przeszkody usunięte!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Błąd w demo 3: {e}")
    
    def demo_4_summary(self):
        """Podsumowanie demonstracji."""
        
        self.print_header("PODSUMOWANIE")
        
        self.get_logger().info("🎓 Gratulacje! Przeszedłeś przez podstawowe operacje MoveIt 2:")
        self.get_logger().info("")
        self.get_logger().info("✓ DEMO 1: Planowanie do nazwanych pozycji")
        self.get_logger().info("  └─ Szybkie, przewidywalne, idealne do standardowych pozycji")
        self.get_logger().info("")
        self.get_logger().info("✓ DEMO 2: Planowanie do pozycji w przestrzeni")
        self.get_logger().info("  └─ Precyzyjne, wymaga IK, używane do manipulacji")
        self.get_logger().info("")
        self.get_logger().info("✓ DEMO 3: Praca z przeszkodami")
        self.get_logger().info("  └─ Automatyczne unikanie kolizji, bezpieczne planowanie")
        self.get_logger().info("")
        self.get_logger().info("📚 CO DALEJ?")
        self.get_logger().info("  1. Przeczytaj przewodniki w repozytorium:")
        self.get_logger().info("     - README_PL.md")
        self.get_logger().info("     - STUDENT_GUIDE_PL.md")
        self.get_logger().info("     - UNITREE_G1_GUIDE_PL.md")
        self.get_logger().info("")
        self.get_logger().info("  2. Eksperymentuj z kodem:")
        self.get_logger().info("     - Zmień pozycje docelowe")
        self.get_logger().info("     - Dodaj więcej przeszkód")
        self.get_logger().info("     - Spróbuj różnych orientacji")
        self.get_logger().info("")
        self.get_logger().info("  3. Przejrzyj annotated demo w:")
        self.get_logger().info("     moveit_ros/moveit_servo/demos/cpp_interface/")
        self.get_logger().info("     - demo_twist.cpp (sterowanie prędkościami)")
        self.get_logger().info("     - demo_pose.cpp (śledzenie pozycji)")
        self.get_logger().info("     - demo_joint_jog.cpp (bezpośrednie sterowanie stawami)")
        self.get_logger().info("")
        self.get_logger().info("🤖 Powodzenia w projektach z robotyką!")
        self.get_logger().info("")


def main():
    """Główna funkcja programu."""
    
    # Inicjalizacja ROS 2
    rclpy.init()
    
    try:
        # Utwórz node demonstracyjny
        demo_node = StudentDemoNode()
        
        # Powitanie
        demo_node.print_header("WITAMY W DEMONSTRACJI MOVEIT 2!")
        demo_node.get_logger().info("Ten skrypt przeprowadzi Cię przez podstawowe operacje MoveIt 2.")
        demo_node.get_logger().info("Upewnij się, że:")
        demo_node.get_logger().info("  ✓ RViz jest uruchomiony")
        demo_node.get_logger().info("  ✓ Widzisz model robota")
        demo_node.get_logger().info("  ✓ Planning Scene Monitor działa")
        demo_node.wait_for_user()
        
        # Demonstracje
        demo_node.demo_1_named_target()
        demo_node.demo_2_cartesian_target()
        demo_node.demo_3_collision_objects()
        demo_node.demo_4_summary()
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Przerwano przez użytkownika (Ctrl+C)")
    except Exception as e:
        print(f"\n\n❌ Wystąpił błąd: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Sprzątanie
        rclpy.shutdown()


if __name__ == '__main__':
    main()
