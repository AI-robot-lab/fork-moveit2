# Szybki Start - MoveIt 2 dla Studentów

## 🎯 Witaj w świecie robotyki z MoveIt 2!

Ten dokument pomoże Ci szybko rozpocząć pracę z MoveIt 2, szczególnie jeśli pracujesz nad projektem z robotem humanoidalnym **Unitree G1 EDU**.

## 📚 Struktura dokumentacji

Przygotowaliśmy dla Ciebie kompleksowy zestaw materiałów edukacyjnych w języku polskim:

### 1. **[README_PL.md](README_PL.md)** - Wprowadzenie
   - **Co to jest MoveIt 2** i dlaczego go używamy
   - **Struktura repozytorium** i główne komponenty
   - **Podstawowe pojęcia** (Planning Scene, Move Group, Kinematyka)
   - **Instalacja i pierwsze uruchomienie**
   - Zastosowanie z robotem Unitree G1 EDU

   👉 **Zacznij tutaj** jeśli pierwszy raz słyszysz o MoveIt 2

### 2. **[PRZEWODNIK_STUDENTA.md](PRZEWODNIK_STUDENTA.md)** - Szczegółowy tutorial
   - **Podstawy teoretyczne** (ROS 2, kinematyka, planowanie)
   - **Konfiguracja środowiska** krok po kroku
   - **Pierwsze programy** z wyjaśnieniami
   - **Zaawansowane koncepcje** (Planning Scene, Constraints, Cartesian paths)
   - **Debugowanie** i rozwiązywanie problemów
   - **Projekty praktyczne** (Pick and Place, Obstacle Avoidance)

   👉 **Czytaj po kolei** aby nauczyć się programowania robotów

### 3. **[UNITREE_G1_INTEGRATION.md](UNITREE_G1_INTEGRATION.md)** - Integracja z G1
   - **Specyfikacja robota** Unitree G1 EDU
   - **Architektura systemu** (MoveIt + ROS 2 Control + Hardware)
   - **Konfiguracja kompletna** (URDF, SRDF, kontrolery)
   - **Przykład bimanual manipulation** (manipulacja obiema rękami)
   - **Zaawansowane tematy** (Whole-body planning, Visual servoing)

   👉 **Przejdź tutaj** gdy będziesz gotowy do pracy z prawdziwym robotem

### 4. **[moveit_py_example_pl.py](moveit_py_example_pl.py)** - Przykładowy kod
   - **Kompletny działający program** z polskimi komentarzami
   - **4 demonstracje**: named targets, pose targets, Cartesian paths, collision avoidance
   - **Szczegółowe wyjaśnienia** każdej linijki kodu
   - **Gotowy do uruchomienia** i modyfikacji

   👉 **Uruchom ten skrypt** aby zobaczyć MoveIt 2 w akcji

## 🚀 Szybki start w 3 krokach

### Krok 1: Instalacja (5-10 minut)

```bash
# Zainstaluj ROS 2 Humble i MoveIt 2
sudo apt update
sudo apt install ros-humble-moveit

# Zainstaluj tutoriale demonstracyjne
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros-planning/moveit2_tutorials.git -b humble
cd ~/ros2_ws
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build

# Dodaj do .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Krok 2: Pierwsze uruchomienie (2 minuty)

```bash
# Terminal 1: Uruchom demo z robotem Panda
ros2 launch moveit2_tutorials demo.launch.py
```

W RViz zobaczysz robota Panda. Możesz:
- Przeciągnąć interaktywny marker (kolorowe osie XYZ)
- Kliknąć **"Plan"** aby zaplanować trajektorię
- Kliknąć **"Execute"** aby wykonać ruch

### Krok 3: Uruchom przykładowy skrypt (5 minut)

```bash
# Terminal 2 (demo musi być uruchomione w Terminal 1)
cd ~/ros2_ws/src/moveit2  # lub gdzie sklonowałeś to repozytorium
python3 moveit_py_example_pl.py
```

Skrypt przeprowadzi Cię przez 4 demonstracje. Obserwuj ruchy robota w RViz!

## 📖 Ścieżka nauki

### Dla początkujących (0-2 tygodnie):
1. ✅ Przeczytaj [README_PL.md](README_PL.md) - zrozumienie podstaw
2. ✅ Uruchom demo w RViz - eksperymentuj ręcznie
3. ✅ Przeczytaj [PRZEWODNIK_STUDENTA.md](PRZEWODNIK_STUDENTA.md) sekcje 1-3
4. ✅ Uruchom i modyfikuj [moveit_py_example_pl.py](moveit_py_example_pl.py)
5. ✅ Napisz własny prosty skrypt (ruch do 3 różnych pozycji)

### Dla średniozaawansowanych (2-4 tygodnie):
1. ✅ Przeczytaj [PRZEWODNIK_STUDENTA.md](PRZEWODNIK_STUDENTA.md) sekcje 4-5
2. ✅ Zaimplementuj projekt "Pick and Place"
3. ✅ Dodaj własne przeszkody do sceny
4. ✅ Eksperymentuj z różnymi plannerami (OMPL, STOMP, Pilz)
5. ✅ Naucz się debugowania (logi, visualization markers)

### Dla zaawansowanych - Unitree G1 (4-8 tygodni):
1. ✅ Przeczytaj [UNITREE_G1_INTEGRATION.md](UNITREE_G1_INTEGRATION.md)
2. ✅ Skonfiguruj URDF i SRDF dla robota G1
3. ✅ Zaimplementuj bimanual manipulation
4. ✅ Eksperymentuj z whole-body planning
5. ✅ Integracja z kamerą (visual servoing)
6. ✅ Projekt finalny: autonomiczne zadanie manipulacji

## 🎓 Projekty studenckie - pomysły

### Łatwe (1-2 tygodnie):
- **Sortowanie obiektów**: Robot segreguje kolorowe klocki
- **Układanie wieży**: Precyzyjne odkładanie obiektów jeden na drugim
- **Śledzenie obiektu**: Robot śledzi ruchomy obiekt (camera feedback)

### Średnie (3-4 tygodnie):
- **Współpraca człowiek-robot**: Podawanie narzędzi operatorowi
- **Autonomiczne nawigowanie**: Robot chodzi i manipuluje obiektami
- **Bimanual assembly**: Składanie obiektów obiema rękami

### Trudne (6-8 tygodni):
- **Whole-body manipulation**: Wykorzystanie nóg do zwiększenia zasięgu
- **Dynamic obstacle avoidance**: Planowanie w środowisku z ruchomymi przeszkodami
- **Learning from demonstration**: Robot uczy się zadań obserwując człowieka

## 🔧 Narzędzia i zasoby

### Dokumentacja
- **Oficjalne tutoriale MoveIt 2**: https://moveit.picknik.ai/
- **ROS 2 dokumentacja**: https://docs.ros.org/en/humble/
- **URDF Tutorial**: http://wiki.ros.org/urdf/Tutorials

### Społeczność i pomoc
- **Discord MoveIt**: https://discord.gg/RrySut8
- **ROS Answers**: https://answers.ros.org/
- **GitHub Discussions**: https://github.com/ros-planning/moveit2/discussions

### Wizualizacja i debugowanie
- **RViz2**: Główne narzędzie wizualizacji ROS
- **PlotJuggler**: Wykresy danych w czasie rzeczywistym
- **rqt**: Zestaw narzędzi GUI dla ROS

## ❓ FAQ - Najczęściej zadawane pytania

### P: Czy muszę znać C++ aby używać MoveIt 2?
**O:** Nie! MoveIt 2 ma doskonałe API Python (`moveit_py`). Wszystkie przykłady w tym repo są w Pythonie.

### P: Czy mogę używać MoveIt 2 bez prawdziwego robota?
**O:** Tak! Możesz pracować w pełni w symulacji (Gazebo, RViz). Świetne do nauki i testowania algorytmów.

### P: Ile czasu zajmuje nauka MoveIt 2?
**O:** 
- **Podstawy**: 1-2 tygodnie (proste ruchy, planowanie)
- **Średni poziom**: 1-2 miesiące (projekty, integracje)
- **Zaawansowany**: 3-6 miesięcy (whole-body, research topics)

### P: Co jeśli MoveIt nie znajduje planu?
**O:** Sprawdź:
1. Czy cel jest w zasięgu robota?
2. Czy nie ma kolizji?
3. Czy wystarczający timeout planowania?
4. Spróbuj innego plannera (RRT → RRT\*)
Zobacz sekcję "Debugowanie" w [PRZEWODNIK_STUDENTA.md](PRZEWODNIK_STUDENTA.md)

### P: Jak pracować z robotem Unitree G1?
**O:** Przeczytaj [UNITREE_G1_INTEGRATION.md](UNITREE_G1_INTEGRATION.md) - kompleksowy przewodnik integracji.

### P: Gdzie mogę znaleźć więcej przykładów?
**O:** 
- `moveit2_tutorials` (oficjalne tutoriale)
- `moveit_py/test/` (testy jednostkowe = przykłady użycia)
- GitHub: szukaj "moveit2 examples"

## 🎯 Następne kroki

Teraz gdy masz przegląd całej dokumentacji:

1. **Przeczytaj** [README_PL.md](README_PL.md) aby zrozumieć podstawy
2. **Uruchom** demo i przykładowy skrypt
3. **Studiuj** [PRZEWODNIK_STUDENTA.md](PRZEWODNIK_STUDENTA.md) krok po kroku
4. **Eksperymentuj** - modyfikuj przykłady, testuj własne pomysły
5. **Buduj** własne projekty używając MoveIt 2

## 📝 Uwagi końcowe

**Konwencje nazewnictwa:**
- Wszystkie nazwy techniczne (klasy, funkcje, pakiety) pozostają **w języku angielskim**
- Komentarze i wyjaśnienia są **po polsku**
- To standardowa praktyka w międzynarodowych projektach

**Wersje oprogramowania:**
- Dokumentacja zakłada **ROS 2 Humble** (LTS - Long Term Support)
- Dla innych dystrybucji (Iron, Rolling) może wymagać drobnych adaptacji

**Feedback:**
Jeśli znajdziesz błędy lub masz sugestie ulepszeń:
- Zgłoś issue na GitHub
- Zaproponuj poprawkę (Pull Request)
- Zapytaj na Discord MoveIt

---

**Powodzenia w nauce robotyki! 🤖**

*Dokument stworzony dla polskojęzycznych studentów studiujących robotykę i pracujących z robotami humanoidalnymi.*
