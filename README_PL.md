# MoveIt 2 - Framework do Planowania Ruchu Robotów

<img src="https://moveit.ai/assets/logo/moveit_logo-black.png" alt="MoveIt Logo" width="200"/>

## O projekcie MoveIt 2

**MoveIt 2** to zaawansowany framework open-source do planowania ruchu robotów w środowisku ROS 2 (Robot Operating System 2). Jest to potężne narzędzie umożliwiające programowanie i sterowanie manipulatorami robotycznymi oraz robotami mobilnymi.

### Czym jest MoveIt 2?

MoveIt 2 to kompleksowa platforma, która:
- **Planuje trajektorię ruchu** - oblicza bezpieczne ścieżki przemieszczania się efektora końcowego robota
- **Wykrywa kolizje** - zapobiega zderzeniom z przeszkodami w środowisku pracy
- **Rozwiązuje kinematykę** - przekształca docelowe pozycje w przestrzeni na kąty stawów robota
- **Integruje czujniki** - wykorzystuje kamery i skanery 3D do percepcji otoczenia
- **Steruje robotami** - komunikuje się z kontrolerami sprzętowymi robotów

## Dlaczego używamy MoveIt 2?

### Główne zalety:

1. **Łatwość użycia** - wysokopoziomowe API ukrywa złożoność algorytmów planowania
2. **Modularność** - można wybierać różne algorytmy planowania (OMPL, STOMP, Pilz)
3. **Wizualizacja** - integracja z RViz pozwala na podgląd planowanych ruchów
4. **Społeczność** - aktywna społeczność ROS i bogata dokumentacja
5. **Kompatybilność** - działa z wieloma modelami robotów przemysłowych i badawczych

## Struktura repozytorium

```
moveit2/
├── moveit_core/              # Podstawowe funkcjonalności (kinematyka, planowanie, kolizje)
├── moveit_ros/               # Integracja z ROS 2 (węzły, topiki, serwisy)
│   ├── planning/             # Komponenty planowania ruchu
│   ├── perception/           # Przetwarzanie danych z czujników
│   └── moveit_servo/         # Sterowanie w czasie rzeczywistym (servoing)
├── moveit_planners/          # Algorytmy planowania trajektorii
│   ├── ompl/                 # Open Motion Planning Library
│   ├── stomp/                # Stochastic Trajectory Optimization
│   └── pilz/                 # Planowanie przemysłowe (liniowe, kołowe)
├── moveit_py/                # Biblioteka Python dla MoveIt 2
├── moveit_setup_assistant/   # Narzędzie konfiguracyjne GUI
└── moveit_plugins/           # Wtyczki rozszerzeń
```

## Instalacja

### Instalacja binarna (zalecana dla początkujących)

```bash
# Instalacja MoveIt 2 dla ROS 2 Humble
sudo apt update
sudo apt install ros-humble-moveit
```

### Kompilacja ze źródeł (dla zaawansowanych)

```bash
# Utworzenie workspace ROS 2
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src

# Sklonowanie repozytorium
git clone https://github.com/ros-planning/moveit2.git -b main

# Instalacja zależności
cd ~/ws_moveit2
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y

# Kompilacja
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Pierwsze kroki

### 1. Uruchomienie prostego przykładu

```bash
# Aktywacja środowiska ROS 2
source /opt/ros/humble/setup.bash

# Jeśli kompilowałeś ze źródeł
source ~/ws_moveit2/install/setup.bash

# Uruchomienie demonstracyjnej sceny z robotem Panda
ros2 launch moveit2_tutorials demo.launch.py
```

### 2. Podstawowe pojęcia

#### Planning Scene
**Co to jest?** Reprezentacja środowiska pracy robota, zawierająca modele przeszkód i kolizji.

**Po co?** Umożliwia planowanie bezpiecznych trajektorii, które unikają kolizji.

#### Move Group
**Co to jest?** Zestaw stawów/linków robota, którymi sterujemy jako całością (np. ramię, chwytak).

**Po co?** Pozwala na planowanie skoordynowanych ruchów wielu stawów jednocześnie.

#### End Effector (Efektor końcowy)
**Co to jest?** Narzędzie na końcu ramienia robota (np. chwytak, spawarka).

**Po co?** Określa punkt, którym chcemy manipulować w przestrzeni roboczej.

#### Joint Space vs Cartesian Space
- **Joint Space** - przestrzeń kątów stawów (wartości dla każdego silnika)
- **Cartesian Space** - przestrzeń kartezjańska (pozycja X,Y,Z i orientacja)

**Po co oba?** Robot planuje w Joint Space, ale my często podajemy cele w Cartesian Space.

## Zastosowanie z robotem Unitree G1 EDU

Robot **Unitree G1 EDU** to humanoidalny robot badawczy, który doskonale współpracuje z MoveIt 2. 

### Dlaczego MoveIt 2 dla Unitree G1?

1. **Złożona kinematyka** - humanoid ma wiele stopni swobody, MoveIt 2 automatyzuje skomplikowane obliczenia
2. **Bezpieczeństwo** - wykrywanie kolizji chroni robota przed samouszkodzeniem
3. **Planowanie całego ciała** - możliwość koordynacji ruchów ramion, nóg i tułowia
4. **Symulacja** - testowanie algorytmów przed wdrożeniem na prawdziwym robocie

### Typowe zadania z G1 EDU:

- **Manipulacja obiektami** - podnoszenie, przenoszenie, odkładanie przedmiotów
- **Interakcja człowiek-robot** - podawanie obiektów, współpraca przy zadaniach
- **Nawigacja i manipulacja** - połączenie poruszania się z użyciem ramion
- **Gestykulacja** - wykonywanie wyrażeń niewerbalnych i gestów

## Zasoby dla studentów

### Dokumentacja
- **Oficjalne tutoriale**: [https://moveit.picknik.ai/](https://moveit.picknik.ai/)
- **MoveIt 2 Python**: Zobacz [moveit_py/README.md](moveit_py/README.md)
- **API Doxygen**: Dokumentacja techniczna klas i funkcji

### Przykładowe projekty
1. **Pick and Place** - podnoszenie i odkładanie obiektów
2. **Trajectory Planning** - planowanie gładkich trajektorii
3. **Collision Avoidance** - unikanie przeszkód dynamicznych
4. **Visual Servoing** - sterowanie oparte na obrazie z kamery

### Dodatkowe przewodniki
- **[PRZEWODNIK_STUDENTA.md](PRZEWODNIK_STUDENTA.md)** - szczegółowy przewodnik krok po kroku
- **[UNITREE_G1_INTEGRATION.md](UNITREE_G1_INTEGRATION.md)** - integracja z robotem Unitree G1 EDU

## Kluczowe komponenty

### 1. moveit_core
**Cel:** Podstawowa logika planowania i kinematyki.

**Zawiera:**
- Rozwiązywanie kinematyki odwrotnej (IK) i prostej (FK)
- Detekcja kolizji
- Reprezentacja modelu robota
- Zarządzanie ograniczeniami ruchu

### 2. moveit_ros
**Cel:** Integracja z ekosystemem ROS 2.

**Zawiera:**
- Węzeł `move_group` - główny serwer planowania
- Interfejsy do komunikacji (action servers, services)
- Monitorowanie sceny (planning scene monitor)
- Integracja z kontrolerami sprzętowymi

### 3. moveit_planners
**Cel:** Różne algorytmy planowania trajektorii.

**Algorytmy:**
- **OMPL** (Open Motion Planning Library) - próbkowanie przestrzeni konfiguracji
- **STOMP** - optymalizacja stochastyczna, unika lokalnych minimów
- **Pilz** - planowanie przemysłowe (proste ruchy liniowe i kołowe)

### 4. moveit_py
**Cel:** Pythonowy interfejs do MoveIt 2.

**Zalety:**
- Prostsze prototypowanie
- Interaktywne środowisko (Jupyter notebooks)
- Dostęp do ekosystemu Python (NumPy, OpenCV, ML)

## Status integracji ciągłej

[![CI (Rolling and Humble)](https://github.com/ros-planning/moveit2/actions/workflows/ci.yaml/badge.svg?branch=main)](https://github.com/ros-planning/moveit2/actions/workflows/ci.yaml?query=branch%3Amain)

## Wkład w projekt

Jesteś studentem i chciałbyś przyczynić się do rozwoju MoveIt 2? 

- **Zgłaszanie błędów**: [GitHub Issues](https://github.com/ros-planning/moveit2/issues)
- **Propozycje ulepszeń**: [GitHub Discussions](https://github.com/ros-planning/moveit2/discussions)
- **Pull requesty**: Zobacz [CONTRIBUTING.md](CONTRIBUTING.md)

## Wsparcie i pomoc

### Gdzie szukać pomocy?

1. **Discord MoveIt**: [Dołącz do serwera](https://discord.gg/RrySut8)
2. **ROS Answers**: [https://answers.ros.org](https://answers.ros.org)
3. **GitHub Discussions**: Zadaj pytanie w tym repozytorium

### Często spotykane problemy

**Problem:** "Package 'moveit_ros' not found"
**Rozwiązanie:** Upewnij się, że sourced środowisko ROS 2:
```bash
source /opt/ros/humble/setup.bash
```

**Problem:** Planer nie znajduje rozwiązania
**Rozwiązanie:** Sprawdź czy:
- Cel jest osiągalny (w workspace robota)
- Nie ma kolizji na starcie lub celu
- Timeout planowania jest wystarczający

## Licencja

MoveIt 2 jest opublikowany na licencji BSD - zobacz [LICENSE.txt](LICENSE.txt).

## Podziękowania

Ten projekt jest rozwijany przez społeczność ROS na całym świecie. Specjalne podziękowania dla:
- **PickNik Robotics** - główny rozwój i utrzymanie
- **Projektu ROSIN** - wsparcie portowania do ROS 2
- **Wszystkich kontrybutorów** - za nieustanny wkład w rozwój

---

**Uwaga:** Ten dokument został stworzony jako pomoc edukacyjna dla polskojęzycznych studentów. Wszystkie nazwy techniczne (klasy, funkcje, pakiety) pozostają w oryginalnej formie angielskiej, zgodnie ze standardami programowania.
