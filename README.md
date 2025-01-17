# Projekt: Modelowanie dynamiki ruchu pojazdu

## Wprowadzenie do projektu

Projekt symulacji ruchu łazika mobilnego został opracowany jako narzędzie edukacyjne i badawcze, mające na celu zrozumienie dynamiki ruchu robotów mobilnych oraz sposobów ich sterowania.  
Symulacja bazuje na równaniach różniczkowych opisujących ruch w dwóch wymiarach, uwzględniając:

- Prędkość liniową,
- Prędkość kątową,
- Orientację względem układu współrzędnych.

Wykorzystano zaawansowane metody numeryczne, takie jak integracja ODE (zwykłych równań różniczkowych), co pozwala na dokładne odwzorowanie trajektorii ruchu w dynamicznie zmieniających się warunkach.

Dzięki dodatkowej integracji systemów ROS 2 oraz symulacji Gazebo projekt umożliwia dynamiczne modelowanie bardziej złożonych scenariuszy ruchu, uwzględniając czynniki takie jak masa łazika, tarcie czy nierówności terenu.

---

## Cele projektu

1. **Modelowanie dynamicznego ruchu:**
   Symulacja odwzorowuje płynne zmiany prędkości liniowej i kątowej w realistycznych warunkach robotycznych.
   
2. **Integracja równań różniczkowych i różnicowych:**
   Użycie metod matematycznych pozwala na dynamiczne obliczanie orientacji oraz trajektorii łazika.
   
3. **Tworzenie narzędzia edukacyjnego:**
   Projekt stanowi pomoc dydaktyczną dla studentów i badaczy zajmujących się kinematyką i dynamiką robotów mobilnych.
   
4. **Rozszerzalność:**
   Projekt można rozbudować o elementy sztucznej inteligencji, systemy sterowania oparte na ROS 2 oraz integrację z rzeczywistymi czujnikami.

---

## Wykorzystane równania

### Równania kinematyczne ruchu łazika
Ruch łazika jest opisany za pomocą równań różniczkowych:
- Położenie w osi $(x)$ i $(y)$:
  $$
  \frac{dx}{dt} = v \cos(\theta), \quad \frac{dy}{dt} = v \sin(\theta)
  $$
- Zmiana orientacji kątowej:
  $$
  \frac{d\theta}{dt} = \omega
  $$
  Gdzie:
  - $(x, y)$ – współrzędne pozycji łazika,
  - $(\theta)$ – orientacja łazika względem osi $(x)$,
  - $(v)$ – prędkość liniowa,
  - $(\omega)$ – prędkość kątowa.

---

### Uwzględnienie ruchu kół
Prędkość kątowa wynika z różnicy prędkości kół:
$$
\omega = \frac{v_R - v_L}{d}
$$
Gdzie:
- $(v_R)$, $(v_L)$ – prędkości kół prawego i lewego,
- $(d)$ – odległość między kołami (rozstaw osi).

---

### Rozwiązanie równań różniczkowych
Symulacja korzysta z metody numerycznej integracji:
$$
\text{state}(t) = \text{odeint} \big( \text{diff\_drive\_ode}, \text{state}(t_0), t, \text{args}=(v, \omega) \big)
$$
Gdzie:
- $(\text{odeint})$ – funkcja z biblioteki `scipy`,
- $(\text{state}(t))$ – stan łazika (pozycja $(x, y)$ i orientacja $(\theta)$) w chwili $(t)$.

---

## Wykorzystane narzędzia

1. **Język programowania:**
   - Python
2. **Biblioteki:**
   - `scipy` (metoda `odeint` do rozwiązywania równań różniczkowych),
   - `numpy` (operacje numeryczne),
   - `matplotlib` (wizualizacja trajektorii).
3. **Systemy robotyczne:**
   - ROS 2 (Robot Operating System),
   - Gazebo (symulacja 3D ruchu robotów).
4. **Metody matematyczne:**
   - Równania różniczkowe zwyczajne (ODE),
   - Równania różnicowe.

---

## Wizualizacja wyników

1. **Trajektoria ruchu:**
   Symulacja rysuje ścieżkę łazika w przestrzeni 2D, pokazując wpływ zmian prędkości i orientacji w czasie.

2. **Dynamiczne zmiany prędkości:**
   Wykresy prędkości liniowej i kątowej w funkcji czasu.

3. **Interaktywna wizualizacja:**
   Integracja z Gazebo pozwala na podgląd 3D trajektorii łazika w wirtualnym środowisku.

---

## Zastosowanie

Projekt może być używany w:
- Edukacji, jako materiał dydaktyczny dla studentów robotyki,
- Badaniach, do symulowania algorytmów sterowania i optymalizacji,
- Prototypowaniu, w kontekście rozwoju robotów mobilnych.

# Konfiguracja i uruchomienie

Projekt został podzielony na moduły znajdujące się w odpowiednich folderach:

## ROS 2 Workspace ( `/ros2_ws`)

Zawiera wszystkie pakiety ROS 2 niezbędne do uruchomienia oprogramowa symulującego równania różnicowe

## TrailblazerML ROS2 Project ( `/TrailblazerML`)

Zawiera projekt oparty na ROS2, korzystający z dystrybucji „humble”. Projekt integruje symulację, wizualizację, teleoperację i sterowania robotem.

## Streamlit frontend ( `/frontend`)

Odpowiada za interfejs użytkownika oraz wizualizację wyników symulacji równań różniczkowych.
