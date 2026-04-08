# LPODRO
LPODRO to tania, wysoko zautomatyzowana i kompaktowa maszyna do domowej produkcji płytek PCB oraz gotowych obwodów elektronicznych. Została ona zaprojektowana z myślą o makerach, hobbystach, amatorach i małych pracowniach. Zastępuje procesy chemiczne precyzyjnym frezowaniem ścieżek i otworów oraz dodaje moduł pick-and-place do montażu elementów SMD na wykonywanych płytkach. System wizyjny (kamera sufitowa + kamera w blacie roboczym) automatyzuje kalibrację, inspekcję i korekcję położenia elementów. Projekt jest open-source, co umożliwia każdemu budowę i modyfikację maszyny do szybkiego prototypowania układów.

## Co to robi:

Sednem LPODRO jest prosty, zautomatyzowany przepływ pracy: użytkownik dostarcza laminat i elementy SMD, a maszyna przeprowadza kolejne etapy procesu. Kamery określają położenia dostarczonych materiałów oraz kontrolują każdy etap produkcji. Wrzeciono zamontowane na osi frezującej precyzyjnie frezuje ścieżki i otwory montażowe co eliminuje potrzebę stosowania przestarzałych i niebezpiecznych chemicznych procesów produkcji płytek, przekształcając surowy laminat w funkcjonalną płytkę PCB. Następnie moduł PnP umieszcza elementy SMD na płytce z wysoką dokładnością, a zintegrowany moduł lutowniczy który zostanie niebawem dodany doprowadzi do trwałego połączenia elementów z płytką — tak, by użytkownik przy minimalnej pracy i bez potrzeby posiadania rozbudowanej wiedzy otrzymał w pełni gotowy, działający układ.

## Konstrukcja maszyny:

Konstrukcja mechaniczna LPODRO oparta jest na autorskim pomyśle ramy ze stalowych prętów połączonych specjalnymi łącznikami przypominającymi łączniki kątowe drukowane w 3D. Rozwiązanie to pozwola na proste i ekstremalnie tanie wykonanie ramy, która jest podstawą całej maszyny. To na nią są wsuwane kolejne części takie jak uchwyty na silniki czy osie. Układ napędowy osi X, Y i Z wrzeciona oparty jest na klasycznym mechanizmie dwóch prowadnic i jednej śruby trapezowej T8 napędzanej silnikiem krokowym Nema17. Maszyna wyposażona jest jednak w dwie osie Z: pierwsza porusza wrzecionem, druga — zamocowana na uchwycie wrzeciona — obsługuję głowicę PNP. Dzięki temu możliwe jest niezależne sterowanie wrzecionem i głowicą, co pozwala na elastyczność operacji (frezowanie, pobieranie i układanie komponentów) przy jednoczesnym uproszczeniu mechaniki.

## Głowica PnP:

Głowica PnP została stworzona od podstaw na potrzeby tego projektu. Jest ona zbudowana z elementów powrzechnie dostępnych i bardzo tanich zachowując jednocześnie funkcjonalność komercyjnie dostępnych odpowiedników. Posiada ona mechanizm łatwej wymiany igeł które są powszechnie dostępne w najróżniejszych rozmiarach. Posiada ona również wbudowany silnik krokowy który współpracując z systemem wizyjnym obraca odpowiednio podnoszone elementy tak aby trafiły one w odpowiedniej pozycji na wykonywaną płytkę PCB. System ruchowy głowicy wykorzystuje niestandardowy, prosty napęd oparty na nawijanym sznurku: silnik nawijając lub odwijając linkę zmienia jej efektywną długość, co przekłada się na ruch góra-dół głowicy. Takie rozwiązanie zapewnia niskie koszty, prostotę konstrukcji i dużą odporność na uszkodzenia mechaniczne — w razie kolizji głowica „dobije” do blatu zamiast ulec uszkodzeniu.

## Automatyczne lutowanie układów:

Projekt przewiduje w przyszłości dołączenie do konstrukcji maszyny zintegrowaną płytę grzewczą. Będzie ona umieszczona w blacie roboczym a po umieszczeniu na niej płytki z naniesioną pastą lutowniczą system wyreguluje profil temperaturowy i jednorodnie rozgrzeje PCB do temperatury reflow, co pozwoli na masowe i powtarzalne zespolenie elementów SMD bez konieczności ręcznego lutowania pojedynczych elementów. Rozwiązanie to — integracja heat plate w maszynę — znacząco podnosi poziom automatyzacji i ergonomii pracy, skracając czas operacji i zmniejszając liczbę interwencji użytkownika. Pozowli ono też zamknąć w obrębie maszyny cały proces produkcji obwodu elektronicznego - stworzenie płytki PCB, naniesienia na nią elementów oraz przytluowania ich do płytki tworząc w ten sposób jendo kompletne urządzenie do produkcji układów elektronicznych. Kontrola profilu grzania (czujnik temperatury + regulacja) zapewni bezpieczeństwo montażu i powtarzalność wyników przy różnych typach płytek i komponentów.

## System wizyjny:

Kluczowym elementem systemu są dwie kamery: kamera sufitowa umieszczona nad blatem wykonuje zdjęcia całej powierzchni roboczej w celu lokalizacji laminatu, automatycznego ustalenia optymalnego punktu zerowego (minimalizującego odpady materiałowe), wykrywania pozycji elementów SMD przygotowanych przez użytkownika do montażu przez maszynę. Druga kamera została osadzona równo z powierzchnią blatu — nad obiektyw przejeżdża chwytany element SMD, a system wizyjny określa jego orientację i offset względem igły PnP, co pozwola na precyzyjne skorygowanie rotacji i położenia przed odłożeniem — mechanizm identyczny w koncepcji z przemysłowymi urządzeniami PnP, ale zaimplementowany w sposób ekonomiczny.

## Elektronika sterująca:

Elektronika sterująca oparta jest na powszechnie dostępnych elementach: jako podstawę użyte zostało Arduino Nano z firmware GRBL do obsługi osi X, Y i Z przy użyciu sterowników krokowych A4988. Równolegle pojawia się drugi układ — Raspberry Pi zero w. Aktualnie steruję on silnikami krokowymi głowicy PnP oraz pracą wrzeciona i pompy próżniowej. Wszystkie inne procesy (generowanie gcode, auto leveling, przesyłanie informacji do maszyny, odbieranie sygnału z kamer itd.) odbywa się na razie na komputerze który stale musi być podłączony do maszyny. W przyszłości wszystkie te zadania przejmie Raspberry Pi co sprawi że do obsługi maszyny wystarczy dowolne urzadzenie z którego będzie można przesłać plik gerber PCB oraz plik współrzędnych PnP - spsób działania będzie podobny do współczesnych drukarek 3d. By móc osiągnąć ten cel przewidzane jest też dodanie do maszyny ekranu oraz guzików.

## Oprogramowanie:

Oprogramowanie sterujące LPODRO zostało napisane od podstaw ze względu na to że dostępne rozwiązania (zarówno komercyjne, jak i open source) nie odpowiaday w pełni wymaganym założeniom automatyzacji i wygody użytkowania. Software odpowiadać za pełną automatyzację procesu: autokalibrację, analizę obrazu, generowanie trajektorii frezu, sekwencjonowanie operacji pick-and-place, korekcję pozycji za pomocą informacji z kamer oraz obsługę błędów i procedur bezpieczeństwa. Obecnie interfejs użytkownika jest w formie CLI. W przyszłości zostanie stworzona nakładka GUI która znacząco podniesie łatwość korzystania i intuicyjność. Obecnie oprogramowanie umożliwia import popularnych formatów projektów PCB, parametryzację procesu oraz monitoring przebiegu zadań. Całe oprogramowanie zostało napisane w języku Python.

## Przykłady:

**Film przedstawiający działanie maszyny:** https://www.youtube.com/watch?v=HcF9H6V-rpg

# UWAGA: WIP

**Projekt nadal znajduje się w fazie work in progress!**

Przedstawiony tu projekt, modele 3d, oprogramowanie itd. nie są jeszcze skończone ani dopracowane. Brakuje dokumentacji, wielu funkcji, jest dużo znanych błędów i problemów a wiele czeka jeszcze na odnalezienie. Strona ta służy jako sposób na łatwe poznanie projektu, śledzenia postępu jego rozwoju a w niedalekiej przyszłości posłuży jako źródło wiedzy i materiałów które będzie można wykorzystać podczas budowy i eksploatacji własnej maszyny w zaciszu swojego warsztatu. 

Nie zalecam budowania maszyny na typ etapie z dostępnych modeli 3d i używania jej z obecnym oprogramowaniem. Są one udostępnione po to aby móc się z nimi zapoznać, zobaczyć jak to działa, z czego składa się projekt i ogólnie z czym to się je. Niestrudzenie pracuję nad projektem tak aby jak najszybciej doprowadzić go do stanu w którym każdy będzie mógł go zbudować w sposób najłatwiejszy jak to tylko możliwe. 



