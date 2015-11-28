/*
Solar controler new version at: http://sq9mdd.qrz.pl
Copyright Rysiek Labus SQ9MDD
 
 Program udostępniony na licencji GPL V.2
 
 Solar Controler jest wolnym oprogramowaniem; możesz go rozprowadzać dalej
 i/lub modyfikować na warunkach Powszechnej Licencji Publicznej GNU,
 wydanej przez Fundację Wolnego Oprogramowania - według wersji 2 tej
 Licencji lub (według twojego wyboru) którejś z późniejszych wersji.
 
 Niniejszy program rozpowszechniany jest z nadzieją, iż będzie on
 użyteczny - jednak BEZ JAKIEJKOLWIEK GWARANCJI, nawet domyślnej
 gwarancji PRZYDATNOŚCI HANDLOWEJ albo PRZYDATNOŚCI DO OKREŚLONYCH
 ZASTOSOWAŃ. W celu uzyskania bliższych informacji sięgnij do
 Powszechnej Licencji Publicznej GNU.
 
 ACCU AGN Żelówka tabela napięcia spoczynkowego i pojemności
 100% - 12.8V
 90% -  12.5V
 80% - 12.4V
 70% - 12.3V
 60% - 12.2V
 50% - 12.1V
 40% - 11.9V
 30% - 11.8V
 20% - 11.6V
 10% - 11.3V
 0% - 10.5V BATTERY DEATH
 
 PWM DUTY
 0 - 0%
 25 - 10%
 64 - 25%
 127 - 50%
 191 - 75%
 242 - 95%
 255 - 100%
 
 ROADMAP
 1. Wykresy na LCD
 2. Dodać obsługę czujnika natężenia prądu i limity prądu
 3. Dodać obsługę czujnika temperatury akumulatorów i zaimplementować ogranieczenie prądów i napięć w funkcji temperatury
 4. Integracja z domoticzem, protokół mysensors
 
 CHANGELOG
 2015.11.27 v.2.8 zmiana PCB dodanie obsługi ładowarki sieciowej
 2015.09.16 v.2.7 czyszczenie kodu, wyrzucenie pisania po eepromie, reset gdy nie pracuje ładowanie
 2014.10.19 v.2.6 dodanie watchdoga po zawieszeniu się sterownika
 2014.08.29 v.2.5 zmiana parametrów pracy, odcięcie z 113 na 110
   Zmiana PWM duty max do 250
   Dodane zabezpieczenie przed przeładowaniem gdy kompletnie brak odbioru przez długi czas i mamy masę światła
   Włączenie podświetlenia gdy ładowanie dobiega końca i mamy światło.
 2014.07.13 v.2.4 Czyszczenie kodu, funkcje debug przesunięte do własnej funkcji.
   Zapisywanie danych o napięciu akumulatora do EEPROM podczas przełączania trybu pracy. 
   Uzyskujemy pomiary do wykresów napięć po cyklu rozładowania i po cyklu ładowania.
   Dane te są wypluwane na port rs/usb podczas zmiany trybu pracy
 2014.07.05 v.2.3 Poprawka zapobiegająca zawieszaniu się urządzenia co 49 dni.
   Wybór częstotliwości pracy PWM przeniesiony do konfiguracji.
 2014.07.04 v.2.2 Drobne poprawki w kodzie, wersja po testach.
 2014.07.04 v.2.1 Zmienne użytkownika zmienione w const, zaoszczędzenie pamięci RAM o 44 bajty, wyczyszczone logo.
 2014.07.03 v.2.0 Zmiana sposobu numeracji wersji, zmiana biblioteki wyświetlacza na LCD5110 basic, dodane logo firmowe na starcie softu.
 2014.06.18 Zmiana pinologi, kompatybilne z PCB, finalna wersja algorytmu sterowania sterowanie propocjonalne dwie krzywe pracy.
 2014.06.17 główny regulator zmiana ze stało czasowego na proporcjonalny od krzywej setpoint panel i krzywej setpoint bateria
 2014.06.16 płynne ograniczenie prądu ładowania od poziomu ustalonego accu, dodanie obsługi podświetlenia LCD, debuger do kalibracji pomiarów napięć.
 2014.06.11 Kompletna zmiana pętli regulacyjnej, czyszczenie kodu. v.1.1
 2014.06.10 Drobne poprawki, zmiana PWM na 100Hz, pełne wspóldzielenie czasu, regulacja interwału pomiaru, regulacja interwału regulacji i lcd, poprawki w pętli MPPT
 2014.06.09 Dwie pętle regulacji, PWM standardowy 500Hz(do zmiany), Obsługa MPPT (Maximum Power Point Tracking) remastering kodu
 2014.06.08 Koncepcja i powstanie pierwszej wersji oprogramowania
 */
//wersja softu
const int soft_ver = 28;

//biblioteki
#include <avr/pgmspace.h>
#include <LCD5110_Basic.h>                       //podłączamy bibliotekę do obsługi wyświetlacza
//#include <EEPROM.h>
#include <avr/wdt.h>  //watchdog

//stałe (wejścia i wyjścia)
#define accu_mesure_in A1
#define solar_mesure_in A2                       //spalilem sobie wejscie A0 :-/
#define mosfet 9
#define load 8
#define podswietlenie 2
#define ext_power 12

/*************************************************************************************************************************************************************************************/
// zmienne ustawienia/dane wejsciowe do ostrożnej modyfikacji
const int debug = 1;                                    //debuger: 1-wykresy pracy pętli regulacyjnej, 2-odczyty przetwornika ADC z pomiaru napięć (do kalibracji)
const int accu_napiecie_max = 144;                      //maksymalny poziom napięcia baterii 14.4 * 10
const int accu_napiecie_pracy = 138;                    //jeśli mam pełno słońca i napięcie na akumulatorze jest wieksze nie napiecie standby to jest setpoint pracy układu
const int accu_napiecie_standby = 135;                  //jeśli jest mało słońca do tego poziomu ładuję akumulator, punkt przełączenia krzywych regulacji
const int accu_napiecie_min = 115;                      //jesli uklad nie laduje i napięcie spadnie do tego poziomu odpal ładowanie zewnętrzne
const int accu_napiecie_odciecia = 110;                 //napięcie przy którym odcinam obciążenie 10% po zdjeciu obciążenia było 50% zmieniam ze 113 na 110
const int accu_napiecie_wlaczenia = 121;                //napięcie przy którym ponownie włączam obciążenie 51% (histereza z odcięciem zależna od obciążenia)
const int accu_napiecie_spoczynkowe_full = 128;         //maksymalne napięcie spoczynkowe akumulatora (potrzebne do wyliczania procentowego naładowania akumulatora)
const int accu_death_lvl = 105;                         //krytyczny poziom napiecia baterii
const int map_input_low = 364;                          //minimum z przetwornika do kalibracji pomiaru V
const int map_input_high = 732;                         //maksimum z przetwornika do kalibracji pomiaru V
const int map_output_low = 100;                         //minimum V do kalibracji pomiaru napięcia  
const int map_output_high = 200;                        //maksimum V do kalibracji pomiaru napięcia
const unsigned long czestotliwosc_kaskady = 100;        //interwał uruchamiania petli regulacji (w ms)
const unsigned long czestotliwosc_showtime = 1000;      //refresh wyświetlacza LCD
const unsigned long czestotliwosc_pomiaru = 10;         //interwał pomiaru napięcia
const unsigned long czas_podtrzymania_osw_lcd = 10;     //czas podtrzymania podświetlenia LCD w sekundach
const unsigned long plot_interval = 500;                //interwał wysyłania danych na port szeregowy
const int solar_pomiar_filtr = 10;                      //filtr pomiaru napięcia, liczba próbek do wyciągnięcia średniej
const int przelicznik_podbicia_napiecia = 9;            //min 6 max 9 spadek na diodzie 0,7V plus 0,4V  dobrać doświadczalnie potrzebne do skutecznego ustawiania napięcia
const int pwm_duty_min = 1;                             //minimalna wartość PWM podczas ładowania nie schodzić poniżej 1
const int pwm_duty_max = 255;                           //maksymalna wartość PWM (244=95%) //zmiana do 100%
const int czestotliwosc_pwm = 100;                      //dopuszczalne wartosci to 100 lub 30
const int kontrast = 70;                                //kontrast wyświetlacza LCD
/*************************************************************************************************************************************************************************************/

//zmienne pomocnicze nie dotykać
char buffor[] = "              ";
int pwm_duty_first = 0;
int pwm_duty_second = 0;
int pwm_duty_second_tmp = 0;
int pwm_duty = 0;
int solar_v = 0;
int batt_v = 0;
int tryb_pracy = 0;
int poprzedni_tryb_pracy = 0;
unsigned long exec_time = 0;
unsigned long show_time = 0;
unsigned long plot_time = 0;
unsigned long mesure_time = 0;
unsigned long light_off_time = 0;
int solar_pomiar_temp = 0;
int accu_pomiar_temp = 0;
int solar_pomiar_licznik = 0;
int punkt_pracy_tmp = 0;
int uchyb = 0; 

//logo, chcesz zrobić swoje to odwiedź: http://www.henningkarlsen.com/electronics/t_imageconverter_mono.php
uint8_t sinux[] PROGMEM={
  0xFF, 0xFF, 0x7F, 0x3F, 0x1F, 0x0F, 0x0F, 0x8F, 0x8F, 0x8F, 0x8F, 0x8F, 0x0F, 0x0F, 0x1F, 0x3F,   // 0x0010 (16) pixels
  0x7F, 0xFF, 0xFF, 0xFF, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0x01, 0xF1, 0xF1, 0xF1, 0xF1,   // 0x0020 (32) pixels
  0xF1, 0xC1, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xF1, 0xF1, 0xF1, 0x01, 0x01, 0x01, 0x01, 0xF1,   // 0x0030 (48) pixels
  0xF1, 0xF1, 0xF1, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xF1, 0xF1, 0xF1, 0xF1, 0x01, 0x01,   // 0x0040 (64) pixels
  0x01, 0x11, 0x71, 0xF1, 0xF1, 0xF1, 0xC1, 0x01, 0x01, 0x01, 0x01, 0xC1, 0xE1, 0xF1, 0xF1, 0x31,   // 0x0050 (80) pixels
  0x11, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0x7C, 0x78, 0x70, 0x70, 0xE1, 0xE3, 0xE3, 0xC3, 0xC3, 0xC7,   // 0x0060 (96) pixels
  0x87, 0x06, 0x0E, 0x0E, 0x1E, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00,   // 0x0070 (112) pixels
  0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x07, 0x1F, 0x7E, 0xF8, 0xF0, 0xE0, 0xC0, 0xFF, 0xFF, 0xFF, 0x00,   // 0x0080 (128) pixels
  0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,   // 0x0090 (144) pixels
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC1, 0xE3, 0xF7, 0xFF, 0x3E, 0x7E, 0xFF, 0xFF,   // 0x00A0 (160) pixels
  0xE3, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFC, 0xF8, 0xF0, 0xF0, 0xF1, 0xE1,   // 0x00B0 (176) pixels
  0xE3, 0xE3, 0xE3, 0xE3, 0xE1, 0xF0, 0xF0, 0xF8, 0xFC, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0,   // 0x00C0 (192) pixels
  0xFF, 0xFF, 0xFF, 0x80, 0x9F, 0x9F, 0x9F, 0x9F, 0x80, 0x80, 0x80, 0x80, 0x80, 0x81, 0x87, 0x8F,   // 0x00D0 (208) pixels
  0x9F, 0x9F, 0x9F, 0x80, 0x80, 0x80, 0x80, 0x81, 0x83, 0x87, 0x8F, 0x8F, 0x9E, 0x9E, 0x9C, 0x9E,   // 0x00E0 (224) pixels
  0x9E, 0x8F, 0x8F, 0x87, 0x87, 0x81, 0x80, 0x80, 0x80, 0x9C, 0x9E, 0x9F, 0x8F, 0x87, 0x83, 0x80,   // 0x00F0 (240) pixels
  0x80, 0x80, 0x80, 0x83, 0x87, 0x8F, 0x9F, 0x9E, 0x98, 0x80, 0x80, 0xFF, 
};

// inicjalizujemy wyświetlacz, mapa połączeń
#define _sck 3
#define _din 4
#define _dc 5
#define _rst 7
#define _ce 6
LCD5110 lcd(_sck,_din,_dc,_rst,_ce); 
extern uint8_t SmallFont[]; //czcionka z biblioteki

//**************************************zaczynamy*****************************************
//pomiar napięcia z uśrednianiem wyników
void pomiar_napiecia(){
  //wykonujemy pomiar co określony w konfiguracji interwał
  //próbki dodajemy do siebie i co zadaną ilość próbek obliczamy średnią arytmetyczną
  //ta funkcja musi być wywoływana co Xms Xrazy (zmienna: solar_pomiar_filtr)
  solar_pomiar_temp = solar_pomiar_temp + analogRead(solar_mesure_in);
  accu_pomiar_temp = accu_pomiar_temp + analogRead(accu_mesure_in);
  solar_pomiar_licznik++;    
  if(solar_pomiar_licznik >= (solar_pomiar_filtr-1)){ 
    solar_pomiar_temp = solar_pomiar_temp/solar_pomiar_filtr; 
    accu_pomiar_temp = accu_pomiar_temp/solar_pomiar_filtr;
    solar_v = map(solar_pomiar_temp,map_input_low,map_input_high,map_output_low,map_output_high);  //parametry kalibracyjne rozbić na osobne zmienne
    batt_v = map(accu_pomiar_temp,map_input_low,map_input_high,map_output_low,map_output_high);    //parametry kalibracyjne rozbić na osobne zmienne
    solar_pomiar_licznik = 0; 
  }
}

//dodaję próbki napięcie do pamięci eeprom ostatnie 30 dni
//dwa zestawy po 30 próbek
//void multi_trend_saving(int zestaw){
//  int temp_eeprom = 0;
//  int aa = 0;
//  int bb = 0;
//  if(zestaw==0){
//    aa = 129;
//    bb = 101;
//  }
//  if(zestaw==1){
//    aa = 229;
//    bb = 201;    
//  }
//  //rejestr przesuwny
//  for(int a = aa; a >= bb; a--){
//    temp_eeprom = EEPROM.read(a);
//    delay(50);
//    EEPROM.write((a+1),temp_eeprom);
//  }
//  EEPROM.write(bb,batt_v);
//}

//wyswietlam dane zapisane w pamieci eeprom
//void multi_trend_printing(){
//  Serial.println("Printing accu V. log");
//  for(int a=1; a <= 30; a++){
//    Serial.print(EEPROM.read(a+100));
//    Serial.print(",");
//    Serial.println(EEPROM.read(a+200));
//  }
//  Serial.println("End of log");  
//}

//główna pętla regulacyjna układu
void regulacja(){
  //punkt pracy dla panela obliczamy dodając bieżące napięcie accu plus przelicznik do zniwelowania spadków napięć
  //następnie limitujemy punkt pracy panela dolny limit tyle co bateria pusta górny limit bateria max + przelicznik 
  //zmieniamy regulację na pracę setpoint accu po osiągnieciu accu standby  
  //krzywa punktu pracy plus jej ograniczanie
  punkt_pracy_tmp = batt_v + przelicznik_podbicia_napiecia;
  punkt_pracy_tmp = map(batt_v, batt_v, accu_napiecie_pracy, (batt_v + przelicznik_podbicia_napiecia), accu_napiecie_pracy);
  //zmiana uchybu w zależności od trybu pracy  
  if(batt_v <= accu_napiecie_standby) uchyb = abs(solar_v - punkt_pracy_tmp); 
  if(batt_v > accu_napiecie_standby) uchyb = abs(accu_napiecie_pracy - batt_v);  
  //głowna pętla regulacja obciążenia panela
  if(solar_v > punkt_pracy_tmp && batt_v < accu_napiecie_pracy){
    pwm_duty_second = pwm_duty_second + uchyb;
  }
  else{
    pwm_duty_second = pwm_duty_second - uchyb;
  }
  //koniec głównej pętli
  //sterowanie mosfetem i ograniczenie sygnału sterującego 
  pwm_duty = constrain(pwm_duty_second,pwm_duty_min,pwm_duty_max); 
  //zabezpieczenie przed przeładowaniem jeślibrak odbiorów i mamy mase swiatła
  if(batt_v <= 144){ 
    analogWrite(mosfet,pwm_duty);
  }
  else{
    analogWrite(mosfet,0);
  }
}

//obsługa LCD na razie full tekstowo
void pokaz_dane(){
  lcd.setFont(SmallFont);

  //solar 
  int s_v_prefix = solar_v / 10;
  int s_v_sufix = solar_v % 10;  
  sprintf(buffor, "SOL: %02u.%1uV",s_v_prefix,s_v_sufix);
  lcd.print(buffor,0,0);

  //accu 
  int a_v_prefix = batt_v / 10;
  int a_v_sufix =  batt_v % 10;
  sprintf(buffor, "ACC: %02u.%1uV",a_v_prefix,a_v_sufix);
  lcd.print(buffor,0,8);

  //wyświetlany tryb pracy i krzywa wskaźnika baterii zależne od trybu pracy
  int bat_percent = 0;
  if(tryb_pracy == 1){
    lcd.print("TRB: CHARGE   ", 0, 16);  
    bat_percent = map(batt_v,accu_death_lvl,accu_napiecie_standby,0,100);
  }
  else{
    lcd.print("TRB: DISCHARGE", 0, 16);
    bat_percent = map(batt_v,accu_death_lvl,accu_napiecie_spoczynkowe_full,0,100);
  }

  //obciążenie układu w %
  int pwm_percent = map(pwm_duty,0,255,0,100);
  sprintf(buffor,"LOD: %03u%s",pwm_percent,"%");
  lcd.print(buffor,0,24);

  //naładowanie baterii w % 
  bat_percent = constrain(bat_percent,0,100);
  sprintf(buffor,"BAT: %03u%s",bat_percent,"%");
  lcd.print(buffor,0,40);  
}

//gaszenie światełka
void zgas_swiatlo(){
  if(millis() >= light_off_time && digitalRead(podswietlenie) == 0){
    digitalWrite(podswietlenie,HIGH); //gasimy podświetlenie
    light_off_time = millis() + (czas_podtrzymania_osw_lcd * 1000);
  }
}

//osobna funkcja do debugowania algorytmu
//wyrzucam dane na port rs232 i rysuję wykresy
void plot_data(){
  // debugger potrzebne do kalibracji przetworników ADC 
  if(debug == 2){
    Serial.print(analogRead(solar_mesure_in));
    Serial.print(",");    
    Serial.print(solar_v);    
    Serial.print(",");
    Serial.print(batt_v);
    Serial.print(",");    
    Serial.println(analogRead(accu_mesure_in));
  }   
  // debugger wyświetlanie parametrów pracy
  if(debug == 1){
    Serial.print(pwm_duty);
    Serial.print(",");
    Serial.print(punkt_pracy_tmp);
    Serial.print(",");
    Serial.print(solar_v);
    Serial.print(",");
    Serial.print(batt_v);
    Serial.print(",");
    Serial.println(uchyb);
  } 
}

//setup
void setup(){
  if(czestotliwosc_pwm == 100){
    TCCR1B = TCCR1B & 0b11111000 | 0x04;  //0x04 100Hz zmiana częstotliwości PWM (aktualnie 100Hz)
  }
  else{
    TCCR1B = TCCR1B & 0b11111000 | 0x05; // 0x05 30Hz zmiana częstotliwości PWM
  }
  pinMode(solar_mesure_in, INPUT); 
  pinMode(accu_mesure_in, INPUT);
  pinMode(mosfet, OUTPUT);
  pinMode(load, OUTPUT);
  pinMode(podswietlenie, OUTPUT);
  pinMode(ext_power, OUTPUT);
  Serial.begin(9600);  
  lcd.InitLCD(kontrast);                                                 //inicjalizacja z kontrastem
  digitalWrite(load, LOW);                                               //enable load at start
  digitalWrite(podswietlenie, LOW);                                      //uruchamiam swiatełko na starcie
  digitalWrite(ext_power, LOW);                                          //gasze ladowanie zewnetrzne
  lcd.setFont(SmallFont);
  sprintf(buffor,"ver: %01u.%01u",soft_ver/10,soft_ver%10);              //wyświetlam wersje softu na starcie
  lcd.print(buffor,0,40);  
  lcd.drawBitmap(0, 15, sinux, 84, 20);                                  //wyświetlam logo sinux
  delay(5000);                                                           //przerwa na reklamy
  lcd.clrScr();                                                          //czyszczenie ekranu  
  light_off_time = millis() + (czas_podtrzymania_osw_lcd*1000);          //po X seundach od znikniecia logo gasimy swiatlo (znaczy ustawiamy czas do zgaśnięcia na za 5sec)
  //odpalam watchdoga
  wdt_enable(WDTO_8S);                      //reset after one second, if no "pat the dog" received  
}

//główna pętla programu
void loop(){
  //POMIAR NAPIECIA SOLARU I BATERII
  if(millis() >= mesure_time){
    pomiar_napiecia();
    mesure_time = millis() + czestotliwosc_pomiaru;
  }

  //HISTEREZA PRZEŁĄCZANIE TRYBÓW PRACY
  if(solar_v > accu_napiecie_max){ //nie obciążony panel tak ma być
    tryb_pracy = 1;    
  } 
  else if(solar_v < batt_v){ 
    tryb_pracy = 0;
  }
  
  // ładowanie zewnętrzne jeśli brak słońca lub napiecie akumulatora osiągneło poziom minimalny
  // wyłączenie ładowania po osiągnięciu poziomu maksymalnego lub gdy pojawi się słońce i rozpoczynam ładowanie ze słońca
  if(batt_v <= accu_napiecie_min){
      digitalWrite(ext_power, HIGH);   
  }elseif(tryb_pracy == 1 || batt_v >= accu_napiecie_max){
      digitalWrite(ext_power, LOW); 
  }

  //Uruchamiamy wyświetlacz przy zmianie trybu pracy 
  if(poprzedni_tryb_pracy != tryb_pracy){
    digitalWrite(podswietlenie, LOW);
    //multi_trend_saving(tryb_pracy);  //przy zmianie trybu zapis do logu
    //multi_trend_printing();          //przy zmianie pracy wywal zapisana historie tymczasowo tak
    light_off_time = millis() + (czas_podtrzymania_osw_lcd*1000);    
  }
  poprzedni_tryb_pracy = tryb_pracy; //if tryb pracy changed... brakuje mi tej komendy z GCL+

  //URUCHAMIANIE REGULACJI
  if(tryb_pracy >= 1){
    if(millis() >= exec_time){
      regulacja();
      exec_time = millis() + czestotliwosc_kaskady;
    }
  }
  else{
    analogWrite(mosfet,0);     
    pwm_duty_first = 0;
    pwm_duty_second = 0;
    pwm_duty = 0;      
  }

  //panel lcd
  if(millis() >= show_time){
    pokaz_dane();
    show_time = millis() + czestotliwosc_showtime;
  }

  //dane na port usb/rs do wykresów
  if(millis() >= plot_time && debug != 0){
    plot_data();
    plot_time = millis() + plot_interval;
  }    


  //jeśli poziom baterii niższy niż 11V lub coś z konfigu to odcinamy obciążenie...
  //ochrona akumulatora przed zniszczeniem
  if(batt_v <= accu_napiecie_odciecia){
    digitalWrite(load, HIGH);                   //disable load  
  }
  else if(batt_v >= accu_napiecie_wlaczenia){   
    digitalWrite(load, LOW);                    //enable load
  }
  if(batt_v >= 136){
    digitalWrite(podswietlenie, LOW);
    light_off_time = millis() + (czas_podtrzymania_osw_lcd*1000);  
  }
  else if(batt_v < 135){
    zgas_swiatlo();
  }
  //poprawka naprawiająca problem z liczeniem czasu (a właściewie problem z przepełnieniem licznika milisekund)
  //zapobiega zawieszaniu się urządzenia co 49dni
  //if(millis() >= 4294967295){
  //  exec_time = 0;
  //  show_time = 0;
  //  mesure_time = 0;
  //  light_off_time = 0;    
  //}
  
  // zauważyłem przywieszki po długim czasie działania
  // pomimo słońca sterownik nie ładuje program pracuje
  // zrobię reset gdy nastąpią takie sytuacje
  if(solar_v >= 200 && batt_v <= 119 && pwm_duty <= 1){
     //cos jest nie halo restartujemy maszyne
     asm volatile ("  jmp 0");
  }
  wdt_reset();                                  // give me another second to do stuff (pat the dog)
  //END OF THIS SHIT
}
