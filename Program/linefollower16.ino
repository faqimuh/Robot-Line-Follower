#include <avr/io.h>
#include <util/delay.h>
#include "U8glib.h"
#include <avr/eeprom.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0); // OLED SSD1306 dengan I2C

// Definisi tombol dengan bitwise operation
#define next (PIND & (1 << PD1))     // Tombol NEXT pada PD1
#define cancel (PINB & (1 << PB3))  // Tombol CANCEL pada PB3
#define ok (PINB & (1 << PB4))      // Tombol OK pada PB4

#define MUX_S0 PD4
#define MUX_S1 PD3
#define MUX_S2 PD2

// Fungsi untuk inisialisasi pin digital untuk kontrol MUX
void MUX_init() {
    DDRD |= (1 << MUX_S0) | (1 << MUX_S1) | (1 << MUX_S2);  // Atur S0, S1, S2 sebagai output
}
// Fungsi untuk inisialisasi ADC
void ADC_init() {
    ADMUX = (1 << REFS0) | (1 << ADLAR);  // Referensi tegangan VCC, hasil diatur ke kiri (8-bit)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Mengaktifkan ADC dengan prescaler 64
}

unsigned char x = 0;
unsigned char periode = 255;
unsigned char pwml = 0;
unsigned char pwmr = 0;

#define dirka_1 (PORTD |= (1 << PD6))
#define dirka_0 (PORTD &= ~(1 << PD6))
#define dirki_1 (PORTB |= (1 << PB1))
#define dirki_0 (PORTB &= ~(1 << PB1))

unsigned int front_sensor;
int adc_masuk[18],min_adc[18], max_adc[18], batas_adc[18],sensor[18],i,ka,ki,kp,kd,kaa,kii,kanann;
int ee_batas[16], ee_sensor[16];
int speed_ka,speed_ki,error_before,mv,mp=0,error,speed,delay_mundur,ms=0,sp=0,mt,r=0;
unsigned long int hasil_baca[16];

ISR(TIMER2_OVF_vect) {
    if (++x > periode) {
        x = 0;
        DDRB |= (1<<2);
        DDRD |= (1<<5);
    }
    // PWM untuk PB2
    if (x < pwmr) {
        PORTB |= (1 << 2);  // Aktifkan PB2 jika x < pwml
    } else {
        PORTB &= ~(1 << 2); // Nonaktifkan PB2
    }
    // PWM untuk PD5
    if (x < pwml) {
        PORTD |= (1 << 5);  // Aktifkan PD5 jika x < pwmr
    } else {
        PORTD &= ~(1 << 5); // Nonaktifkan PD5
    }
}

int main(void) {
    // Pengaturan Port
    PORTD |=_BV(PD1); // pin PD.1 input pullup  (NEXT)
    PORTB |=_BV(PB3); // pin PB3 input pullup   (CANCEL)
    PORTB |=_BV(PB4); // pin PB4 input pullup   (OK)

    DDRB |= _BV(PB1);//kontrol arah putar motor kanan
    DDRD |= _BV(PD6);//kontrol arah purah motor kiri

    MUX_init();       // Inisialisasi MUX
    //UART_init(9600);  // UART dengan baud rate 9600
    ADC_init();       // Inisialisasi ADC
    cli();
    // Konfigurasi Timer1 untuk PWM
    TCCR2A = 0x00; // Normal mode (tidak ada PWM atau mode CTC)
    TCCR2B = 0x01; // Prescaler 1 (sehingga timer beroperasi dengan clock 16 MHz)
    TCNT2 = 0;     // Reset nilai counter
    TIMSK2 |= (1 << TOIE2); // Aktifkan interrupt overflow
    sei(); // Aktifkan global interrupt
    //Inisialisasi OLED
    u8g.setFont(u8g_font_unifont); // Set font
    u8g.firstPage();
    do {
        u8g.setPrintPos(12, 14);
        u8g.print("LINE FOLLOWER");
        u8g.setPrintPos(43, 29);
        u8g.print("ROBOT");
        u8g.setPrintPos(55, 44);
        u8g.print("BY");
        u8g.setPrintPos(43, 59);
        u8g.print("PAQII");
    } while (u8g.nextPage()); 
    _delay_ms(200);
    main_menu();
    while (1) {}
}

//////////////////////////////////////MENU UTAMA ////////////////////////////////////////////////////
void main_menu() {
    unsigned char loop_menu = 0;
    while (ok) {
        if (!next) {
            _delay_ms(100);
            if (!next) while (!next);
            if (++loop_menu > 5) loop_menu = 0;
        }
        if (!cancel) {
            _delay_ms(100);
            if (!cancel) while (!cancel);
            if (--loop_menu == 255) loop_menu = 5;
        }
        u8g.firstPage();
        do {
            u8g.setFont(u8g_font_unifont);  // Atur font
            u8g.drawStr(10, 10, "Menu Setting:");
            switch (loop_menu) {

                case 0: u8g.drawStr(0, 30, "1. START"); break;
                case 1: u8g.drawStr(0, 30, "2. AUTO CAL"); break;
                case 2: u8g.drawStr(0, 30, "3. Parameter PID"); break;
                case 3: u8g.drawStr(0, 30, "4. Cek SENSOR"); break;
                case 4: u8g.drawStr(0, 30, "5. Cek ADC"); break;
                case 5: u8g.drawStr(0, 30, "6. TEST MOTOR"); break;
            }
        } while (u8g.nextPage());
    }
        if (loop_menu == 0) {
            start();
        } else if (loop_menu == 1) {
            kalibrasi();
        } else if (loop_menu == 2) {
            kontrol_pid();
        } else if (loop_menu == 3) {
            cek_sensor();
        }else if (loop_menu == 4) {
            cek_adc();
        }else if (loop_menu == 5) {
            tes_jalan();
        }
}//////////////////////////////////////END MENU UTAMA//////////////////////////////////////////////

///////////////////////////////////////////////////////////// Fungsi untuk mengatur saluran MUX//////////////
void MUX_selectChannel(uint8_t channel) {
    PORTD = (PORTD & 0xE3) | ((channel & 0x07) << 2);  // Masukkan 3-bit channel ke S0, S1, S2
}//////////////////////////////////////////////////////////////END PROGRAM/////////////////////////////////

////////////Fungsi untuk membaca nilai ADC dalam 8-bit////////////////////////////////////////////////
uint8_t read_adc(uint8_t adc_input) {
    ADMUX = (ADMUX & 0xF8) | (adc_input & 0x07);  // Pilih kanal ADC (PC3 = channel 3)
    ADCSRA |= (1 << ADSC);  // Memulai konversi
    while (ADCSRA & (1 << ADSC));  // Tunggu konversi selesai
    return ADCH;  // Kembalikan nilai 8-bit dari ADCH
}/////////////////////////////////END PROGRAM///////////////////////////////////////////////////////

////////////////END PROGRAM KALIBRASI////////////////////////////////////////////////////
void kalibrasi(){
  u8g.firstPage();
  do {
    u8g.setPrintPos(20, 29); 
    u8g.print("CALIBRATION");
    u8g.setPrintPos(0, 59); 
    u8g.print("1. OK");
  } while( u8g.nextPage() );
  for (int i=0;i<16;i++){
        min_adc[i]=0;
        max_adc[i]=255;
  }
  while(ok){
    test();
    for (int i=0;i<16;i++){
         if (adc_masuk[i]>min_adc[i]){min_adc[i]=adc_masuk[i];};
         if (adc_masuk[i]<max_adc[i]){max_adc[i]=adc_masuk[i];};
         batas_adc[i]=min_adc[i]+(max_adc[i]-min_adc[i])/2;
        // Simpan nilai ke EEPROM menggunakan ee_batas[i] sebagai alamat
        if (eeprom_read_byte((uint8_t *)&ee_sensor[i]) != (uint8_t)batas_adc[i]) {
            eeprom_write_byte((uint8_t *)&ee_sensor[i], (uint8_t)batas_adc[i]);
      }
    }
    }
    _delay_ms(500);
    while(ok){
      u8g.firstPage();
      test();
        do {
            u8g.setFont(u8g_font_unifont);
            u8g.setPrintPos(25, 29);
            u8g.print("CEK BIT CALL");
            u8g.setPrintPos(0, 48);
            for (int i = 0; i < 16; i++) {
              if(adc_masuk[i] > batas_adc[i]) {
                    front_sensor = front_sensor | (1 << (15 - i));
                    u8g.print("1");
                } else {
                    front_sensor = front_sensor | (0 << (15 - i));
                    u8g.print("0");
                };
            }

        } while (u8g.nextPage());
    }
    u8g.firstPage();
    do {
    u8g.setPrintPos(20, 29); 
    u8g.print("COMPLETE!!!");
  } while( u8g.nextPage() );
    _delay_ms(500);
    main_menu();
}////////////////END PROGRAM KALIBRASI//////////////////////////////////////////////////

////////////////////////////////////////PROGRAM CEK NILAI BIT////////////////////////////////////////////////////
void cek_sensor(){
  for (int i = 0; i < 16; i++) {
        batas_adc[i] = eeprom_read_byte((uint8_t *)&ee_sensor[i]);
    }
  u8g.firstPage();
  do {
    u8g.setPrintPos(20, 29); 
    u8g.print("CEK BIT");
    u8g.setPrintPos(0, 59); 
    u8g.print("1. OK");
  } while( u8g.nextPage() );
  while (ok) {
    test();
    u8g.firstPage();
        do {
            u8g.setFont(u8g_font_unifont);
            u8g.setPrintPos(25, 29);
            u8g.print("CEK BIT");
            u8g.setPrintPos(0, 48);
            for (int i = 0; i < 16; i++) {
              if(adc_masuk[i] > batas_adc[i]) {
                    front_sensor = front_sensor | (0<< (15 - i));
                    u8g.print("0");
                } else {
                    front_sensor = front_sensor | (1 << (15 - i));
                    u8g.print("1");
                }
            }
        } while (u8g.nextPage());
    }
    u8g.firstPage();
    do {
    u8g.setPrintPos(10, 29); 
    u8g.print("BACK TO MENU");
  } while( u8g.nextPage() );
    _delay_ms(500);
    main_menu();
}//////////////////////////////////////END PROGRAM CEK NILAI BIT////////////////////////////////////////////////

//////////////////////////////////////PROGRAM GANTI NILAI PARAMETER////////////////////////////////////////////////
void kontrol_pid() {
    unsigned char loop_pid = 0;
    kp = eeprom_read_byte((uint8_t *)0);
    kd = eeprom_read_byte((uint8_t *)1);
    speed = eeprom_read_byte((uint8_t *)2);
    u8g.firstPage();
    do {
      u8g.setPrintPos(20, 10);
      u8g.print("PD Setting");
    } while( u8g.nextPage() );
      _delay_ms(500);

    while (ok) {
    if (!ok) while (!ok){}

        if (!next) {
            _delay_ms(50);
            if (!next) while (!next);
            if (++loop_pid > 3) loop_pid = 0;
        }

        // Tampilkan menu PID
        u8g.firstPage();
        do { 
            u8g.setFont(u8g_font_unifont);
            u8g.drawStr(20, 10, "PID Setting:");
            switch (loop_pid) {
                case 0:
                    u8g.setPrintPos(10,30);
                    u8g.print("1. KP = ");
                    u8g.print(kp);
                    break;
                case 1:
                    u8g.setPrintPos(10,30);
                    u8g.print("2. KD = ");
                    u8g.print(kd);
                    break;
                case 2:
                    u8g.setPrintPos(10,30);
                    u8g.print("3. Speed = ");
                    u8g.print(speed);
                    break;
                case 3:
                    u8g.setPrintPos(10,30);
                    u8g.print("4. Save");
                    break;
            }
        } while (u8g.nextPage());
    }
        _delay_ms(500);

        while (ok) {
            if (!ok) while (!ok){}

            if (loop_pid == 0) { // KP
                if (!next) {
                    _delay_ms(20);
                    if (!next) while (!next);
                    if (++kp > 100) kp = 0;
                    print_kp();
                } else if (!cancel) {
                    _delay_ms(20);
                    if (!cancel) while (!cancel);
                    if (--kp == 255) kp = 100;
                    print_kp();
                }

            } else if (loop_pid == 1) { // KD
                if (!next) {
                    _delay_ms(20);
                    if (!next) while (!next);
                    if (++kd > 100) kd = 0;
                    print_kd();
                } else if (!cancel) {
                    _delay_ms(20);
                    if (!cancel) while (!cancel);
                    if (--kd == 255) kd = 100;
                    print_kd();
                }
            } else if (loop_pid == 2) { // KD
                if (!next) {
                    _delay_ms(20);
                    if (!next) while (!next);
                    if (++speed > 100) speed = 0;
                    print_speed();
                } else if (!cancel) {
                    _delay_ms(20);
                    if (!cancel) while (!cancel);
                    if (--speed == 255) speed = 255;
                    print_speed();
                }

            } else if (loop_pid == 3) { // Save
                eeprom_write_byte((uint8_t *)0, kp);
                eeprom_write_byte((uint8_t *)1, kd);
                eeprom_write_byte((uint8_t *)2, speed);
                    u8g.firstPage();
                do {
                    u8g.setFont(u8g_font_6x10);
                    u8g.drawStr(10, 15, "Parameter saved!");
                    u8g.setPrintPos(10,25);
                    u8g.print(F("1. KP = "));
                    u8g.print(kp);
                    u8g.setPrintPos(10,35);
                    u8g.print(F("2. KD = "));
                    u8g.print(kd);
                    u8g.setPrintPos(10,45);
                    u8g.print(F("3. Speed = "));
                    u8g.print(speed);
                } while (u8g.nextPage());
                _delay_ms(2000);
                main_menu();
            }
        }
                eeprom_write_byte((uint8_t *)0, kp);
                eeprom_write_byte((uint8_t *)1, kd);
                eeprom_write_byte((uint8_t *)2, speed);
                u8g.firstPage();
                do {
                    u8g.setFont(u8g_font_6x10);
                    u8g.drawStr(10, 15, "Parameter saved!");
                    u8g.setPrintPos(10,25);
                    u8g.print(F("1. KP = "));
                    u8g.print(kp);
                    u8g.setPrintPos(10,35);
                    u8g.print(F("2. KD = "));
                    u8g.print(kd);
                    u8g.setPrintPos(10,45);
                    u8g.print(F("3. Speed = "));
                    u8g.print(speed);
                } while (u8g.nextPage());

                _delay_ms(2000);
                main_menu();    
}
void print_kp(){
      u8g.firstPage();
      do {
          u8g.setPrintPos(15,30);
                    u8g.print("KP = ");
                    u8g.print(kp);
      } while (u8g.nextPage());
}
void print_speed(){
      u8g.firstPage();
      do {
          u8g.setPrintPos(15,30);
                    u8g.print("Speed = ");
                    u8g.print(speed);
      } while (u8g.nextPage());
}

void print_kd(){
      u8g.firstPage();
      do {
          u8g.setPrintPos(25,30);
                    u8g.print("KD = ");
                    u8g.print(kd);
      } while (u8g.nextPage());
}//////////////////////////////////////END PROGRAM GANTI NILAI PARAMETER///////////////////////////////////////


//////////////////////////////////////PROGRAM CEK NILAI ADC SENSOR/////////////////////////////////////////////////
void cek_adc(){
  u8g.firstPage();
  do {
    u8g.setPrintPos(20, 29);
    u8g.print("CEK ADC");
  } while( u8g.nextPage() );
    _delay_ms(500);
  while (ok) {
    test();
    u8g.firstPage();
        do {
            u8g.setFont(u8g_font_unifont);
            u8g.setPrintPos(0,10);
            u8g.print(adc_masuk[0]);
            u8g.print(" ");
            u8g.print(adc_masuk[1]);
            u8g.print(" ");
            u8g.print(adc_masuk[2]);
            u8g.print(" ");
            u8g.print(adc_masuk[3]);

            u8g.setPrintPos(0,25);
            u8g.print(adc_masuk[4]);
            u8g.print(" ");
            u8g.print(adc_masuk[5]);
            u8g.print(" ");
            u8g.print(adc_masuk[6]);
            u8g.print(" ");
            u8g.print(adc_masuk[7]);

            u8g.setPrintPos(0,40);
            u8g.print(adc_masuk[8]);
            u8g.print(" ");
            u8g.print(adc_masuk[9]);
            u8g.print(" ");
            u8g.print(adc_masuk[10]);
            u8g.print(" ");
            u8g.print(adc_masuk[11]);

             u8g.setPrintPos(0,55);
            u8g.print(adc_masuk[12]);
            u8g.print(" ");
            u8g.print(adc_masuk[13]);
            u8g.print(" ");
            u8g.print(adc_masuk[14]);
            u8g.print(" ");
            u8g.print(adc_masuk[15]);
        } while (u8g.nextPage());
    }
    u8g.firstPage();
    do {
    u8g.setPrintPos(10, 29); 
    u8g.print("BACK TO MENU");
    }
    while( u8g.nextPage() );
    _delay_ms(500);
    main_menu();
}//////////////////////////////////////END PROGRAM CEK NILAI ADC//////////////////////////////////////////////////

//////////////////////////////////////CEK KONDISI JALAN MOTOR//////////////////////////////////////////////////
void cek_motor(){
  u8g.firstPage();
  do {
    u8g.setPrintPos(20, 29); 
    u8g.print("CEK MOTOR");
    u8g.setPrintPos(0, 59); 
    u8g.print("OK");
  } while( u8g.nextPage() );
   while(ok){
  pwml=pwmr=50;
  _delay_ms(5000);

  speed_ki=speed_ka=-50;
  PWM_out();
  _delay_ms(5000);

  speed_ki=20;
  speed_ka=-50;
  PWM_out();
  _delay_ms(5000);

  speed_ki=-50;
  speed_ka=20;
  PWM_out();
  _delay_ms(5000);

  speed_ki=speed_ka=0;
  PWM_out();
  break;
 }
 u8g.firstPage();
    do {
    u8g.setPrintPos(10, 29); 
    u8g.print("BACK TO MENU");
    }
    while( u8g.nextPage() );
    _delay_ms(500);
 main_menu();
}//////////////////////////////////////END PROGRAM CEK MOTOR//////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////PROGRAM UTAMA UNTUK KONTROL MOTOR////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////PROGRAM PEMBACAAN NILAI ADC SENSOR////////////////////////////////////////////////////
void test(){
            MUX_selectChannel(0);adc_masuk[7] = read_adc(3);        
            MUX_selectChannel(1);adc_masuk[3] = read_adc(3);
            MUX_selectChannel(2);adc_masuk[5] = read_adc(3);
            MUX_selectChannel(3);adc_masuk[1] = read_adc(3);
            MUX_selectChannel(4);adc_masuk[6] = read_adc(3);
            MUX_selectChannel(5);adc_masuk[2] = read_adc(3);
            MUX_selectChannel(6);adc_masuk[4] = read_adc(3);
            MUX_selectChannel(7);adc_masuk[0] = read_adc(3);

            MUX_selectChannel(8);adc_masuk[15] = read_adc(2);
            MUX_selectChannel(9);adc_masuk[11] = read_adc(2);
            MUX_selectChannel(10);adc_masuk[13] = read_adc(2);
            MUX_selectChannel(11);adc_masuk[9] = read_adc(2);
            MUX_selectChannel(12);adc_masuk[14] = read_adc(2);
            MUX_selectChannel(13);adc_masuk[10] = read_adc(2);
            MUX_selectChannel(14);adc_masuk[12] = read_adc(2);
            MUX_selectChannel(15);adc_masuk[8] = read_adc(2);
}////////////////END PROGRAM /////////////////////////////////////////////////////////////

//////////////////////////////////////////PROGRAM KONVERSI ADC SENSOR KE BIT///////////////
void variabel_sensor(){
    for (int i = 0; i < 16; i++) {
        ee_sensor[i] = eeprom_read_byte((uint8_t *)&ee_sensor[i]);
    }
        test();
        front_sensor=0;
        for(i=0;i<16;i++){
            if(adc_masuk[i]>ee_sensor[i]){
                front_sensor = front_sensor | (0 << (15 - i));
            }else {
                front_sensor = front_sensor | (1 << (15 - i));
                }
        }

        for(i=0;i<16;i++){
            if(adc_masuk[i]>ee_sensor[i]){
                sensor[i]=0;
            }
            else {sensor[i]=1;
            }
        }
        if(sensor[4]==0){kanann=0;}
        if(sensor[11]==0){kanann=1;}

}///////////////////////////////////////////////END PROGRAM///////////////////////////////

////////////////////////////////////////////KALKULASI NILAI EROR/////////////////////////////////////////
void track(){
    variabel_sensor();//untuk mendapatkan nilai data bin sensor dalam variabel front_sensor
        //if     (front_sensor==0b0111111111111111)error=7+sp;
        // else if(front_sensor==0b0011111111111111)error=10+sp;
        if(front_sensor==0b0001111111111111)error=7+sp;
        // else if(front_sensor==0b1001111111111111)error=9+sp;
        else if(front_sensor==0b0000111111111111)error=6+sp;
        // else if(front_sensor==0b1000111111111111)error=8+sp;
        // else if(front_sensor==0b1100111111111111)error=8+sp;
        else if(front_sensor==0b1000011111111111)error=5+sp;
        // else if(front_sensor==0b1100011111111111)error=6+sp;
        // else if(front_sensor==0b1110011111111111)error=6+sp;
        else if(front_sensor==0b1100001111111111)error=4+sp;
        // else if(front_sensor==0b1110001111111111)error=5+sp;
        // else if(front_sensor==0b1111001111111111)error=5+sp;
        else if(front_sensor==0b1110000111111111)error=3+sp;
        // else if(front_sensor==0b1111000111111111)error=4+sp;
        // else if(front_sensor==0b1111100111111111)error=4+sp;
        else if(front_sensor==0b1111000011111111)error=2+sp;
        // else if(front_sensor==0b1111100011111111)error=1+sp;
        // else if(front_sensor==0b1111110011111111)error=1+sp;
        else if(front_sensor==0b1111100001111111)error=1+sp;
        else if(front_sensor==0b1111110001111111)error=0+sp;
        else if(front_sensor==0b1111111001111111)error=0+sp;//
        else if(front_sensor==0b1111110000111111)error=0+sp;
        else if(front_sensor==0b1111111000111111)error=0+sp;
        else if(front_sensor==0b1111111000011111)error=-1+sp;
        // else if(front_sensor==0b1111111100111111)error=-1+sp;
        // else if(front_sensor==0b1111111100011111)error=-1+sp;
        else if(front_sensor==0b1111111100001111)error=-2+sp;
        // else if(front_sensor==0b1111111110011111)error=-4+sp;
        // else if(front_sensor==0b1111111110001111)error=-4+sp;
        else if(front_sensor==0b1111111110000111)error=-3+sp;
        // else if(front_sensor==0b1111111111001111)error=-5+sp;
        // else if(front_sensor==0b1111111111000111)error=-5+sp;
        else if(front_sensor==0b1111111111000011)error=-4+sp;
        // else if(front_sensor==0b1111111111100111)error=-6+sp;
        // else if(front_sensor==0b1111111111100011)error=-6+sp;
        else if(front_sensor==0b1111111111100001)error=-5+sp;
        // else if(front_sensor==0b1111111111110011)error=-8+sp;
        // else if(front_sensor==0b1111111111110001)error=-8+sp;
        else if(front_sensor==0b1111111111110000)error=-6+sp;
        //else if(front_sensor==0b1111111111111001)error=-9+sp;
        else if(front_sensor==0b1111111111111000)error=-7+sp;
        // else if(front_sensor==0b1111111111111100)error=-10+sp;
        // else if(front_sensor==0b1111111111111110)error=-10+sp;
        else if(front_sensor==0b1111111111111111){error=15;
                  // if(mp==0){
                  //   if(kanann==1){error=15;}
                  //   else {error=-15;}
                  // }
                  // if(mp==2){
                  //   if(error_before>-15 && error_before<0)
                  //    {error=error_before+2;}
                  //   else if(error_before<15 && error_before>0)
                  //    {error=error_before-2;}
                  //   else {error=error_before;
                  //   }
                  // }
             }
        else {error=error_before;}
}//////////////////////////////END PROGRAM KALKULASI NILAI EROR/////////////////////////////////////////

//////////////////////////////////////////PROGRAM KONTROL PD////////////////////////////////////////////
void PID(){
 mv=(kp*error)+(kd*(error-error_before));
 speed_ka=speed+mv;
 speed_ki=speed-mv;
 error_before=error;
 if(speed_ka>200) {speed_ka=200;};
 if(speed_ki>200) {speed_ki=200;};
 if(speed_ka<-200){speed_ka=-200;};
 if(speed_ki<-200){speed_ki=-200;};
}//////////////////////////////////////////END PROGRAM PD///////////////

/////////////////PROGRAM OUT MOTOR/////////////////////////////////////////////////////////
void PWM_out(){
      if(speed_ka>=0){
        pwmr=speed_ka;
        PORTB &= ~(1 << 1);                    //maju
     }
     else{
        pwmr=speed_ka;
        PORTB |= (1 << 1);                                         //mundur
     }

     if(speed_ki>=0){
        pwml=speed_ki;
        PORTD &= ~(1 << 6);
     }
      else{
        pwml=speed_ki;
        PORTD |= (1 << 6);
     }
}///////////////END PROGRAM//////////////////////////////////////////////////////////////

//////////////////////BELOK KANAN///////////////////////////////////////////////////////////
void belokR(long del){
    while (sensor[10] != 0) {
      speed_ka=-50;
      speed_ki=150;
      PWM_out();
      variabel_sensor();
    }
    _delay_ms(del);
    speed_ka = speed_ki = 0;
    PWM_out();
}//////////////////END BELOK KANAN/////////////////////////////////////////////////////////

//////////////////////BELOK KIRI///////////////////////////////////////////////////////////
void belokL(long del){
    while (sensor[5] != 0) {
        speed_ka = 200;
        speed_ki = -100;
        PWM_out();
        variabel_sensor();
    }
    _delay_ms(del);
    speed_ka = speed_ki = 0;
    PWM_out();
}//////////////////END BELOK KIRI////////////////////////////////////////////////////////

//////////////////////////////////PROGRAM JALAN////////////////////////////////////////////////////////
void jalan(){
    variabel_sensor();
    track();
    PID();
    PWM_out();
}//////////////////////////////////END JALAN////////////////////////////////////////////////////////

//////////////////////////////////PROGRAM GERAK FREEDOM//////////////////////////
void gerak(int ki, int ka,long del){
    while(del>0){
        speed_ka=ka;
        speed_ki=ki;
        PWM_out();
        _delay_ms(1);
        del--;
    }
}//////////////////////////////////END PROGRAM FREEDOM//////////////////////////

//////////////////////////////////PROGRAM BERHENTI DENGAN JEDA //////////////////////////
void pause(){
    gerak(0,0,2000);
}//////////////////////////////////END PROGRAM BERHENTI DENGAN JEDA //////////////////////////

//////////////////////////////////MAJU BERDASARKAN SENSOR //////////////////////////
void maju_sensor(unsigned int numer){
    if(numer==0){
      variabel_sensor();
      while(sensor[0]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==1){
      variabel_sensor();
      while(sensor[1]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==2){
      variabel_sensor();
      while(sensor[2]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==3){
      variabel_sensor();
      while(sensor[3]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==4){
      variabel_sensor();
      while(sensor[4]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==5){
      variabel_sensor();
      while(sensor[5]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==15){
      variabel_sensor();
      while(sensor[15]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==14){
      variabel_sensor();
      while(sensor[14]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==13){
      variabel_sensor();
      while(sensor[13]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==12){
        variabel_sensor();
        while(sensor[12]!=0){
            variabel_sensor();  
            jalan();
        }
    }

    if(numer==11){
      variabel_sensor();
      while(sensor[11]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==10){
      variabel_sensor();
      while(sensor[10]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==150){
      variabel_sensor();
      while(sensor[0]!=0 || sensor[15]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==141){
      variabel_sensor();
      while(sensor[1]!=0 || sensor[14]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==132){
      variabel_sensor();
      while(sensor[2]!=0 || sensor[13]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==123){
      variabel_sensor();
      while(sensor[3]!=0 || sensor[12]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==114){
      variabel_sensor();
      while(sensor[4]!=0 || sensor[11]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==105){
      variabel_sensor();
      while(sensor[5]!=0 || sensor[10]!=0){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==99){
      variabel_sensor();
      if(ms==0 || ms==1){
        while(front_sensor!=0b1111111111111111){
          variabel_sensor();
          jalan();
        }
      }
    
    if(ms==2){
      while(front_sensor!=0b1111111111){
        variabel_sensor();
        jalan();
      }
    }

    }
    if(numer==98){
      variabel_sensor();
      while(front_sensor!=0b1110000000000111){
        variabel_sensor();
        jalan();
      }
    }

    if(numer==97) {
        variabel_sensor();
        while(front_sensor!=0b0000000000000000){
            variabel_sensor();//speed_ka=speed_ki=speed;PWM_out();
            jalan();
        }
    }
    // if(r==1){
    //     speed_ka=speed_ki=speed*(-1);
    //     PWM_out();
    //     delay_mundur=(speed/3);
    //     while (delay_mundur--) {
    //     _delay_ms(1);  // Delay 1 ms per iterasi
    //     }
    //     speed_ka=speed_ki=0;
    //     PWM_out();
    // }
}//////////////////////////////////END PROGRAM ///////////////////////////////////

//////////////////////////////////MAJU BERDASARKAN DELAY //////////////////////////
void maju_hitung(long delai){
  while(delai!=0){
    jalan();
    _delay_ms(1);
    delai--;
  }
}////////////////////////////END MAJU BERDASARKAN DELAY //////////////////////////

void tes_jalan(void){
    kp = eeprom_read_byte((uint8_t *)0);
    kd = eeprom_read_byte((uint8_t *)1);
    speed = eeprom_read_byte((uint8_t *)2);
    
  mp=sp=mt=0;ms=0;
  while(1){
 jalan();
  }
  pause();
  main_menu();
}
void start(){
  kp = eeprom_read_byte((uint8_t *)0);
    kd = eeprom_read_byte((uint8_t *)1);
    speed = eeprom_read_byte((uint8_t *)2);
    mp=sp=mt=0;ms=0;
  gerak(0,0,1000);
  maju_sensor(3);
  gerak(100,15,400);
  maju_hitung(200);
  gerak(160,0,200);
  maju_hitung(300);//lurusan 1

  gerak(0,160,280);
  maju_hitung(180);
  gerak(0,140,200);

  maju_hitung(150);//lurrusan ke 2
  gerak(150,15,175);
  maju_hitung(180);
  gerak(150,15,270);
  maju_hitung(280);//lurusan ke 3
  gerak(15,150,330);
  maju_hitung(220);
  gerak(0,150,180);
  gerak(0,0,20);
  gerak(150,150,100);
  gerak(0,0,1000);
  gerak(150,-150,260);
  // jalur2();
  gerak(0,0,500);
  //maju_hitung(100);
  gerak(100,100,100);
  gerak(15,100,400);
  maju_hitung(200);
  gerak(0,160,200);
  maju_hitung(300);//lurusan 1

  gerak(160,0,280);
  maju_hitung(180);
  gerak(140,0,200);
  maju_hitung(150);//lurrusan ke 2

  gerak(15,150,175);
  maju_hitung(180);
  gerak(15,150,270);
  maju_hitung(300);//lurusan ke 3

  gerak(150,15,330);
  maju_hitung(200);
  gerak(150,0,190);
  gerak(0,0,30);
  maju_hitung(100);
  pause();
  main_menu();  
}