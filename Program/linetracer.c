#include <fungsi.h>
#include <mega16.h>
#include <stdio.h>
#include <delay.h>
#include <alcd.h>
//DEFINISI AWAL
#define ADC_VREF_TYPE 0x20
#define up PINB.5
#define down PINB.4
#define back PINB.1
#define next PINB.3
#define ok PINB.2
#define rem_kiri PORTD.3
#define rem_kanan PORTD.2
#define ditekan 0
#define tidak_ditekan 1
#define on 0
#define off 1
#define mundur 0
#define maju 1
#define kiri PORTD.6
#define kanan PORTD.7
#define speed_kiri OCR1A
#define speed_kanan OCR1B
//VARIABLE SETTING
signed long int n=0,i=0;
unsigned long int adc_max[10]={0,0,0,0,0,0,0,0,0,0};
unsigned long int adc_min[10]={250,250,250,250,250,250,250,250,250,250};
unsigned long int data_sensor[8];
unsigned long int data_rata_sensor[8];
unsigned long int hasil_baca[8];
unsigned long int hasil_track;
signed long int error_sensor;
unsigned char e[10];
unsigned int rata_error[8];
eeprom unsigned char kp=1;
eeprom unsigned char ki=1;
eeprom unsigned char kd=1;
eeprom unsigned char speed;
eeprom unsigned char ts;
signed int speedka;
signed int speedki;
long int P;
unsigned char tampil [33];
signed long int xsampling=0;
signed int d1,d2,d3,D;
signed long int i1,i2,i3,Y;
signed long int last_error=0;

// Timer2 overflow interrupt service routine
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
TCNT2=0xc2;
ts=xsampling;
xsampling=0;
}
// Timer 0 output compare interrupt service routine
interrupt [TIM0_COMP] void timer0_comp_isr(void)
{

}
unsigned char read_adc(unsigned char adc_input)
{
ADMUX=adc_input | (ADC_VREF_TYPE & 0xff);
delay_us(20);
ADCSRA|=0x40;
while ((ADCSRA & 0x10)==0);
ADCSRA|=0x10;
return ADCH;
}
