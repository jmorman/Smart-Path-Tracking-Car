//sensorpwmmerge3 main.c 3/08/2019 Joseph Morman
//Use ADCs to read data from three RPR220s
//AIN4 - PD3 - SS1
//AIN5 - PD2 - SS2
//AIN2 - PE1 - SS3
//PWM Module 0 Generator 0 Signal 0 (MOPWM0) - PB6
//PWM Module 0 Generator 0 Signal 1 (MOPWM1) - PB7

#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <math.h>

#define PORTF 0x20
#define PORTE 0x10
#define PORTD 0x08

#define USR_SW2 0x10
#define RED 0x02
#define BLUE 0x04
#define GREEN 0x08
#define USR_SW1 0x01

#define TIMER_0 0x01
#define T0_COUNT_DOWN 0x00000000

#define ADC_BUFFER_LENGTH 10
#define PWM_BUFFER_LENGTH 100

int adc1_val = 0;
long adc1_reads = 0;
int adc1_buffer[ADC_BUFFER_LENGTH];
int adc2_val = 0;
long adc2_reads = 0;
int adc2_buffer[ADC_BUFFER_LENGTH];
int adc3_val = 0;
long adc3_reads = 0;
int adc3_buffer[ADC_BUFFER_LENGTH];

long pwm_sets = 0;
int pwm_val = 0;
int pwm_buffer[PWM_BUFFER_LENGTH];

int stop_count = 0;
int continue_count = 0;
int stop_flag = 0;

int direction = 0;

int pwm_sets_since_line_found = 0;
int adc1_threshold = 0x66;
int adc2_threshold = 0x74;
int adc3_threshold = 0x80;

void enable_interrupts()
{ 
  //ADC0 SS1
  NVIC_EN0_R |= 0x00008000; //enable IRQ15
  NVIC_PRI3_R = 3 << 29; //set interrupt 15 to priority 3
  
  ADC0_ISC_R |= ADC_ISC_IN1; //clear interrupt flag
  ADC0_IM_R |= ADC_IM_MASK1; //SS1 Interrupt Mask*/
  
  //ADC0 SS2
  NVIC_EN0_R |= 0x00010000; //enable IRQ16
  NVIC_PRI4_R = 3 << 5; //set interrupt 16 to priority 3
  
  ADC0_ISC_R |= ADC_ISC_IN2; //clear interrupt flag
  ADC0_IM_R |= ADC_IM_MASK2; //SS2 Interrupt Mask*/
  
  //ADC0 SS3
  NVIC_EN0_R |= 0x00020000; //enable IRQ17
  NVIC_PRI4_R = 3 << 13; //set interrupt 17 to priority 3
  
  ADC0_ISC_R |= ADC_ISC_IN3; //clear interrupt flag
  ADC0_IM_R |= ADC_IM_MASK3; //SS3 Interrupt Mask*/
}

void GPIO_Init()
{  
  //PWM
  GPIO_PORTB_AFSEL_R |= 0xC0; //alternate function enable for 1100 0000 - PB7 and PB6
  GPIO_PORTB_PCTL_R &= ~0xFF000000;
  GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB7_M0PWM1 | GPIO_PCTL_PB6_M0PWM0;
  GPIO_PORTB_AMSEL_R &= ~0xC0; //disable analog
  GPIO_PORTB_DEN_R |= 0xC0; //enable digital
  
  //ADCs and LEDs
  SYSCTL_RCGCGPIO_R = PORTD | PORTE | PORTF; //Turn on GPIO Ports D and F by enabling clock
  
  //Port D and E has our ADCs _PD3, PD2, PE1 = 0000 1100 = 0x0C for D, 0000 0010 = 0x02 for E
  //p817
  GPIO_PORTD_AFSEL_R |= 0x0C; //enable alternate function
  GPIO_PORTD_DEN_R &= ~0x0C; //disable digital
  GPIO_PORTD_AMSEL_R |= 0x0C; //analog mode enable
  
  GPIO_PORTE_AFSEL_R |= 0x02; //enable alternate function
  GPIO_PORTE_DEN_R &= ~0x02; //disable digital
  GPIO_PORTE_AMSEL_R |= 0x02; //analog mode enable
  
  GPIO_PORTF_DIR_R = RED | GREEN | BLUE & ~USR_SW1 & ~USR_SW2; //set Port F as output
  GPIO_PORTF_DEN_R = RED | GREEN | BLUE | USR_SW1 | USR_SW2; //enable digital PORT F
  GPIO_PORTF_DATA_R = 0; //clear all Port F
  
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; //unlock GPIOCR register
  GPIO_PORTF_CR_R = USR_SW2; //enables committing new value to GPIOPUR
  GPIO_PORTF_PUR_R = USR_SW1 | USR_SW2; //enable pull up resistor that allows USR_SW2
  //to be used as input
}

void Timer0_Init()
{
  SYSCTL_RCGCTIMER_R |= TIMER_0; //enable Timer 0 bit in RCGCTIMER register
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; //disable Timer 0  in GPTMCTL register
  TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER; 
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD; //configure mode register to Periodic Mode
  TIMER0_TAMR_R |= T0_COUNT_DOWN; //count down mode
  
  TIMER0_TAILR_R = 0x000250;//Number of cycles before timer resets
  
  TIMER0_CTL_R |= TIMER_CTL_TAEN; //enable timer and start counting down
  TIMER0_CTL_R |= TIMER_CTL_TAOTE; //enable Timer A ADC trigger
}

void PLL_Init(void)
{
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;

  SYSCTL_RCC_R |= SYSCTL_RCC_BYPASS; //bypass PLL while initializing
  SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;

  SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;   
  SYSCTL_RCC_R |= SYSCTL_RCC_XTAL_16MHZ;
  
  SYSCTL_RCC_R &= ~SYSCTL_RCC_OSCSRC_M;
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M; //configure to use main oscillator source

  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWRDN; 
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
  
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400; 
  SYSCTL_RCC2_R &= ~0x1FC00000;
  SYSCTL_RCC2_R |= 0x13 <<22 ;  

  SYSCTL_RCC_R |= SYSCTL_RCC_USESYSDIV;
  
  while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0){};
  SYSCTL_RCC_R &= ~SYSCTL_RCC_BYPASS; //hecca
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
}

void PWM0_Set(int a_val, int b_val, int period){
  PWM0_0_CTL_R = 0;
  PWM0_0_GENA_R = (PWM_0_GENA_ACTCMPAD_ZERO|PWM_0_GENA_ACTCMPAU_ONE);
  PWM0_0_GENB_R = (PWM_0_GENB_ACTCMPBD_ZERO|PWM_0_GENB_ACTCMPBU_ONE);
  PWM0_0_LOAD_R = period;       
  PWM0_0_CMPA_R = a_val;      
  PWM0_0_CMPB_R = b_val;
  PWM0_0_CTL_R |= (PWM_0_CTL_MODE|PWM_0_CTL_ENABLE);
  PWM0_ENABLE_R |= (PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN);
}

void ADC_Init()
{
  SYSCTL_RCGCADC_R |= 0x01; //Enable and provide a clock to ADC module 0 in Run mode
  for(int i = 0; i < 1000; i++){}//delay to let things settle before sampling

  //Sample Sequencer Configuration
  ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1 & ~ADC_ACTSS_ASEN2 & ~ADC_ACTSS_ASEN3; //ADC SS1, SS2, SS3 Disable
  ADC0_EMUX_R = ADC_EMUX_EM3_TIMER | ADC_EMUX_EM2_TIMER | ADC_EMUX_EM1_TIMER; //set timer to be used as trigger select for SS3, SS2, SS1
  
  ADC0_SSMUX1_R |= (4<<28) | (4<<24) | (4<<20) | (4<<16) | (4<<12) | (4<<8) | (4<<4) | 4; //SS1 - AIN4 - PD3
  ADC0_SSMUX2_R |= (5<<28) | (5<<24) | (5<<20) | (5<<16) | (5<<12) | (5<<8) | (5<<4) | 5; //SS2 - AIN5 - PD2
  ADC0_SSMUX3_R |= (2<<28) | (2<<24) | (2<<20) | (2<<16) | (2<<12) | (2<<8) | (2<<4) | 2; //SS3 - AIN2 - PE1
  
  ADC0_SSCTL1_R |= ADC_SSCTL1_IE0 | ADC_SSCTL1_END0; //interrupt enable, end of sequence
  ADC0_SSCTL2_R |= ADC_SSCTL2_IE0 | ADC_SSCTL2_END0; //interrupt enable, end of sequence
  ADC0_SSCTL3_R |= ADC_SSCTL3_IE0 | ADC_SSCTL3_END0; //interrupt enable, end of sequence
  
  ADC0_ACTSS_R |= ADC_ACTSS_ASEN1 | ADC_ACTSS_ASEN2 | ADC_ACTSS_ASEN3; //ADC SS1, SS2, SS3 Enable
  ADC0_PSSI_R |= /*ADC_PSSI_GSYNC |*/ ADC_PSSI_SS3 | ADC_PSSI_SS2 | ADC_PSSI_SS1; //start sampling

}

int max(int val1, int val2){
  return (val1 > val2 ? val1 : val2);
}

void adc_to_pwm(int base, int diff, int period, int w0, int w1, float frac)
{
  if(stop_flag == 1)
  {
    PWM0_Set(period - 1, period - 1, period);
  }
  else
  {
    ADC0_IM_R &= ~ADC_IM_MASK1 & ~ADC_IM_MASK2 & ~ADC_IM_MASK3; //temp disable
    
    int corrected_adc1 = adc1_val;
    int corrected_adc2 = adc2_val;
    int corrected_adc3 = adc3_val;
    
    //check for stopping
    int threshold = 0x73;
    if(corrected_adc1 > threshold && corrected_adc2 > threshold && corrected_adc3 > threshold)
    {
      stop_count++;
      continue_count = 0;
      if(stop_count > 600)
      {
        stop_flag = 1;
        PWM0_Set(period - 1, period - 1, period);
      }
    }
    else
    {
      continue_count++;
      if(continue_count > 100)
      {
        continue_count = 0;
        stop_count = 0;
      }
    }
  
    int max_adc = max(corrected_adc1, corrected_adc2);
  
    int raw_gen_a_val = base+diff*(corrected_adc2-corrected_adc1)/max_adc;
  
    for(int i = PWM_BUFFER_LENGTH - 1; i > 0; i--){
      pwm_buffer[i] = pwm_buffer[i-1];
    }
    pwm_sets++;
    pwm_sets_since_line_found++;
  
    pwm_val = (w0*raw_gen_a_val+w1*pwm_buffer[1])/(w0+w1);
    pwm_buffer[0] = pwm_val;
    
    int gen_a_val = pwm_val;
    int gen_b_val = 2 * base - gen_a_val;
  
    //PWM0_Set(gen_a_val, gen_b_val, period);
  
    int dir_a_val = period/2+(period/2-1)*direction;
    int dir_b_val = period/2-(period/2-1)*direction;
    
    //GPIO_PORTF_DATA_R &= ~BLUE;
    PWM0_Set((int) round((1-frac) * gen_a_val + frac * dir_a_val), 
             (int) round((1-frac) * gen_b_val + frac * dir_b_val), 
              period);
  
    ADC0_IM_R |= ADC_IM_MASK1 | ADC_IM_MASK2 | ADC_IM_MASK3; //reenable
  }
}

void ADC0_SS1_Handler()
{
  adc1_val = ADC0_SSFIFO1_R;
  adc1_reads++;

  for(int i = ADC_BUFFER_LENGTH - 1; i > 0; i--){
    adc1_buffer[i] = adc1_buffer[i-1]; //(adc1_buffer[i] + adc1_buffer[i-1])/2;
  }
  adc1_buffer[0] = adc1_val;
  
  if((adc1_buffer[0] > adc1_threshold) &&(adc1_buffer[1] > adc1_threshold))//&& (adc3_buffer[2] > threshold)&& (adc3_buffer[3] > threshold))
  {
    direction = -1;
    GPIO_PORTF_DATA_R |= GREEN;
    pwm_sets_since_line_found = 0;
  }
  else
  {
    GPIO_PORTF_DATA_R &= ~GREEN;
  }
  
  ADC0_ISC_R |= ADC_ISC_IN1; //clear flag
}

void ADC0_SS2_Handler()
{
  adc2_val = ADC0_SSFIFO2_R;
  adc2_reads++;
  for(int i = ADC_BUFFER_LENGTH - 1; i > 0; i--){
    adc2_buffer[i] = adc2_buffer[i-1];//(adc2_buffer[i] + adc2_buffer[i-1])/2;
  }
  adc2_buffer[0] = adc2_val;
  
  //GPIO_PORTF_DATA_R ^= BLUE;
  
   if((adc2_buffer[0] > adc2_threshold) && (adc2_buffer[1] > adc2_threshold))//&& (adc2_buffer[2] > threshold)&& (adc2_buffer[3] > threshold))
  {
    direction = 1;
    pwm_sets_since_line_found = 0;
    GPIO_PORTF_DATA_R |= RED;    
  } 
  else
  {
    GPIO_PORTF_DATA_R &= ~RED;
  }
  
  ADC0_ISC_R |= ADC_ISC_IN2; //clear flag
}

void ADC0_SS3_Handler()
{
  adc3_val = ADC0_SSFIFO3_R;
  adc3_reads++;
  for(int i = ADC_BUFFER_LENGTH - 1; i > 0; i--){
    adc3_buffer[i] = adc3_buffer[i-1];//(adc3_buffer[i] + adc3_buffer[i-1])/2;
  }
  adc3_buffer[0] = adc3_val;
   if((adc3_buffer[0] > adc3_threshold) && (adc3_buffer[1] > adc3_threshold))//&& (adc3_buffer[2] > threshold)&& (adc3_buffer[3] > threshold))
  {
    pwm_sets_since_line_found = 0;
    //GPIO_PORTF_DATA_R |= BLUE;    
  } 
  else
  {
    //GPIO_PORTF_DATA_R &= ~BLUE;
  }
  
  ADC0_ISC_R |= ADC_ISC_IN3; //clear flag
}

 int main()
{
  SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB; //Turn on GPIO Port B by enabling its clock
  Timer0_Init();
  GPIO_Init();
  PLL_Init();
  int period = 2000;
  PWM0_Set(period/2, period/2,period);
  ADC_Init();
  enable_interrupts();
  
  while(1){
    if(stop_flag != 1)
    {
      adc_to_pwm(800,550,2000, 3, 1, 0.08);
    }
    else
    {
      PWM0_ENABLE_R &= ~PWM_ENABLE_PWM0EN & ~PWM_ENABLE_PWM1EN;
    }
  }
  return 0;
}
