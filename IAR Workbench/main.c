#include "TM4C123GH6PM.h"
#include <stdio.h>
#include <math.h>

float R_LDR;
float voltage;
float LUX = 0;
float B = 5845256999.367113;
float m = -2.011862635;




int main(void)
{
     unsigned int adc_value;
	  
    // The initialization sequence for the ADC is as follows: 
     
    // 1. Enable the ADC clock using the RCGCADC register
    SYSCTL->RCGCADC |= (1<<0);    // Enabling the clock for AD0 //
    
    // 2. Enable the clock to the appropriate GPIO modules via the RCGCGPIO register
    SYSCTL->RCGCGPIO |= (1<<4);  // Enable Clock to GPIOE //
    GPIOE->DIR &= ~(1<<1) ;      // Setting the Direction for the GPIO E as Input //
    
    // 3. Set the GPIO AFSEL bits for the ADC input pins
    GPIOE->AFSEL |= (1<<3);       // enable alternate function for the PE3 Pin //
    
    // 4. Configure the AINx signals to be analog inputs by clearing the corresponding DEN bit in the GPIO Digital Enable (GPIODEN) register
    GPIOE->DEN &= ~(1<<3);        // disable digital function for the PE3 Pin //
    
    // 5. Disable the analog isolation circuit for all ADC input pins that are to be used by writing a
    // "1" to the appropriate bits of the GPIOAMSEL register in the associated GPIO block.
    GPIOE->AMSEL |= (1<<3);       // enable analog function for the PE3 Pin  //
   
    // The configuration for each sample sequencer should be as follows: 
    
    // 1. Ensure that the sample sequencer is disabled by clearing the corresponding ASENn bit in the ADCACTSS register. 
    // Programming of the sample sequencers is allowed without having them enabled.
    // Disabling the sequencer during programming prevents erroneous execution if a trigger event were to occur during the configuration process. 
    ADC0->ACTSS &= ~(1<<3);        // disable SS3 during configuration //
    
    // 2. Configure the trigger event for the sample sequencer in the ADCEMUX register. 
    ADC0->EMUX &= ~0xF000;    // Continous Sampling on the SS3 //
    
    // 3. When using a PWM generator as the trigger source, use the ADC Trigger Source Select (ADCTSSEL) register to specify in which PWM module the generator
    // is located.
    // The default register reset selects PWM module 0 for all generators.
    // Not needed
    
    // 4. For each sample in the sample sequence, configure the corresponding input source in the ADCSSMUXn register. 
    ADC0->SSMUX3 = 0;         // get input from AIN0 //
    
    // 5. For each sample in the sample sequence, configure the sample control bits in the corresponding nibble in the ADCSSCTLn register.
    // When programming the last nibble,ensure that the END bit is set. Failure to set the END bit causes unpredictable behavior. 
    ADC0->SSCTL3 |= (1<<1)|(1<<2);        // take one sample at a time, set flag at 1st sample //
    
    // 6. If interrupts are to be used, set the corresponding MASK bit in the ADCIM register. 
    // Not needed
    
    // 7. Enable the sample sequencer logic by setting the corresponding ASENn bit in the ADCACTSS register. 
    ADC0->ACTSS |= (1<<3);         // enable ADC0 sequencer 3 //
    
	
    while(1)
    {
        ADC0->PSSI |= (1<<3);        // Enable SS3 conversion or start sampling data from AN0 //
        
        while((ADC0->RIS & 8) == 0) ;   // checking the 3rd bit of the RIS to see if the sampling is completed or not//
        
        adc_value = ADC0->SSFIFO3; // read adc coversion result from SS3 FIFO //
        
        ADC0->ISC = 8;          // clear coversion clear flag bit //
		
        voltage = (adc_value * 0.0008); // convert digital value back into voltage //
	                 
        R_LDR = ((3.3/voltage) - 1)*10000; // calculating the calue of resistence of LDR //
    
        LUX = B*pow(R_LDR,m) ; // Calculating the value of LUX //
      
    }

}