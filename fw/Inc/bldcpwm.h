/* =================================================================================
File name:        BLDCPWM.H                  
                    
Description: 
Hardware-dependent PWM macro with BLDC motor commutation logic
for STM32G474 (HRTIM1)

Config (#define PWM_METHOD):
Switching mode 1: terminal 1 - disable
                  terminal 2 - PWM
                  terminal 3 - PWM

Switching mode 2: terminal 1 - disable
                  terminal 2 - PWM
                  terminal 3 - force low side
=====================================================================================
 
------------------------------------------------------------------------------*/
#ifndef __BLDCPWM_H__
#define __BLDCPWM_H__
//------------------------------------------------
// Configure driver in this section:
#define PWM_METHOD 1

#define AH (LL_HRTIM_OUTPUT_TA1)
#define AL (LL_HRTIM_OUTPUT_TA2)
#define BH (LL_HRTIM_OUTPUT_TC1)
#define BL (LL_HRTIM_OUTPUT_TC2)
#define CH (LL_HRTIM_OUTPUT_TD1)
#define CL (LL_HRTIM_OUTPUT_TD2)

#if (PWM_METHOD == 1)
static inline void bldcpwm_update(uint32_t halfPwmPeriod, uint32_t duty, uint32_t sector)
{                                                                            
switch (sector)                                                                                   
    {                                                                                               
    case 0:                                                                                         
      /* State s1: current flows to motor windings from phase A->B, de-energized phase = C */       
      LL_HRTIM_DisableOutput(HRTIM1, CH | CL);                                                   
      LL_HRTIM_EnableOutput(HRTIM1, AH | AL | BH | BL);                                       
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, halfPwmPeriod + duty);                                   
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, halfPwmPeriod - duty);                                                                         
      break;                                                                                        
    case 1:                                                                                         
      /* State s2: current flows to motor windings from phase A->C, de-energized phase = B */       
      LL_HRTIM_DisableOutput(HRTIM1, BH | BL);                                                   
      LL_HRTIM_EnableOutput(HRTIM1, AH | AL | CH | CL);                                       
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, halfPwmPeriod + duty);                                                                     
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, halfPwmPeriod - duty);                                        
      break;                                                                                        
    case 2:                                                                                         
      /* State s3: current flows to motor windings from phase B->C, de-energized phase = A */       
      LL_HRTIM_DisableOutput(HRTIM1, AH | AL);                                                   
      LL_HRTIM_EnableOutput(HRTIM1, BH | BL | CH | CL);                                                                          
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, halfPwmPeriod + duty);                                   
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, halfPwmPeriod - duty);                                        
      break;                                                                                        
    case 3:                                                                                         
      /* State s4: current flows to motor windings from phase B->A, de-energized phase = C */       
      LL_HRTIM_DisableOutput(HRTIM1, CH | CL);                                                   
      LL_HRTIM_EnableOutput(HRTIM1, BH | BL | AH | AL);                                       
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, halfPwmPeriod - duty);                                        
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, halfPwmPeriod + duty);                                                                      
      break;                                                                                        
    case 4:                                                                                         
      /* State s5: current flows to motor windings from phase C->A, de-energized phase = B */       
      LL_HRTIM_DisableOutput(HRTIM1, BH | BL);                                                   
      LL_HRTIM_EnableOutput(HRTIM1, CH | CL | AH | AL);                                       
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, halfPwmPeriod - duty);                                                                    
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, halfPwmPeriod + duty);                                   
      break;                                                                                        
    case 5:                                                                                         
      /* State s6: current flows to motor windings from phase C->B, de-energized phase = A */       
      LL_HRTIM_DisableOutput(HRTIM1, AH | AL);                                                   
      LL_HRTIM_EnableOutput(HRTIM1, CH | CL | BH | BL);                                                                         
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, halfPwmPeriod - duty);                                        
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, halfPwmPeriod + duty);                                   
      break;                                                                                        
    default:                                                                                        
      break;                                                                                        
    }
}
#endif
#if (PWM_METHOD == 2)
static inline void bldcpwm_update(uint32_t halfPwmPeriod, uint32_t duty, uint32_t sector)
{                                                                            
switch (sector)                                                                                   
    {                                                                                               
    case 0:                                                                                         
      /* State s1: current flows to motor windings from phase A->B, de-energized phase = C */ 
      LL_HRTIM_DisableOutput(HRTIM1, CH | CL);                                                   
      LL_HRTIM_EnableOutput(HRTIM1, AH | AL | BL | BH);                                       
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, duty);
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, 0);                                   
      LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_C, 0);                                                                         
      break;                                                                                        
    case 1:                                                                                         
      /* State s2: current flows to motor windings from phase A->C, de-energized phase = B */       
      LL_HRTIM_DisableOutput(HRTIM1, BH | BL );                                                   
      LL_HRTIM_EnableOutput(HRTIM1, AH | AL | CL | CH);                                       
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, duty);                                                                     
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, 0);
      LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_D, 0);                                        
      break;                                                                                        
    case 2:                                                                                         
      /* State s3: current flows to motor windings from phase B->C, de-energized phase = A */       
      LL_HRTIM_DisableOutput(HRTIM1, AH | AL );                                                   
      LL_HRTIM_EnableOutput(HRTIM1, BH | BL | CL | CH);                                                                          
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, duty);                                   
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, 0);
      LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_D, 0);                                    
      break;                                                                                        
    case 3:                                                                                         
      /* State s4: current flows to motor windings from phase B->A, de-energized phase = C */       
      LL_HRTIM_DisableOutput(HRTIM1, CH | CL );                                                   
      LL_HRTIM_EnableOutput(HRTIM1, BH | BL | AL | AH);                                       
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, 0);
      LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_A, 0);                                        
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, duty);                                                                      
      break;                                                                                        
    case 4:                                                                                         
      /* State s5: current flows to motor windings from phase C->A, de-energized phase = B */       
      LL_HRTIM_DisableOutput(HRTIM1, BH | BL );                                                   
      LL_HRTIM_EnableOutput(HRTIM1, CH | CL | AL | AH);                                       
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, 0);
      LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_A, 0);                                                                    
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, duty);                                   
      break;                                                                                        
    case 5:                                                                                         
      /* State s6: current flows to motor windings from phase C->B, de-energized phase = A */       
      LL_HRTIM_DisableOutput(HRTIM1, AH | AL);                                                   
      LL_HRTIM_EnableOutput(HRTIM1, CH | CL | BL | BH);                                                                         
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, 0); 
      LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_C, 0);                                       
      LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_D, duty);                                   
      break;                                                                                        
    default:                                                                                        
      break;                                                                                        
    }
}
#endif

#endif // __BLDCPWM_H__
