//******************************************************************************
//  ECE 3363 Unit 3, Example 6 - Scrolling Message
//
//  Description: This code displays the message "ECE-3363----" onto the display,
//  scrolling it from right to left at a rate of 0.5s per character. Looping when it reaches
//  the end.
//
//  ACLK = n/a, MCLK = SMCLK = default DCO
//   Dylan Tarter
//   Texas Instruments Inc.
//   April 2025
//   Built with Code Composer Studio V12.8.1
//******************************************************************************

#include <msp430.h>

// Macros for GPIO pins on the daugher board
#define DIG1    BIT0 // P3.0, Left-most digit ON/OFF
#define DIG2    BIT1 // P3.1, Second digit ON/OFF
#define DIG3    BIT2 // P3.2, Third digit ON/OFF
#define DIG4    BIT3 // P3.3, Right-most digit ON/OFF
#define DIGCOL  BIT7 // P3.7, Colon digit ON/OFF

#define SEGA    BIT0 // P2.0, Segment A ON/OFF
#define SEGB    BIT1 // P2.1, Segment B ON/OFF
#define SEGC    BIT2 // P2.2, Segment C ON/OFF
#define SEGD    BIT3 // P2.3, Segment D ON/OFF
#define SEGE    BIT4 // P2.4, Segment E ON/OFF
#define SEGF    BIT5 // P2.5, Segment F ON/OFF
#define SEGG    BIT6 // P2.6, Segment G ON/OFF
#define SEGDP   BIT7 // P2.7, Segment DP ON/OFF

#define BTN1    BIT5 // P1.5, Left-most push button
#define BTN2    BIT3 // P1.3, Middle push button
#define BTN3    BIT7 // P4.7, Right most push button

// Look up table for converting a code number to a segment pattern
const unsigned char seven_seg_lut[] = {
                                       SEGA+SEGB+SEGC+SEGD+SEGE+SEGF     ,// Display 0 = 0
                                       SEGA+          SEGD+SEGE+SEGF+SEGG,// Display E = 1
                                       SEGA+          SEGD+SEGE+SEGF     ,// Display C = 2
                                       SEGA+SEGB+SEGC+SEGD+          SEGG,// Display 3 = 3
                                                                     SEGG,// Display - = 4
                                       SEGA+SEGB+SEGC+SEGD+          SEGG,// Display 3 = 5
                                       SEGA+     SEGC+SEGD+SEGE+SEGF+SEGG,// Display 6 = 6
                                       0                                 ,// Display NULL
};

// ECE-3363---- message, interpreted as integers
const unsigned int message[] = {
                                1,
                                2,
                                1,
                                4,
                                3,
                                3,
                                6,
                                3,
                                4,
                                4,
                                4,
                                4,
};

// Current location in the message
unsigned int message_index = 0;

// What is displayed to the screen
unsigned char display[] = {7,7,7,7,7,7};

// Write a specific digit to the display
void write(unsigned char digit, unsigned char num) {
    P2OUT = seven_seg_lut[num]; // Load the pattern onto the segments

    P3OUT &=~ (DIG1+DIG2+DIG3+DIG4+DIGCOL); // Turn off all digits

    // Select the specific digit to power
    switch(digit) {
    case 1: P3OUT |= DIG1; break;
    case 2: P3OUT |= DIG2; break;
    case 3: P3OUT |= DIG3; break;
    case 4: P3OUT |= DIG4; break;
    case 5: P3OUT |= DIGCOL; break;
    case 6: P3OUT |= DIG1+DIG2+DIG3+DIG4+DIGCOL; break;
    }
}



int main(void) {
  WDTCTL = WDTPW | WDTHOLD;                 // Pause watchdog timer (ESSENTIAL)

  // Configure Segment pins
  P2DIR |= SEGA+SEGB+SEGC+SEGD+SEGE+SEGF+SEGG+SEGDP;
  P2OUT |= SEGA+SEGB+SEGC+SEGD+SEGE+SEGF+SEGG+SEGDP;

  // Configure digit pins
  P3DIR |= DIG1+DIG2+DIG3+DIG4+DIGCOL;
  P3OUT|= DIG1+DIG2+DIG3+DIG4+DIGCOL;

  PM5CTL0 &= ~LOCKLPM5;                     // Unlock GPIO Ports (ESSENTIAL)

  // Configure TimerA0
   TA0CTL |= TASSEL__SMCLK + MC__UP + ID__8; // Timer counts up, at 1,000,000MHz / 8
   TA0CCR0 = 62499;                          // 1000000/8 = 62500 - 1 = Period of 0.5s
   TA0CCTL0 |= CCIE;                         // Enables interrupts for channel 0 (CCR0)

  __enable_interrupt();                     // Enable interrupts
  __no_operation();                         // For debugger to accurately
                                            // place breakpoints around LPM transitions

  while(1) {                                // Just loop and do nothing forever
      int i;
      // Loop through each character on the display (1-6)
      for(i = 1; i < 7; i++) {
          write(i,display[i-1]);
          __delay_cycles(2083);
      }
  }
}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR           
__interrupt void Timer_ISR(void) {
     // Shift register for the display, moving each character over to the left by 1
     display[0] = display[1];
     display[1] = display[2];
     display[2] = display[3];
     // Loads the new character onto the screen at the far right
     display[3] = message[message_index++];

     // Loops the message when the index reaches the end
     if(message_index >= 12) {
         message_index = 0;
     }
}
