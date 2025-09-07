//MSP430 code
#include <msp430.h>
#include <stdint.h>

/* ====== Pin Definitions ====== */
// Motor Pins
#define IN1_PIN    BIT2    // P2.2 - Left motor forward
#define IN2_PIN    BIT7    // P4.7 - Left motor backward
#define IN3_PIN    BIT4    // P2.4 - Right motor forward
#define IN4_PIN    BIT6    // P1.6 - Right motor backward

// IR sensors (active-low when over the line)
#define IR_LEFT_PIN    BIT0  // P2.0
#define IR_CTR_PIN     BIT4  // P1.4
#define IR_RGT_PIN     BIT3  // P9.3

// Ultrasonic sensor
#define TRIG_PIN      BIT7    // P1.7
#define ECHO_PIN      BIT5    // P2.5

// LCD Display
#define DIG1 1
#define DIG2 2
#define DIG3 3
#define DIG4 4

/* ====== Constants ====== */
#define DEBOUNCE_THRESH    1       // Super fast response - minimal debouncing
#define PWM_CYCLE          10    // 10 steps → 1 kHz @ 1 ms tick

#define ECHO_START_TIMEOUT 4000   // ~4ms
#define ECHO_END_TIMEOUT   4000
#define DISTANCE_MIN_CM    2
#define DISTANCE_MAX_CM    400
#define STOP_THRESHOLD_CM  20     // Stop if object closer than this

/* ====== Globals ====== */
volatile int8_t dir_left   = 0;
volatile int8_t dir_right  = 0;
volatile int8_t duty_left  = 0;
volatile int8_t duty_right = 0;
volatile uint16_t distance_cm = 0;
volatile uint8_t obstacle_detected = 0;
volatile uint8_t last_good_state = 0;  // Remember the last valid line position

// LCD segment data
const unsigned int seg_data[] = {
    0xFC00, 0x6000, 0xDB00, 0xF300, 0x6700,
    0xB700, 0xBF00, 0xE400, 0xFF00, 0xF700
};

/* ====== Function Prototypes ====== */
// Initialization
static void init_gpio(void);
static void init_ir(void);
static void init_ultrasonic(void);
static void init_timer(void);
static void lcd_init(void);

// Line Following
static void compute_control(uint8_t L, uint8_t Ce, uint8_t R);
static void apply_pwm(uint16_t tick);

// Ultrasonic
static unsigned int measure_distance_cm(void);
static void delay_us(unsigned int us);

// LCD Display
static void load_digit(unsigned char pos, unsigned int num);
static void lcd_display_number(unsigned int num);

/* ====== MAIN ====== */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;    // stop watchdog
    PM5CTL0 &= ~LOCKLPM5;        // unlock GPIO on FRAM devices

    init_gpio();
    init_ir();
    init_ultrasonic();
    init_timer();
    lcd_init();

    __enable_interrupt();        // enable interrupts globally
    __bis_SR_register(LPM0_bits);// enter low-power mode; ISR does all the work
    return 0;                    // never reached
}

/* ====== Initialization Functions ====== */
static void init_gpio(void) {
    // Left motor pins
    P2DIR |= IN1_PIN;    P2OUT &= ~IN1_PIN;
    P4DIR |= IN2_PIN;    P4OUT &= ~IN2_PIN;
    // Right motor pins
    P2DIR |= IN3_PIN;    P2OUT &= ~IN3_PIN;
    P1DIR |= IN4_PIN;    P1OUT &= ~IN4_PIN;
}

static void init_ir(void) {
    // P2.0
    P2DIR &= ~IR_LEFT_PIN;
    P2REN |= IR_LEFT_PIN;
    P2OUT |= IR_LEFT_PIN;
    // P1.4
    P1DIR &= ~IR_CTR_PIN;
    P1REN |= IR_CTR_PIN;
    P1OUT |= IR_CTR_PIN;
    // P9.3
    P9DIR &= ~IR_RGT_PIN;
    P9REN |= IR_RGT_PIN;
    P9OUT |= IR_RGT_PIN;
}

static void init_ultrasonic(void) {
    // Ultrasonic TRIG
    P1DIR |= TRIG_PIN;
    P1OUT &= ~TRIG_PIN;

    // Ultrasonic ECHO
    P2DIR &= ~ECHO_PIN;
    P2REN &= ~ECHO_PIN;
    P2OUT &= ~ECHO_PIN;
}

static void init_timer(void) {
    // Timer A0 for PWM and IR sensor polling (1 kHz)
    TA0CCR0   = 1000 - 1;                // SMCLK/1000 → 1 ms
    TA0CCTL0 |= CCIE;                    // CCR0 interrupt enable
    TA0CTL    = TASSEL_2 | MC_1 | TACLR; // SMCLK, up mode, clear

    // Timer A1 for ultrasonic measurement
    CSCTL0_H = CSKEY_H;
    CSCTL1   = DCOFSEL_0;
    CSCTL2   = SELS__DCOCLK;
    CSCTL3   = DIVS__1;
    CSCTL0_H = 0;

    TA1CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
}

static void lcd_init(void) {
    LCDCPCTL0 = 0xFFD0;
    LCDCPCTL1 = 0xF83F;
    LCDCPCTL2 = 0x00F8;
    LCDCCTL0  = LCDDIV__1 | LCDPRE__16 | LCD4MUX | LCDLP;
    LCDCMEMCTL = LCDCLRM;
    __delay_cycles(10000);
    LCDCCTL0 |= LCDON;
}

/* ====== Line Following Functions ====== */
static void compute_control(uint8_t L, uint8_t Ce, uint8_t R) {
    // If obstacle detected, stop motors regardless of line position
    if (obstacle_detected) {
        dir_left = dir_right = 0;
        duty_left = duty_right = 0;
        return;
    }

    // IMPROVED: Line following logic with differential turning
    // For IR sensors: 1 means sensor is over the black line (active)

    // Store the current sensor state for later reference
    uint8_t current_state = (L << 2) | (Ce << 1) | R;

    if (!L && !Ce && !R) {
        // No line detected - continue using last valid state
        // but at reduced speed, or go straight if no last state
        if (last_good_state == 0) {
            // Default to straight if truly lost
            dir_left = 1;  dir_right = 1;
            duty_left = 40; duty_right = 40;
        } else if (last_good_state & 0x04) {
            // Last saw line on left - turn left
            dir_left = -1;  dir_right = 1;
            duty_left = 70; duty_right = 70;
        } else if (last_good_state & 0x01) {
            // Last saw line on right - turn right
            dir_left = 1;  dir_right = -1;
            duty_left = 70; duty_right = 70;
        } else {
            // Default to straight
            dir_left = 1;  dir_right = 1;
            duty_left = 60; duty_right = 60;
        }
    }
    else if (!L && Ce && !R) {
        // Center line only - go straight (full speed)
        dir_left = 1;  dir_right = 1;
        duty_left = 80; duty_right = 80;
        last_good_state = current_state;
    }
    else if (L && !Ce && !R) {
        // Left sensor only - sharp left turn with PROPER DIFFERENTIAL MOTION
        dir_left = -1;  dir_right = 1;  // Left wheel REVERSE, right wheel forward
        duty_left = 70; duty_right = 80;
        last_good_state = current_state;
    }
    else if (!L && !Ce && R) {
        // Right sensor only - sharp right turn with PROPER DIFFERENTIAL MOTION
        dir_left = 1;  dir_right = -1;  // Left wheel forward, right wheel REVERSE
        duty_left = 80; duty_right = 70;
        last_good_state = current_state;
    }
    else if (L && Ce && !R) {
        // Left and center - moderate left turn
        dir_left = 0;  dir_right = 1;  // Left wheel stop, right wheel forward
        duty_left = 0; duty_right = 85;
        last_good_state = current_state;
    }
    else if (!L && Ce && R) {
        // Right and center - moderate right turn
        dir_left = 1;  dir_right = 0;  // Left wheel forward, right wheel stop
        duty_left = 85; duty_right = 0;
        last_good_state = current_state;
    }
    else if (L && !Ce && R) {
        // Both sides but not center - possible junction, use differential steering
        dir_left = -1;  dir_right = 1;  // Left wheel reverse, right wheel forward
        duty_left = 60; duty_right = 75;
        last_good_state = current_state;
    }
    else if (L && Ce && R) {
        // All sensors over line - intersection, continue straight
        dir_left = 1;  dir_right = 1;
        duty_left = 70; duty_right = 70;
        last_good_state = current_state;
    }

    // Safety check to make sure robot never completely stops due to bad state
    if (dir_left == 0 && dir_right == 0) {
        dir_left = 1;  dir_right = 1;
        duty_left = 50; duty_right = 50;
    }
}

static void apply_pwm(uint16_t tick) {
    // LEFT motor
    if (dir_left > 0) {
        // Forward at specified duty cycle
        if (tick < duty_left) {
            P2OUT |= IN1_PIN;   // Set output high when within duty cycle
        } else {
            P2OUT &= ~IN1_PIN;  // Otherwise set low
        }
        P4OUT &= ~IN2_PIN;      // Ensure reverse pin is off
    }
    else if (dir_left < 0) {
        // Backward at specified duty cycle
        if (tick < duty_left) {
            P4OUT |= IN2_PIN;   // Set output high when within duty cycle
        } else {
            P4OUT &= ~IN2_PIN;  // Otherwise set low
        }
        P2OUT &= ~IN1_PIN;      // Ensure forward pin is off
    }
    else {
        // Motor stopped (brake)
        P2OUT &= ~IN1_PIN;
        P4OUT &= ~IN2_PIN;
    }

    // RIGHT motor
    if (dir_right > 0) {
        // Forward at specified duty cycle
        if (tick < duty_right) {
            P2OUT |= IN3_PIN;   // Set output high when within duty cycle
        } else {
            P2OUT &= ~IN3_PIN;  // Otherwise set low
        }
        P1OUT &= ~IN4_PIN;      // Ensure reverse pin is off
    }
    else if (dir_right < 0) {
        // Backward at specified duty cycle
        if (tick < duty_right) {
            P1OUT |= IN4_PIN;   // Set output high when within duty cycle
        } else {
            P1OUT &= ~IN4_PIN;  // Otherwise set low
        }
        P2OUT &= ~IN3_PIN;      // Ensure forward pin is off
    }
    else {
        // Motor stopped (brake)
        P2OUT &= ~IN3_PIN;
        P1OUT &= ~IN4_PIN;
    }
}

/* ====== Ultrasonic Functions ====== */
static void delay_us(unsigned int us) {
    while (us--) __delay_cycles(1);
}

static unsigned int measure_distance_cm(void) {
    unsigned int start = 0, end = 0, duration = 0;
    unsigned long timeout;

    // Trigger 10µs pulse
    P1OUT &= ~TRIG_PIN;
    delay_us(2);
    P1OUT |= TRIG_PIN;
    delay_us(10);
    P1OUT &= ~TRIG_PIN;

    // Wait for echo start
    timeout = ECHO_START_TIMEOUT;
    while (!(P2IN & ECHO_PIN)) {
        if (--timeout == 0) return 0;
    }
    start = TA1R;

    // Wait for echo end
    timeout = ECHO_END_TIMEOUT;
    while (P2IN & ECHO_PIN) {
        if (--timeout == 0) return 0;
    }
    end = TA1R;

    // Calculate duration and convert to cm
    duration = (end >= start) ? (end - start)
                              : (0xFFFF - start + end);
    unsigned int dist_cm = duration / 58;

    if (dist_cm < DISTANCE_MIN_CM || dist_cm > DISTANCE_MAX_CM)
        return 0;

    return dist_cm;
}

/* ====== LCD Functions ====== */
static void load_digit(unsigned char pos, unsigned int num) {
    unsigned char hi = seg_data[num] >> 8;
    unsigned char lo = seg_data[num] & 0xFF;
    switch (pos) {
        case 1: LCDM10 = hi; LCDM11 = lo; break;
        case 2: LCDM6  = hi; LCDM7  = lo; break;
        case 3: LCDM4  = hi; LCDM5  = lo; break;
        case 4: LCDM19 = hi; LCDM20 = lo; break;
    }
}

static void lcd_display_number(unsigned int num) {
    load_digit(DIG1, (num / 1000) % 10);
    load_digit(DIG2, (num / 100) % 10);
    load_digit(DIG3, (num / 10) % 10);
    load_digit(DIG4, num % 10);
}

/* ====== Timer0_A0 ISR: 1 ms tick for PWM and sensors ====== */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void) {
    static uint16_t pwm_tick = 0;
    static uint16_t ultrasonic_timer = 0;
    static int8_t cntL = 0, cntCe = 0, cntR = 0;
    static uint8_t motor_update_counter = 0;
    static uint16_t recovery_timer = 0;

    // 1) Process ultrasonic sensor every 100ms (every 100 ticks)
    if (++ultrasonic_timer >= 100) {
        ultrasonic_timer = 0;
        distance_cm = measure_distance_cm();
        lcd_display_number(distance_cm);

        // Update obstacle detection flag - ONLY consider obstacles within threshold
        obstacle_detected = (distance_cm > 0 && distance_cm < STOP_THRESHOLD_CM);
    }

    // 2) Read IR sensors - active LOW means sensor is over black line
    // Reading sensors multiple times for better reliability
    uint8_t rawL1 = !(P2IN & IR_LEFT_PIN);
    uint8_t rawCe1 = !(P1IN & IR_CTR_PIN);
    uint8_t rawR1 = !(P9IN & IR_RGT_PIN);

    // Small delay between readings
    __delay_cycles(10);

    uint8_t rawL2 = !(P2IN & IR_LEFT_PIN);
    uint8_t rawCe2 = !(P1IN & IR_CTR_PIN);
    uint8_t rawR2 = !(P9IN & IR_RGT_PIN);

    // Use the more "detected" state between the two readings
    uint8_t rawL = rawL1 | rawL2;
    uint8_t rawCe = rawCe1 | rawCe2;
    uint8_t rawR = rawR1 | rawR2;

    // 3) Highly responsive debouncing - almost immediate reaction
    // If line detected, increment counter quickly
    // If line lost, immediately reset counter
    if (rawL) {
        if (cntL < DEBOUNCE_THRESH) cntL++;
    } else {
        cntL = 0;  // Immediate reset for quick response when line is lost
    }

    if (rawCe) {
        if (cntCe < DEBOUNCE_THRESH) cntCe++;
    } else {
        cntCe = 0;
    }

    if (rawR) {
        if (cntR < DEBOUNCE_THRESH) cntR++;
    } else {
        cntR = 0;
    }

    // Get debounced sensor values - only need 1 count to consider detection
    uint8_t L = (cntL > 0);  // Any count triggers detection
    uint8_t Ce = (cntCe > 0);
    uint8_t R = (cntR > 0);

    // 4) Compute control strategy frequently (every 2ms) for responsive turning
    if (++motor_update_counter >= 2) {
        motor_update_counter = 0;
        compute_control(L, Ce, R);

        // Reset the recovery timer if line is detected
        if (L || Ce || R) {
            recovery_timer = 0;
        }

        // If nothing detected for too long, force a recovery pattern
        if (!L && !Ce && !R) {
            if (++recovery_timer > 500) {  // ~1 second of no line
                // Execute a search pattern - slow spin in place
                dir_left = -1;
                dir_right = 1;
                duty_left = 50;
                duty_right = 50;
            }
        }
    }

    // 5) Update H-bridge outputs on every tick
    apply_pwm(pwm_tick);

    // 6) Advance & wrap PWM tick
    if (++pwm_tick >= PWM_CYCLE) pwm_tick = 0;
}

