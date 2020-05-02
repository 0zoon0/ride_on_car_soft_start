//move commands
#define STOP 0
#define MOVE_BACKWARD 1
#define MOVE_FORWARD 2
//motion state
#define NO_MOTION 0
#define MOTION_STARTED 1
#define MOVING 2
#define STOPPING 3
//pins
#define FRWD PB3 // pull up to 5v, goes to GND when requested to move forward
#define BCK PB4 // pull up to 5v, goes to GND when requested to move backward
#define FRWD_RELAY PB1 // 1 logic to switch relays to forward 
#define BCK_RELAY PB2 // 1 logic to switch relays to backward
#define PWM PB0 // PWM pulse for soft start/stop

uint8_t current_movement_direction = STOP;
volatile uint8_t movement_state = NO_MOTION;
volatile unsigned int ticks;  // pulses counter
volatile unsigned int pulse; // pulse threshold

ISR(TIM0_COMPA_vect)  // 100 KHz interrupt frequency
{
  if(ticks >= 2000)  // One frame (20ms) completed
  {
    ticks = 0;
    
    if (movement_state == MOTION_STARTED)
    {
      if (pulse < 2000)
        pulse += 40;
      else
        movement_state = MOVING;
    }
    else if (movement_state == STOPPING)
    {
      if (pulse > 0)
        pulse -= 40;
      else
        movement_state = NO_MOTION;  
    }
  }

  ticks = ticks + 1;
 
  if(ticks <= pulse)  // Generate pulse
    PORTB |= (1<<PWM);  // pulse high
  else
    PORTB &= ~(1<<PWM); // pulse low
} 

void setup() {

    // remove system clock prescaler so that it runs at 9.6Mhz
    //1. Write the Clock Prescaler Change Enable (CLKPCE) bit to one and all other bits in CLKPR to zero.
    //2. Within four cycles, write the desired value to CLKPS while writing a zero to CLKPCE. 
    // set CLKPCE to 1 and other bits to zero
    CLKPR = 1<<CLKPCE; //0b10000000;
    // set prescaler to 1
    CLKPR = 0b00000000;
    //when compiling we have to use the optimization flags -0s, otherwise the process might take more than four cycles

    // Configure timer 1 for CTC mode
    TCCR0A |= (1<<WGM01);
    TCCR0A &= ~(1<<WGM00);
    TCCR0B &= ~(1<<WGM02);
    
    TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
    OCR0A = 95; // Set CTC compare value, so that at 9.6 MHz we interupt every 96 (counting from 0) counts we get 100kHz 
    // No prescaler for timer
    TCCR0B |= (1<<CS00); 
    TCCR0B &= ~((1<<CS01) | (1<<CS02));

    ticks = 0;
    pulse = 0;

    DDRB |= (1<<PWM) | (1<<BCK_RELAY) | (1<<FRWD_RELAY) ; // set output pins
    DDRB &= ~(1<<FRWD | 1<<BCK); // set input pins

    sei(); //  Enable global interrupts
}

void loop() 
{
  if (movement_state == NO_MOTION)
  {
    //car has stopped
    current_movement_direction = STOP; //update current movement direction
    turnOnRelays(current_movement_direction); // turn off relays
  }

  //read desired direction
  uint8_t dir = determineDirection();

  if (current_movement_direction != dir) //a change has been requested
  {    
    if (current_movement_direction == STOP)
    {
      //car is stopped at this moment
      current_movement_direction = dir;
      turnOnRelays(current_movement_direction);
      movement_state = MOTION_STARTED; //PWM will start to increase gradually the power
    }
    else if (movement_state == MOTION_STARTED || movement_state == MOVING)
    {
      //car is moving and a change in direction has requested
      movement_state = STOPPING; //soft stop the car before changing the direction, PWM will decrease the power
    }
  }
  else //no change for direction has requested
  {
    if(current_movement_direction != STOP && movement_state == STOPPING)
    {
      //car is still moving but slowing down at this point
      movement_state = MOTION_STARTED; //increase power to full speed
    }
  }

  delay(10);
}

void writeDigital(uint8_t pin, bool value)
{
    if (value == HIGH) 
      PORTB |= (1<<pin); 
    else 
      PORTB &= ~(1<<pin);
}

uint8_t determineDirection(void)
{
  uint8_t forward_pin = 0, backward_pin = 0;

  // take several measurements to make sure the lines are settled
  for (uint8_t i=0; i<5; i++)
  {
    forward_pin += digitalRead(FRWD);
    backward_pin += digitalRead(BCK);
    delay(10);
  }

  if (forward_pin > 4 && backward_pin == 0)
    return MOVE_BACKWARD;
  else if (forward_pin == 0 && backward_pin > 4)
    return MOVE_FORWARD;
  else
    return STOP;
}

void turnOnRelays(uint8_t dir)
{
  if (dir == MOVE_FORWARD)
  {
    writeDigital(BCK_RELAY, LOW);
    delay(200); //wait relay to switch off 
    writeDigital(FRWD_RELAY, HIGH);
  }
  else if (dir == MOVE_BACKWARD)
  {
    writeDigital(FRWD_RELAY, LOW);
    delay(200); //wait relay to switch off 
    writeDigital(BCK_RELAY, HIGH);
  }
  else
  {
    writeDigital(FRWD_RELAY, LOW);
    writeDigital(BCK_RELAY, LOW);
  } 
  delay(100); //wait relay to switch  
}

