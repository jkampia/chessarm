// Robotic arm control code :)

// afaik pin #s used in digitalWriteFast() calls
// must be defined at compile time, so the pulse pins
// need to be individually assigned in this case
// maybe im dumb tho. if so sue me
const int pul_0 = 2;
const int pul_1 = 5; 
const int pul_2 = 6;
const int pul_3 = 24;
const int pul_4 = 27;

const int num_joints = 5; 

const int dir[num_joints] = {3, 6, 9,  25, 28};
const int ena[num_joints] = {4, 7, 10, 26, 29};

uint64_t last_halfstep_time[num_joints];
uint64_t last_fullstep_time[num_joints];

int test_steps_to_move[num_joints] = {0, 0, 0, 0, 0};

uint64_t last_led_high_time; 




void setup() 
{
  const int baud = 921600;
  Serial.begin(baud);
  Serial.println("Serial init with baud " + String(baud));

  pinMode(LED_BUILTIN, OUTPUT); 

  // set all io pins to their respective modes
  for (int i = 0; i < num_joints; i++) 
  {
    pinMode(dir[i], OUTPUT);
    pinMode(ena[i], OUTPUT);
  }

  // set all timers
  for (int i = 0; i < num_joints; i++)
  {
    last_halfstep_time[i] = micros();
    last_fullstep_time[i] = micros(); 
  }

  last_led_high_time = micros(); 
}





void asyncPulseLED(const int period=1000000, const float duty_cycle=0.2)
{
  /*
  This function exists to show externally that the main timer is still
  running smoothly, if it stops blinking something crashed
  */
  if (int(micros() - last_led_high_time) > period)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    last_led_high_time = micros();
  }
  else if (int(micros() - last_led_high_time) > period * duty_cycle)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}







void asyncTestJ0(int& steps_to_move, int delay_us) 
{
  const int idx = 0;
  if (steps_to_move == 0) // if j0 is not yet in movement
  {
    last_fullstep_time[idx] = micros() - delay_us; 
    last_halfstep_time[idx] = micros() - delay_us/2;
    return;
  }

  if (int(micros() - last_fullstep_time[idx]) > delay_us) // enough time has passed since last high write
  {
    digitalWriteFast(pul_0, HIGH);
    last_fullstep_time[idx] = micros(); 
  }
  else if (int(micros() - last_halfstep_time[idx]) > delay_us)
  {
    digitalWriteFast(pul_0, LOW);
    last_halfstep_time[idx] = micros(); 
    steps_to_move--; 
  }
}


void runAsyncTests()
{
  asyncTestJ0(test_steps_to_move[0], 4000); 
}



void loop() 
{
  asyncPulseLED();

}






