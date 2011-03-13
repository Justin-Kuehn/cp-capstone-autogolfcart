#define INPUT 0
#define INPUTPIN 5 
#define TICKSPERREV 160.0
#define CIRCUMFERENCE 1.1 //(in meters)
#define MAX_TICKS 32001

unsigned int previousTime = 0;
unsigned int currentTime = 0;
unsigned int deltaTime = 0;
int state = 0;
int val = 0;
int ticks = 0;

void setup()
{
  pinMode(INPUTPIN, INPUT);
  Serial.begin(9600);
  previousTime = millis();
}

void loop()
{ 
  if((val = digitalRead(INPUTPIN)) != state)
  {
    ticks = (ticks + 1) % MAX_TICKS;
    state = val;
  }

  currentTime = millis();
  if( (currentTime - previousTime) > 100 ) // Send at 10 Hz
  { 
    previousTime = currentTime;
    Serial.println(ticks);
  }
}

