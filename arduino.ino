#define NEUTRAL 307
#define REVERSE 206
#define FORWARD 409

const int SteeringPin =  4;
const int ForwordPin =  3;

int tmp_angle = 90;
int angle = 0;
int newAngle = 0;
int idx = 0;
int value = 0;

const int MaxChars = 4;
char strValue[MaxChars+1];
unsigned int SteeringPWM;
unsigned int ForwardPWM;

int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    return toReturn ;
}


void setup() {
  Serial.begin(115200);
  analogWriteFrequency(SteeringPin, 50);
  analogWriteFrequency(ForwordPin, 50);
  analogWriteResolution(12);  // analogWrite value 0 to 4095
  pinMode(SteeringPin, OUTPUT);
  pinMode(ForwordPin, OUTPUT);
  
  SteeringPWM = NEUTRAL;
  ForwardPWM = NEUTRAL;
  
  analogWrite(SteeringPin, SteeringPWM);
  analogWrite(ForwordPin, ForwardPWM);
  delay(3500);
  
  angle = 90;
}

void loop() {
}



void serialEvent()
{
   while(Serial.available()) 
   {
      char ch = Serial.read();
      Serial.write(ch);
      if(idx < MaxChars && isDigit(ch)) { 
            strValue[idx++] = ch; 
      } else { 
            strValue[idx] = 0; 
            newAngle = atoi(strValue); 
            if(newAngle > 0 && newAngle < 180){
                   if(newAngle < angle){
                       for(; angle > newAngle; angle -= 1) {
                         tmp_angle = angle -5;
                         analogWrite(SteeringPin, map(tmp_angle, 0, 180, 206, 409));
                         analogWrite(ForwordPin, 311);
                       }
                   }  
                    else{
                       for(; angle < newAngle; angle += 1){
                         tmp_angle = angle +5;
                         analogWrite(SteeringPin, map(tmp_angle, 0, 180, 206, 409));
                         analogWrite(ForwordPin, 311);
                       }
                    }
            } 
                
             else if(newAngle == 0){
               analogWrite(SteeringPin, map(90, 0, 180, 206, 409)); 
               analogWrite(ForwordPin, 307);
               delay(2000);
             }

             else;
            idx = 0;
            angle = newAngle;
      }  
   }
}
