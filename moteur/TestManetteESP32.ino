const int analogPin = 4; //premier signal
const int analogPin2 = 2; //deuxieme signal
int angle;
int angle2;

void setup() {
  Serial.begin(9600);
}

void loop() {
  angle = analogRead(analogPin);
  angle2 = analogRead(analogPin2);
  Serial.print("angle :");
  Serial.println(angle);
  Serial.print("angle 2 :");
  Serial.println(angle2);
  Serial.println("");
  // put your main code here, to run repeatedly:

}
