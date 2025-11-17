byte ledPin = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("work");
  digitalWrite(ledPin, HIGH); 
  delay(1000); 
  digitalWrite(ledPin, LOW); 
  delay(1000); 
}
