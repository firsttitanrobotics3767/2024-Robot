void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // pinMode(A0, INPUT);
}

int val = 0;
int valMod = 0;

void loop() {
  // put your main code here, to run repeatedly:
  valMod = analogRead(A0) - 80;

  val = analogRead(A0);
  // val = val - 80;

  Serial.print("valMod:");
  Serial.print(valMod);
  Serial.print(",");
  Serial.print("raw val:");
  Serial.println(val);
  delay(100);
}
