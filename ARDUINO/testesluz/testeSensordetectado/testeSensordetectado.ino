int valoresluz[11];
int minluz[11];
void setup() {
  // put your setup code here, to run once:
  valoresluz[0] = analogRead(A0);
  valoresluz[1] = analogRead(A1);
  valoresluz[2] = analogRead(A2);
  valoresluz[3] = analogRead(A3);
  valoresluz[4] = analogRead(A4);
  valoresluz[5] = analogRead(A5);
  valoresluz[6] = analogRead(A6);
  valoresluz[7] = analogRead(A7);
  valoresluz[8] = analogRead(A8);
  valoresluz[9] = analogRead(A9);
  valoresluz[10] = analogRead(A10);
  for (int i = 0; i < 11; i++)
  {
    minluz[i] = valoresluz[i];
    //valoresluz[i] = map(valoresluz[i], minluz[i], 1024, 0, 10);
  }
}

void loop() {
  valoresluz[0] = analogRead(A0);
  valoresluz[1] = analogRead(A1);
  valoresluz[2] = analogRead(A2);
  valoresluz[3] = analogRead(A3);
  valoresluz[4] = analogRead(A4);
  valoresluz[5] = analogRead(A5);
  valoresluz[6] = analogRead(A6);
  valoresluz[7] = analogRead(A7);
  valoresluz[8] = analogRead(A8);
  valoresluz[9] = analogRead(A9);
  valoresluz[10] = analogRead(A10);
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 11; i++)
  {
    valoresluz[i] = map(valoresluz[i], minluz[i], 1024, 0, 10);
    if (i != 10) {
      Serial.print(valoresluz[i]);
      Serial.print(" ");
    }
    else
    {
      Serial.print(valoresluz[i]);
      Serial.println(" ");
    }
    if (valoresluz[i] > 5)
    {
      int ultimobranco = i;
    }
  }

}
