void buzz_100ms()
{
  digitalWrite(BUZZ_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZ_PIN, LOW);
  delay(100);
}

void start_buzzer()
{
  pinMode(BUZZ_PIN, OUTPUT);
  buzz_100ms();
}

bool is_armed()
{
  return radio_data.switches[4] > 1500;
}

bool kill()
{
  return radio_data.switches[5] > 1500;
}
