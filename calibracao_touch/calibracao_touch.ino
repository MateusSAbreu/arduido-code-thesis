#include <XPT2046_Touchscreen.h>

#define TOUCH_CS 7

XPT2046_Touchscreen ts(TOUCH_CS);

void setup() {
  Serial.begin(115200);
  delay(1000);

  ts.begin();
  ts.setRotation(3);   // usa a mesma rotação do teu projeto

  Serial.println("=== CALIBRACAO TOUCH XPT2046 ===");
  Serial.println("Toca nos 4 cantos do ecrã lentamente.");
  Serial.println("Mantém o dedo uns 2 segundos em cada canto.");
  Serial.println();
}

void loop() {
  if (ts.touched()) {
    TS_Point p = ts.getPoint();

    Serial.print("RAW X = ");
    Serial.print(p.x);
    Serial.print("   RAW Y = ");
    Serial.println(p.y);

    delay(200);  // evita spam massivo
  }
}
