#include <VirtualWire.h>

void setup() {
  Serial.begin(9600);
  vw_setup(2000);
  Serial.println("Go !");
  pinMode(13, OUTPUT);
}

void loop() {
  byte message[VW_MAX_MESSAGE_LEN];
  // Lit un message de maximum 26 caractères depuis le port série
  int len = Serial.readBytesUntil('\n', (char*) message, VW_MAX_MESSAGE_LEN - 1);
  if (!len) {
    digitalWrite(13, LOW);
    delay(1000);
    return; // Pas de message
  }
  message[len] = '\0'; // Ferme la chaine de caractères

  digitalWrite(13, HIGH);
  delay(1000);
  vw_send(message, len + 1); // On envoie le message
  vw_wait_tx(); // On attend la fin de l'envoi
}
