#include <AccelStepper.h>

// ---------------- PINOS ----------------

const int M_STEPX = 2;
const int M_DIRX = 5;

const int M_STEPY = 2;
const int M_DIRY = 5;

const int M_STEPZ = 2;
const int M_DIRZ = 5;

const int pinoEnable = 8;

//const int botaoStart = A2;
//const int botaoStop = A0;

// --------------- CONFIGs ----------------

int delayPassos = 200;     //Controle de velocidade
bool motorLigado = false;  //Mudar esse faz o motor iniciar (Ligado/Desligado)
int passos = 600;          //Passos do motor (Ex.: 200p = 1 volta completa)
String comando = "";

// -------------------------------------------------

void setup() {

  pinMode(M_STEPX, OUTPUT);
  pinMode(M_DIRX, OUTPUT);

  //pinMode(botaoStart, INPUT_PULLUP);
  //pinMode(botaoStop, INPUT_PULLUP);

  //direção fixa "positiva"
  //digitalWrite(M_DIR, HIGH);
  
  Serial.begin(9600);
  Serial.println("Digite START ou STOP");
}

// =====================================================


void stepMotor(int stepPin) {

  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayPassos);

  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayPassos);
}

// =====================================================


void loop() {
  // -------------------------------------------------
  

  if (Serial.available()) {
    comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toLowerCase();

    if (comando == "start") {

      motorLigado = true;
      Serial.println("Motor ligado");

    } else if (comando == "stop") {

      motorLigado = false;
      Serial.println("Motor parando");

    } else {
      Serial.println("Comando desconhecido");
    }

    /*if (digitalRead(botaoStart) == LOW) {
        delay(30);
        while (digitalRead(botaoStart) == LOW) {}
        
    }

    if (digitalRead(botaoStop) == LOW) {
        delay(30);
        while (digitalRead(botaoStop) == LOW) {}
    }*/

    // -------------------------------------------------

    if (motorLigado) {

      digitalWrite(M_DIRX, HIGH);
      for (int i = 0; i < passos; i++) {
        stepMotor(M_STEPX);
        Serial.println("deve se ligar");
      }
      //stepMotor(M_STEP);
      //delay(800);

    }else if (motorLigado == false){

      digitalWrite(M_DIRX, LOW);
      for (int i = 0; i < passos; i++) {
        stepMotor(M_STEPX);
        Serial.println("deve se parar");
      }
    }
  }
}