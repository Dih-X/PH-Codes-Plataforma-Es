#include <AccelStepper.h>

// ---------------- PINOS ----------------

const int M_STEP_X = 2;
const int M_DIR_X = 5;

const int M_STEP_Y = 3;
const int M_DIR_Y = 6;

const int M_STEP_Z = 4;
const int M_DIR_Z = 7;

const int pinoEnable = 8;

const int botaoStart = A2;
const int botaoStop = A0;

AccelStepper motorX(1, M_STEP_X, M_DIR_X);
AccelStepper motorY(1, M_STEP_Y, M_DIR_Y);
AccelStepper motorZ(1, M_STEP_Z, M_DIR_Z);

enum Estado {
  IDLE,
  ATIVAR_X,
  ATIVAR_Z,
  ESPERA_Z,
  REVERSE_X,
  REVERSE_Z,
  STOP_EMERGENCE,
};

Estado estado = IDLE;

// --------------- CONFIGs ----------------

const float VEL_MAX = 800.0;
const float ACEL = 400.0;

long passosX = 800;  //dist. movimento 200passos
long passosY = 200;
long passosZ = 800;

unsigned long tempoEsperaZ = 0;

String comando = "";

bool working = false;

// -------------------------------------------------

void CycleBegin() {
  motorX.setCurrentPosition(0);
  motorY.setCurrentPosition(0);
  motorZ.setCurrentPosition(0);

  motorX.moveTo(passosX);
  working = true;
  estado = ATIVAR_X;
}

void EmerStopp() {
  estado = STOP_EMERGENCE;
  working = false;
}

// -------------------------------------------------

void setup() {
  Serial.begin(9600);

  pinMode(botaoStart, INPUT_PULLUP);
  pinMode(botaoStop, INPUT_PULLUP);

  motorX.setMaxSpeed(VEL_MAX);
  motorX.setAcceleration(ACEL);

  motorY.setMaxSpeed(VEL_MAX);
  motorY.setAcceleration(ACEL);

  motorZ.setMaxSpeed(VEL_MAX);
  motorZ.setAcceleration(ACEL);

  Serial.println("=============================");
  Serial.println(" | Digite ATV para acionar |");
  Serial.println(" | Digite EMR para parar   |");
  Serial.println("=============================");
}

// =====================================================

/*
=============================
 | Digite ATV para acionar |
 | Digite EMR para parar   |
=============================
 | ATIVANDO...             |
 | PARADA EMER...          |
 | Motor ja esta parado... |
 | comando desconhecido    | 
  | Start      - pressed |
  | Emer. Stop - pressed |
*/

void loop() {
  // -------------------------------------------------

  if (Serial.available()) {
    comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toLowerCase();

    if (comando == "atv" && estado == IDLE) {
      CycleBegin();
      Serial.println(" | ATIVANDO...             |");

    } else if (comando == "emr" && estado != IDLE) {
      EmerStopp();
      Serial.println(" | PARADA EMER...          |");

    } else if (comando == "emr" && estado == IDLE) {
      Serial.println(" | Motor ja esta parado... |");

    } else {
      Serial.println(" | comando desconhecido    |");
    }
  }

  // -------------------------------------------------

  if (digitalRead(botaoStart) == LOW && estado == IDLE) {
    delay(50);
    
    if (digitalRead(botaoStart) == LOW) {
      CycleBegin();
      Serial.println("  | Start      - pressed |");
      Serial.println(" | ATIVANDO...             |");
    }
  } else if (digitalRead(botaoStop) == LOW && estado != IDLE && estado != STOP_EMERGENCE) {
    EmerStopp();
    Serial.println("  | Emer. Stop - pressed |");
    Serial.println(" | PARADA EMER...          |");

  } else if (digitalRead(botaoStop) == LOW && estado == IDLE && estado) {
    Serial.println("| Motor ja esta parado... |");
  }

  // -------------------------------------------------

  switch (estado) {
    case IDLE:
      break;

    case ATIVAR_X:
      if (motorX.distanceToGo() == 0) {
        motorZ.moveTo(passosZ);
        Serial.println("Motor X andando...");

        estado = ATIVAR_Z;
      }

      break;
    case ATIVAR_Z:
      if (motorZ.distanceToGo() == 0) {
        Serial.println("Motor Z andando...");
        tempoEsperaZ = millis();

        estado = ESPERA_Z;
      }

      break;
    case ESPERA_Z:
      if (millis() - tempoEsperaZ >= 2000) {
        Serial.println("...espera simulada...");
        motorX.moveTo(0);
        
        estado = REVERSE_X;
      }

      break;
    case REVERSE_X:
      if (motorX.distanceToGo() == 0) {
        Serial.println("Motor X voltando...");
        motorZ.moveTo(0);

        estado = REVERSE_Z;
      }

      break;
    case REVERSE_Z:
      if (motorZ.distanceToGo() == 0) {
        Serial.println("Motor Z voltando...");

        estado = IDLE;
      }

      break;
    case STOP_EMERGENCE:

      motorX.moveTo(motorX.currentPosition());
      motorZ.moveTo(motorZ.currentPosition());

      motorX.setCurrentPosition(0);
      motorZ.setCurrentPosition(0);

      estado = IDLE;

      break;
  }

  motorX.run();
  motorZ.run();
  
}