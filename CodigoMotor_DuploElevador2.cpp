#include <AccelStepper.h>

// ---------------- PINOS ----------------

const int X_STEP_Pin = 2;
const int X_DIR_Pin = 5;

//const int M_STEP_Y = 3;
//const int M_DIR_Y = 6;

const int Z_STEP_Pin = 4;
const int Z_DIR_Pin = 7;

const int pinoEnable = 8;

const int botaoStart = A2;
const int botaoStop = A0;

AccelStepper motorX(AccelStepper::DRIVER, X_STEP_Pin, X_DIR_Pin); //AccelStepper::DRIVER
AccelStepper motorZ(AccelStepper::DRIVER, Z_STEP_Pin, Z_DIR_Pin);

/*enum Estado {
  IDLE,
  ATIVAR_X,
  ATIVAR_Z,
  ESPERA_Z,
  REVERSE_X,
  REVERSE_Z,
  STOP_EMERGENCE,
};*/

enum Estado {
  IDLE,
  SUBIR,
  HOLD,
  DESCER,
  STOP_EMERGENCE,
};

Estado estado = IDLE;

// --------------- CONFIGs ----------------

const float VEL_MAX = 800.0;
const float ACEL = 400.0;

long passosX = 1600;  //dist. movimento 200passos
long passosZ = 1600;
//long passosY = 200;

unsigned long tempoEsperaZ = 0;

String comando = "";

//bool working = false;

// -------------------------------------------------

void CycleBegin() {
  motorX.setCurrentPosition(0);
  motorZ.setCurrentPosition(0);

  motorX.moveTo(passosX);
  motorZ.moveTo(passosZ);

  estado = SUBIR;
}

void Descer(){
  motorX.moveTo(0);
  motorZ.moveTo(0);

  estado = DESCER;
}

void EmerStopp() {
  estado = STOP_EMERGENCE;
}

void Retorno(){
  motorX.moveTo(0);
  motorZ.moveTo(0);
  estado = IDLE;
}
// -------------------------------------------------

void setup() {
  pinMode(pinoEnable, OUTPUT);
  digitalWrite(pinoEnable, LOW);

  Serial.begin(9600);

  pinMode(botaoStart, INPUT_PULLUP);
  pinMode(botaoStop, INPUT_PULLUP);

  motorX.setMaxSpeed(VEL_MAX);
  motorX.setAcceleration(ACEL);

  //motorY.setMaxSpeed(VEL_MAX);
  //motorY.setAcceleration(ACEL);

  motorZ.setPinsInverted(true, false, true); //inverte a direcao usando AccelStepper

  motorZ.setMaxSpeed(VEL_MAX);
  motorZ.setAcceleration(ACEL);

  Serial.println("=============================");
  Serial.println(" | Digite ATV para acionar | ");
  Serial.println(" | Digite DSC para voltar  | ");
  Serial.println(" | Digite EMR para parar   | ");
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
      Serial.println(" | SUBINDO...              |");

    } else if (comando == "atv" && estado != IDLE){
      Serial.println(" | ESPERE DESCER CMPLTMNT..|");
      
    } else if (comando == "dsc" && estado == HOLD){
      Descer();
      Serial.println(" | DESCENDO...             |");
      
    } else if (comando == "dsc" && estado == STOP_EMERGENCE){
      Descer();
      Serial.println(" | DESCENDO POS EMR...     |");
      
    } else if (comando == "dsc" && estado != HOLD){
      Serial.println(" | ESPERE O CICLO FNLZR... |");

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

  /*if (digitalRead(botaoStart) == LOW && estado == IDLE) {
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
  }*/

  // -------------------------------------------------

  switch (estado) {
    case IDLE:
      break;

    case SUBIR:
      if (motorX.distanceToGo() == 0 && motorZ.distanceToGo() == 0) {
        Serial.println("Motor X & Z subindo...");

        estado = HOLD;
      }

      break;
    case HOLD:
        //Serial.println("X & Z holdados... ");
      break;
    case DESCER:
      if (motorX.distanceToGo() == 0 && motorZ.distanceToGo() == 0) {
        Serial.println("X & Z descendo...");

        estado = IDLE;
      }

      break;
    case STOP_EMERGENCE:

      motorX.moveTo(motorX.currentPosition());
      motorZ.moveTo(motorZ.currentPosition());

      if ((comando == "atv" || comando == "dsc") && estado == STOP_EMERGENCE){
        //delay(1000);
        Retorno();
        Serial.println(" | DESCENDO POS EMR CMD    |");
      }

      break;
  }

  motorX.run();
  motorZ.run();
  
}