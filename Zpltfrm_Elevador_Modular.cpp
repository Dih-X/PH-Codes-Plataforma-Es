#include <AccelStepper.h>

const int M_STEP_Z = 4;
const int M_DIR_Z = 7;

const int pinoEnable = 8;

AccelStepper motorX(AccelStepper::DRIVER, X_STEP_Pin, X_DIR_Pin); //AccelStepper::DRIVER
AccelStepper motorZ(AccelStepper::DRIVER, Z_STEP_Pin, Z_DIR_Pin);

enum Estado {
  IDLE,
  SUBIR,
  HOLD,
  DESCER,
  STOP_EMERGENCE,
};

Estado estado = IDLE;

// ------------------- CONFIGs ---------------------

const float VEL_MAX = 800.0;
const float ACEL = 400.0;

long passosX = 1600;  //dist. movimento 200passos
long passosZ = 1600;

unsigned long tempoEsperaZ = 0;

String comando = "";

//////FUNCOES////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void MoverMotor() {

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

//////SETUP//////////////////////////////////////////////////////////

void setup() {
  pinMode(pinoEnable, OUTPUT);
  digitalWrite(pinoEnable, LOW);

  Serial.begin(9600);

  pinMode(botaoStart, INPUT_PULLUP);
  pinMode(botaoStop, INPUT_PULLUP);

  motorX.setMaxSpeed(VEL_MAX);
  motorX.setAcceleration(ACEL);

  motorZ.setPinsInverted(true, false, true);        //inverte a direcao usando AccelStepper

  motorZ.setMaxSpeed(VEL_MAX);
  motorZ.setAcceleration(ACEL);

  motorX.setCurrentPosition(0);
  motorZ.setCurrentPosition(0);

}


void loop() {
  if (Serial.available()) {
    comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toLowerCase();

    if (comando == "atv" && estado == IDLE) {
      MoverMotor();
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