#include <AccelStepper.h>

const int Z1_STEP = 3;  //6
const int Z1_DIR = 6;
const int Z2_STEP = 4;  //8
const int Z2_DIR = 7;   //9

const int pinoEnable = 8;

const int ZY_STEP = 99;
const int ZY_DIR = 98;

const int Zgarra_STEP = 10;  //10   
const int Zgarra_DIR = 15;

AccelStepper motorZ(AccelStepper::DRIVER, Z1_STEP, Z1_DIR);
AccelStepper motor2Z(AccelStepper::DRIVER, Z2_STEP, Z2_DIR);

AccelStepper motorZgarra(AccelStepper::DRIVER, Zgarra_STEP, Zgarra_DIR);  //Garra Open/Close
AccelStepper motorZYgarra(AccelStepper::DRIVER, ZY_STEP, ZY_DIR);

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
long passosZgarra = 1600;

unsigned long tempoEsperaZ = 0;

String comando = "";

//////FUNCOES////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void MoverMotor() {

  motorX.moveTo(passosX);
  motorZ.moveTo(passosZ);
  motorZgarra.moveTo(passosZgarra);

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

  motorZ.setMaxSpeed(VEL_MAX);
  motorZ.setAcceleration(ACEL);

  motor2Z.setMaxSpeed(VEL_MAX);   //2M
  motor2Z.setAcceleration(ACEL);  //2M

  motorZgarra.setMaxSpeed(VEL_MAX);
  motorZgarra.setAcceleration(ACEL);

  pinMode(Zgarra_STEP, OUTPUT);
  pinMode(Zgarra_DIR, OUTPUT);
   
  pinMode(Zstart, INPUT_PULLUP);
  pinMode(Zend, INPUT_PULLUP);

  motorZ.setPinsInverted(false, true, false); 
  motor2Z.setPinsInverted(true, false, true); 

  motorZgarra.setPinsInverted(false, true, false);

  Serial.begin(9600);

  //pinMode(botaoStart, INPUT_PULLUP);
  //pinMode(botaoStop, INPUT_PULLUP);

  //motorX.setMaxSpeed(VEL_MAX);
  //motorX.setAcceleration(ACEL);


  motorX.setCurrentPosition(0);
  motorZ.setCurrentPosition(0);
  motorZgarra.setCurrentPosition(0);
}

//////LOOP///////////////////////////////////////////////////////////

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
  motor2Z.run();
  motorZgarra.run(); 
}