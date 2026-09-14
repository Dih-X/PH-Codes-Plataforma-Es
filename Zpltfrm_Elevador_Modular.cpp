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

/////////////////////////////////////////////////////////////////////