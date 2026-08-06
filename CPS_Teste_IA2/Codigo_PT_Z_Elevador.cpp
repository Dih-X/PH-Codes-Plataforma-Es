/*
  ============================================================
   ARDUINO Z - ELEVADOR + GARRA (ESCRAVO) - BASEADO NA v.3
  ============================================================
  Este arduino eh ESCRAVO no barramento I2C (endereco 0x09).
  Ele controla os 3 mecanismos do braco Z:
    - Elevador (motorZ / motor2Z)
    - Garra que segura a bateria (motorZgarra)
    - Extensao/avanco da garra (motorZYgarra)

  Fica esperando comandos do arduino Y (mestre). Cada comando
  chega com 2 bytes: [mecanismo, acao]. Quando o mestre pede
  status (Wire.requestFrom(Z_ADDR, 3)) este arduino devolve
  3 bytes, um status por mecanismo, nesta ordem:
  elevador, garra, extensao.

  IMPORTANTE: as constantes do protocolo abaixo tem que ser
  IDENTICAS as do arduino Y (mestre) e do arduino X.
  ============================================================
*/

#include <Wire.h>
#include <AccelStepper.h>

#define Z_ADDR 0x09

//====================== PROTOCOLO I2C ==========================
const byte MEC_ELEVADOR  = 0x01;
const byte MEC_GARRA     = 0x02;
const byte MEC_EXTENSAO  = 0x03;

const byte ACAO_MOVER     = 0x01;
const byte ACAO_RETORNAR  = 0x02;
const byte ACAO_PARAR     = 0x03;
const byte ACAO_ZERAR     = 0x04;

const byte STATUS_INICIO  = 0x01;
const byte STATUS_FIM     = 0x02;
const byte STATUS_MOVENDO = 0x04;
//================================================================

//Z Elevador da bateria
const int Z1_STEP = 2;
const int Z1_DIR  = 3;
const int Z2_STEP = 4;
const int Z2_DIR  = 5;

//Garra que segura a bateria (abre/fecha)
const int Zgarra_STEP = 6;
const int Zgarra_DIR  = 7;

//Extensao (avanco/recuo) da garra Z
const int ZY_STEP = 8;
const int ZY_DIR  = 9;

//SENSORES
const int Zstart   = A0;
const int Zend     = A1;
const int ZGstart  = A2;
const int ZGend    = A3;
const int ZExstart = 10;
const int ZExend   = 11;

const int pinoEnable = 12;     //Enable do driver CNC deste arduino

AccelStepper motorZ(AccelStepper::DRIVER, Z1_STEP, Z1_DIR);
AccelStepper motor2Z(AccelStepper::DRIVER, Z2_STEP, Z2_DIR);
AccelStepper motorZgarra(AccelStepper::DRIVER, Zgarra_STEP, Zgarra_DIR);
AccelStepper motorZYgarra(AccelStepper::DRIVER, ZY_STEP, ZY_DIR);

long passosZ = 1600;            //Dist = altura desejada onde se possa trocar a bateria do drone
long passosZgarra = 800;        //Passos = fechamento da garra da bateria
long passosZYgarra = 800;       //Passos = extensao/avanco da garra

const float VEL_MAX = 800.0;
const float ACEL = 200.0;

volatile byte mecanismoRecebido = 0;
volatile byte comandoRecebido = 0;
volatile bool novoComando = false;

void executarComando(byte mecanismo, byte acao); //protÃ³tipo

void setup(){
    Wire.begin(Z_ADDR);
    Wire.onReceive(receberComando);
    Wire.onRequest(enviarStatus);

    pinMode(pinoEnable, OUTPUT);
    digitalWrite(pinoEnable, LOW);

    pinMode(Zstart, INPUT_PULLUP);
    pinMode(Zend, INPUT_PULLUP);
    pinMode(ZGstart, INPUT_PULLUP);
    pinMode(ZGend, INPUT_PULLUP);
    pinMode(ZExstart, INPUT_PULLUP);
    pinMode(ZExend, INPUT_PULLUP);

    motorZ.setMaxSpeed(VEL_MAX);
    motorZ.setAcceleration(ACEL);

    motor2Z.setMaxSpeed(VEL_MAX);
    motor2Z.setAcceleration(ACEL);

    motorZgarra.setMaxSpeed(VEL_MAX);
    motorZgarra.setAcceleration(ACEL);

    motorZYgarra.setMaxSpeed(VEL_MAX);
    motorZYgarra.setAcceleration(ACEL);

    motorZ.setPinsInverted(false, true, false);
    motor2Z.setPinsInverted(true, false, true);   //2o motor espelhado (direcao invertida)
    motorZgarra.setPinsInverted(false, true, false);   //INVERTER CASO ESTEJA NO SENTIDO ERRADO
    motorZYgarra.setPinsInverted(false, true, false);

    Serial.begin(9600);   //opcional, so pra debug local via monitor serial
    Serial.println("Arduino Z pronto (escravo 0x09)");
}

// Chamado automaticamente quando o mestre manda um comando.
// Roda em contexto de interrupcao do I2C: mantido curto de proposito.
void receberComando(int numBytes){
    if (numBytes < 2) return;
    mecanismoRecebido = Wire.read();
    comandoRecebido = Wire.read();
    novoComando = true;
}

// Chamado automaticamente quando o mestre pede o status (Wire.requestFrom)
void enviarStatus(){
    byte statusElevador = 0;
    if (digitalRead(Zstart) == LOW) statusElevador |= STATUS_INICIO;
    if (digitalRead(Zend) == LOW)   statusElevador |= STATUS_FIM;
    if (motorZ.distanceToGo() != 0 || motor2Z.distanceToGo() != 0) statusElevador |= STATUS_MOVENDO;

    byte statusGarra = 0;
    if (digitalRead(ZGstart) == LOW) statusGarra |= STATUS_INICIO;
    if (digitalRead(ZGend) == LOW)   statusGarra |= STATUS_FIM;
    if (motorZgarra.distanceToGo() != 0) statusGarra |= STATUS_MOVENDO;

    byte statusExtensao = 0;
    if (digitalRead(ZExstart) == LOW) statusExtensao |= STATUS_INICIO;
    if (digitalRead(ZExend) == LOW)   statusExtensao |= STATUS_FIM;
    if (motorZYgarra.distanceToGo() != 0) statusExtensao |= STATUS_MOVENDO;

    Wire.write(statusElevador);
    Wire.write(statusGarra);
    Wire.write(statusExtensao);
}

void loop(){

    if (novoComando){
        novoComando = false;
        executarComando(mecanismoRecebido, comandoRecebido);
    }

    motorZ.run();
    motor2Z.run();
    motorZgarra.run();
    motorZYgarra.run();
}

void executarComando(byte mecanismo, byte acao){
    switch (mecanismo){

        case MEC_ELEVADOR:
            switch (acao){
                case ACAO_MOVER:    motorZ.moveTo(passosZ); motor2Z.moveTo(passosZ); break;
                case ACAO_RETORNAR: motorZ.moveTo(0); motor2Z.moveTo(0); break;
                case ACAO_PARAR:    motorZ.moveTo(motorZ.currentPosition()); motor2Z.moveTo(motor2Z.currentPosition()); break;
                case ACAO_ZERAR:    motorZ.setCurrentPosition(0); motor2Z.setCurrentPosition(0); break;
            }
            break;

        case MEC_GARRA:                                              //fecharGarraBateria() / abrirGarraBateria() do codigo original
            switch (acao){
                case ACAO_MOVER:    motorZgarra.moveTo(passosZgarra); break;   //fecha (pega a bateria)
                case ACAO_RETORNAR: motorZgarra.moveTo(0); break;              //abre
                case ACAO_PARAR:    motorZgarra.moveTo(motorZgarra.currentPosition()); break;
                case ACAO_ZERAR:    motorZgarra.setCurrentPosition(0); break;
            }
            break;

        case MEC_EXTENSAO:
            switch (acao){
                case ACAO_MOVER:    motorZYgarra.moveTo(passosZYgarra); break;  //extruda a garra
                case ACAO_RETORNAR: motorZYgarra.moveTo(0); break;              //recolhe a garra
                case ACAO_PARAR:    motorZYgarra.moveTo(motorZYgarra.currentPosition()); break;
                case ACAO_ZERAR:    motorZYgarra.setCurrentPosition(0); break;
            }
            break;
    }
}