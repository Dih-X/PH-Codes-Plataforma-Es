/*
  ============================================================
   ARDUINO X - GARRA ALINHADORA (ESCRAVO) - BASEADO NA v.3
  ============================================================
  Este arduino eh ESCRAVO no barramento I2C (endereco 0x08).
  Ele so controla os motores X1/X2 (garra que alinha e segura
  o drone) e os fins de curso de cada lado da garra.

  Fica esperando comandos do arduino Y (mestre) e responde com
  um byte de status quando o mestre pede (Wire.requestFrom).

  IMPORTANTE: as constantes do protocolo abaixo tem que ser
  IDENTICAS as do arduino Y (mestre) e do arduino Z.
  ============================================================
*/

#include <Wire.h>
#include <AccelStepper.h>

#define X_ADDR 0x08

//====================== PROTOCOLO I2C ==========================
const byte ACAO_MOVER     = 0x01;
const byte ACAO_RETORNAR  = 0x02;
const byte ACAO_PARAR     = 0x03;
const byte ACAO_ZERAR     = 0x04;

const byte STATUS_X1_INICIO = 0x01;
const byte STATUS_X1_FIM    = 0x02;
const byte STATUS_X2_INICIO = 0x04;
const byte STATUS_X2_FIM    = 0x08;
const byte STATUS_X_MOVENDO = 0x10;
//================================================================

//X - Alinhadores da pinÃ§a
const int X1_STEP = 2;
const int X1_DIR  = 3;
const int X2_STEP = 4;
const int X2_DIR  = 5;

//SENSORES
const int Xstart  = 6;
const int Xend    = 7;
const int X2start = 8;
const int X2end   = 9;

const int pinoEnable = 10;     //Enable do driver CNC deste arduino

AccelStepper motorX(AccelStepper::DRIVER, X1_STEP, X1_DIR);
AccelStepper motor2X(AccelStepper::DRIVER, X2_STEP, X2_DIR);

long passosX = 800;             //dist. movimento | Dist = metade da plataforma - largura da base do drone
const float VEL_MAX = 800.0;
const float ACEL = 200.0;

volatile byte comandoRecebido = ACAO_PARAR;   //ultimo comando recebido do mestre
volatile bool novoComando = false;

void setup(){
    Wire.begin(X_ADDR);
    Wire.onReceive(receberComando);
    Wire.onRequest(enviarStatus);

    pinMode(pinoEnable, OUTPUT);
    digitalWrite(pinoEnable, LOW);

    pinMode(Xstart, INPUT_PULLUP);
    pinMode(Xend, INPUT_PULLUP);
    pinMode(X2start, INPUT_PULLUP);
    pinMode(X2end, INPUT_PULLUP);

    motorX.setMaxSpeed(VEL_MAX);
    motorX.setAcceleration(ACEL);

    motor2X.setMaxSpeed(VEL_MAX);
    motor2X.setAcceleration(ACEL);

    motorX.setPinsInverted(false, true, false);
    motor2X.setPinsInverted(true, false, true);   //2o motor espelhado (direcao invertida)

    Serial.begin(9600);   //opcional, so pra debug local via monitor serial
    Serial.println("Arduino X pronto (escravo 0x08)");
}

// Chamado automaticamente quando o mestre manda um comando.
// Roda em contexto de interrupcao do I2C: mantido curto de proposito.
void receberComando(int numBytes){
    if (numBytes < 1) return;
    comandoRecebido = Wire.read();
    novoComando = true;
}

// Chamado automaticamente quando o mestre pede o status (Wire.requestFrom)
void enviarStatus(){
    byte status = 0;
    if (digitalRead(Xstart) == LOW)  status |= STATUS_X1_INICIO;
    if (digitalRead(Xend) == LOW)    status |= STATUS_X1_FIM;
    if (digitalRead(X2start) == LOW) status |= STATUS_X2_INICIO;
    if (digitalRead(X2end) == LOW)   status |= STATUS_X2_FIM;
    if (motorX.distanceToGo() != 0 || motor2X.distanceToGo() != 0) status |= STATUS_X_MOVENDO;
    Wire.write(status);
}

void loop(){

    if (novoComando){
        novoComando = false;
        switch (comandoRecebido){
            case ACAO_MOVER:
                motorX.moveTo(passosX);
                motor2X.moveTo(passosX);
                break;
            case ACAO_RETORNAR:
                motorX.moveTo(0);
                motor2X.moveTo(0);
                break;
            case ACAO_PARAR:
                motorX.moveTo(motorX.currentPosition());
                motor2X.moveTo(motor2X.currentPosition());
                break;
            case ACAO_ZERAR:
                motorX.setCurrentPosition(0);
                motor2X.setCurrentPosition(0);
                break;
        }
    }

    motorX.run();
    motor2X.run();
}