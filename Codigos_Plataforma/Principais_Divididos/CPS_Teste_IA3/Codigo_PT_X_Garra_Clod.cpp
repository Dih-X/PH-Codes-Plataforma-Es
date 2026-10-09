/*
  ============================================================
   ARDUINO X - GARRA ALINHADORA (ESCRAVO) - v.4 (revisado)
  ============================================================
  Escravo I2C no endereco 0x08. Controla os motores X1/X2 e os
  fins de curso de cada lado da garra. Obedece o arduino Y.

  MUDANCAS EM RELACAO A v.3
   - Fila de comandos (nada se perde se o mestre mandar varios
     comandos seguidos).
   - Cada lado (X1 e X2) para NA HORA no seu proprio fim de
     curso, sem depender do mestre nem do outro lado.
   - Comando pendente conta como "MOVENDO" no status (o mestre
     nao confunde "ainda nao processei" com "ja terminei").
   - ACAO_EMERGENCIA / ACAO_HABILITAR.
   - Watchdog: se o mestre ficar mudo enquanto os motores andam,
     entra em emergencia sozinho.
   - ACAO_PARAR agora e parada suave (stop()).

  O protocolo abaixo precisa ser IDENTICO nos tres codigos.
  ============================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

#define X_ADDR 0x08

//====================== PROTOCOLO I2C ==========================
const byte ACAO_MOVER       = 0x01;
const byte ACAO_RETORNAR    = 0x02;
const byte ACAO_PARAR       = 0x03;
const byte ACAO_ZERAR       = 0x04;
const byte ACAO_EMERGENCIA  = 0x05;
const byte ACAO_HABILITAR   = 0x06;

const byte STATUS_X1_INICIO     = 0x01;
const byte STATUS_X1_FIM        = 0x02;
const byte STATUS_X2_INICIO     = 0x04;
const byte STATUS_X2_FIM        = 0x08;
const byte STATUS_X_MOVENDO     = 0x10;
const byte STATUS_X_EMERGENCIA  = 0x20;
//================================================================

// X - Alinhadores da pinca
const int X1_STEP = 2;
const int X1_DIR  = 3;
const int X2_STEP = 4;
const int X2_DIR  = 5;

// SENSORES
const int Xstart  = 6;
const int Xend    = 7;
const int X2start = 8;
const int X2end   = 9;

const int pinoEnable = 10;     // Enable do driver (ativo em LOW)

// Se algum motor girar ao contrario, troque aqui (o 2o e espelhado)
const bool INV_DIR_X1 = false;
const bool INV_DIR_X2 = true;

// Na emergencia, corta o enable dos drivers?
const bool DESLIGAR_DRIVER_NA_EMERGENCIA = true;

AccelStepper motorX(AccelStepper::DRIVER, X1_STEP, X1_DIR);
AccelStepper motor2X(AccelStepper::DRIVER, X2_STEP, X2_DIR);

long passosX = 800;             // distancia do movimento da garra
const float VEL_MAX = 800.0;
const float ACEL = 200.0;

const unsigned long TIMEOUT_MESTRE = 1500;   // ms sem contato do mestre com motor andando

// Fila de comandos recebidos (escrita na interrupcao, lida no loop)
const byte TAM_FILA = 8;
volatile byte filaAcao[TAM_FILA];
volatile byte filaIni = 0;
volatile byte filaFim = 0;
volatile bool processando = false;

volatile bool emergencia = false;
volatile bool pedidoEmergencia = false;
volatile unsigned long ultimoContato = 0;

//--------------------------------------------------------------
void pararImediato(AccelStepper &m){
    m.setCurrentPosition(m.currentPosition());
}

void pararSuave(AccelStepper &m){
    if (m.speed() == 0.0) m.moveTo(m.currentPosition());
    else m.stop();
}

bool algumMotorMovendo(){
    return motorX.distanceToGo() != 0 || motor2X.distanceToGo() != 0;
}

// Fim de curso: so olha o sensor do lado para onde o motor anda.
void vigiarLado(AccelStepper &m, int pIni, int pFim){
    long d = m.distanceToGo();
    if (d > 0 && digitalRead(pFim) == LOW){
        pararImediato(m);
    }else if (d < 0 && digitalRead(pIni) == LOW){
        m.setCurrentPosition(0);            // re-zera no sensor de inicio
    }
}

void entrarEmergencia(){
    pararImediato(motorX);
    pararImediato(motor2X);
    if (DESLIGAR_DRIVER_NA_EMERGENCIA) digitalWrite(pinoEnable, HIGH);
    emergencia = true;
}

void executarComando(byte acao){

    if (acao == ACAO_HABILITAR){
        digitalWrite(pinoEnable, LOW);
        emergencia = false;
        return;
    }
    if (emergencia && (acao == ACAO_MOVER || acao == ACAO_RETORNAR)) return;

    switch (acao){
        case ACAO_MOVER:
            motorX.moveTo(passosX);
            motor2X.moveTo(passosX);
            break;
        case ACAO_RETORNAR:
            motorX.moveTo(0);
            motor2X.moveTo(0);
            break;
        case ACAO_PARAR:
            pararSuave(motorX);
            pararSuave(motor2X);
            break;
        case ACAO_ZERAR:
            motorX.setCurrentPosition(0);
            motor2X.setCurrentPosition(0);
            break;
    }
}

//--------------------------------------------------------------
// Interrupcao I2C: curta de proposito.
void receberComando(int numBytes){
    ultimoContato = millis();
    while (numBytes-- > 0 && Wire.available()){
        byte acao = Wire.read();
        if (acao == ACAO_EMERGENCIA){
            pedidoEmergencia = true;          // tratado de imediato no loop
        }else{
            byte prox = (filaFim + 1) % TAM_FILA;
            if (prox != filaIni){             // fila cheia = descarta
                filaAcao[filaFim] = acao;
                filaFim = prox;
            }
        }
    }
    while (Wire.available()) Wire.read();
}

void enviarStatus(){
    ultimoContato = millis();

    byte status = 0;
    if (digitalRead(Xstart) == LOW)  status |= STATUS_X1_INICIO;
    if (digitalRead(Xend) == LOW)    status |= STATUS_X1_FIM;
    if (digitalRead(X2start) == LOW) status |= STATUS_X2_INICIO;
    if (digitalRead(X2end) == LOW)   status |= STATUS_X2_FIM;

    bool pendente = (filaIni != filaFim) || processando;
    if (algumMotorMovendo() || pendente) status |= STATUS_X_MOVENDO;
    if (emergencia) status |= STATUS_X_EMERGENCIA;

    Wire.write(status);
}

//--------------------------------------------------------------
void setup(){

    pinMode(pinoEnable, OUTPUT);
    digitalWrite(pinoEnable, LOW);

    pinMode(Xstart, INPUT_PULLUP);
    pinMode(Xend, INPUT_PULLUP);
    pinMode(X2start, INPUT_PULLUP);
    pinMode(X2end, INPUT_PULLUP);

    motorX.setMaxSpeed(VEL_MAX);
    motorX.setAcceleration(ACEL);
    motorX.setMinPulseWidth(5);

    motor2X.setMaxSpeed(VEL_MAX);
    motor2X.setAcceleration(ACEL);
    motor2X.setMinPulseWidth(5);

    // setPinsInverted(direcao, step, enable)
    motorX.setPinsInverted(INV_DIR_X1, false, false);
    motor2X.setPinsInverted(INV_DIR_X2, false, false);

    Serial.begin(9600);   // opcional, so debug local
    Serial.println("Arduino X pronto (escravo 0x08)");

    Wire.begin(X_ADDR);   // por ultimo: so atende o mestre com tudo configurado
    Wire.onReceive(receberComando);
    Wire.onRequest(enviarStatus);
}

void loop(){

    if (pedidoEmergencia){
        pedidoEmergencia = false;
        entrarEmergencia();
    }

    // tira UM comando da fila (com interrupcoes desligadas)
    byte acao = 0;
    bool tem = false;
    noInterrupts();
    if (filaIni != filaFim){
        acao = filaAcao[filaIni];
        filaIni = (filaIni + 1) % TAM_FILA;
        tem = true;
        processando = true;
    }
    interrupts();

    if (tem){
        executarComando(acao);
        processando = false;
    }

    // seguranca local
    vigiarLado(motorX,  Xstart,  Xend);
    vigiarLado(motor2X, X2start, X2end);

    // watchdog do mestre
    if (!emergencia){
        noInterrupts();
        unsigned long t = ultimoContato;
        interrupts();
        if (t != 0 && algumMotorMovendo() && (millis() - t) > TIMEOUT_MESTRE){
            entrarEmergencia();
        }
    }

    motorX.run();
    motor2X.run();
}
