/*
  ============================================================
   ARDUINO Z - ELEVADOR + GARRA (ESCRAVO) - v.4 (revisado)
  ============================================================
  Escravo I2C no endereco 0x09. Controla 3 mecanismos:
    - Elevador (motorZ / motor2Z, dois motores espelhados)
    - Garra que segura a bateria (motorZgarra)
    - Extensao/avanco da garra (motorZYgarra)

  Cada comando chega com 2 bytes: [mecanismo, acao].
  O status devolve 3 bytes: elevador, garra, extensao.

  MUDANCAS EM RELACAO A v.3
   - Fila de comandos: antes, so o ultimo comando recebido
     sobrevivia e a leitura mecanismo/acao podia ser misturada.
   - Fins de curso param o motor NA HORA, localmente (e re-zeram
     a posicao no sensor de inicio).
   - Comando pendente conta como MOVENDO no status.
   - ACAO_EMERGENCIA / ACAO_HABILITAR.
   - Watchdog: mestre mudo com motor andando -> emergencia.
   - ACAO_PARAR agora e parada suave (stop()).

  ATENCAO (elevador vertical): se DESLIGAR_DRIVER_NA_EMERGENCIA
  for true, o elevador perde o torque de seguranca e pode CAIR
  com a bateria se o fuso nao for autotravante. Por padrao deixei
  false aqui: o motor para na hora, mas continua energizado.

  O protocolo abaixo precisa ser IDENTICO nos tres codigos.
  ============================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

#define Z_ADDR 0x09

//====================== PROTOCOLO I2C ==========================
const byte MEC_TODOS     = 0x00;
const byte MEC_ELEVADOR  = 0x01;
const byte MEC_GARRA     = 0x02;
const byte MEC_EXTENSAO  = 0x03;

const byte ACAO_MOVER       = 0x01;
const byte ACAO_RETORNAR    = 0x02;
const byte ACAO_PARAR       = 0x03;
const byte ACAO_ZERAR       = 0x04;
const byte ACAO_EMERGENCIA  = 0x05;
const byte ACAO_HABILITAR   = 0x06;

const byte STATUS_INICIO      = 0x01;
const byte STATUS_FIM         = 0x02;
const byte STATUS_MOVENDO     = 0x04;
const byte STATUS_EMERGENCIA  = 0x08;
//================================================================

// Z Elevador da bateria
const int Z1_STEP = 2;
const int Z1_DIR  = 3;
const int Z2_STEP = 4;
const int Z2_DIR  = 5;

// Garra que segura a bateria (abre/fecha)
const int Zgarra_STEP = 6;
const int Zgarra_DIR  = 7;

// Extensao (avanco/recuo) da garra Z
const int ZY_STEP = 8;
const int ZY_DIR  = 9;

// SENSORES
const int Zstart   = A0;
const int Zend     = A1;
const int ZGstart  = A2;
const int ZGend    = A3;
const int ZExstart = 10;
const int ZExend   = 11;

const int pinoEnable = 12;     // Enable do driver (ativo em LOW)

// Se algum motor girar ao contrario, troque aqui
const bool INV_DIR_Z1    = false;
const bool INV_DIR_Z2    = true;    // 2o motor do elevador espelhado
const bool INV_DIR_GARRA = false;
const bool INV_DIR_EXT   = false;

// Corta o enable na emergencia? (ver aviso no cabecalho)
const bool DESLIGAR_DRIVER_NA_EMERGENCIA = false;

AccelStepper motorZ(AccelStepper::DRIVER, Z1_STEP, Z1_DIR);
AccelStepper motor2Z(AccelStepper::DRIVER, Z2_STEP, Z2_DIR);
AccelStepper motorZgarra(AccelStepper::DRIVER, Zgarra_STEP, Zgarra_DIR);
AccelStepper motorZYgarra(AccelStepper::DRIVER, ZY_STEP, ZY_DIR);

long passosZ = 1600;            // altura onde se troca a bateria do drone
long passosZgarra = 800;        // fechamento da garra da bateria
long passosZYgarra = 800;       // extensao/avanco da garra

const float VEL_MAX = 800.0;
const float ACEL = 200.0;

const unsigned long TIMEOUT_MESTRE = 1500;   // ms sem contato com motor andando

// Fila de comandos (escrita na interrupcao, lida no loop)
const byte TAM_FILA = 8;
volatile byte filaMec[TAM_FILA];
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

bool elevadorMovendo(){ return motorZ.distanceToGo() != 0 || motor2Z.distanceToGo() != 0; }
bool garraMovendo()   { return motorZgarra.distanceToGo() != 0; }
bool extensaoMovendo(){ return motorZYgarra.distanceToGo() != 0; }

bool algumMotorMovendo(){
    return elevadorMovendo() || garraMovendo() || extensaoMovendo();
}

// Fins de curso de um mecanismo (1 ou 2 motores com o mesmo par de sensores).
// Para mecanismo de 1 motor, passe o mesmo motor duas vezes.
// So olha o sensor do lado para onde o motor anda.
void vigiarGrupo(AccelStepper &a, AccelStepper &b, int pIni, int pFim){
    long d = a.distanceToGo();
    if (d == 0) d = b.distanceToGo();

    if (d > 0 && digitalRead(pFim) == LOW){
        pararImediato(a);
        pararImediato(b);
    }else if (d < 0 && digitalRead(pIni) == LOW){
        a.setCurrentPosition(0);            // re-zera no sensor de inicio
        b.setCurrentPosition(0);
    }
}

void entrarEmergencia(){
    pararImediato(motorZ);
    pararImediato(motor2Z);
    pararImediato(motorZgarra);
    pararImediato(motorZYgarra);
    if (DESLIGAR_DRIVER_NA_EMERGENCIA) digitalWrite(pinoEnable, HIGH);
    emergencia = true;
}

void executarComando(byte mecanismo, byte acao){

    if (acao == ACAO_HABILITAR){
        digitalWrite(pinoEnable, LOW);
        emergencia = false;
        return;
    }
    if (emergencia && (acao == ACAO_MOVER || acao == ACAO_RETORNAR)) return;

    switch (mecanismo){

        case MEC_ELEVADOR:
            switch (acao){
                case ACAO_MOVER:    motorZ.moveTo(passosZ); motor2Z.moveTo(passosZ); break;
                case ACAO_RETORNAR: motorZ.moveTo(0); motor2Z.moveTo(0); break;
                case ACAO_PARAR:    pararSuave(motorZ); pararSuave(motor2Z); break;
                case ACAO_ZERAR:    motorZ.setCurrentPosition(0); motor2Z.setCurrentPosition(0); break;
            }
            break;

        case MEC_GARRA:
            switch (acao){
                case ACAO_MOVER:    motorZgarra.moveTo(passosZgarra); break;   // fecha (pega a bateria)
                case ACAO_RETORNAR: motorZgarra.moveTo(0); break;              // abre
                case ACAO_PARAR:    pararSuave(motorZgarra); break;
                case ACAO_ZERAR:    motorZgarra.setCurrentPosition(0); break;
            }
            break;

        case MEC_EXTENSAO:
            switch (acao){
                case ACAO_MOVER:    motorZYgarra.moveTo(passosZYgarra); break; // extruda a garra
                case ACAO_RETORNAR: motorZYgarra.moveTo(0); break;             // recolhe a garra
                case ACAO_PARAR:    pararSuave(motorZYgarra); break;
                case ACAO_ZERAR:    motorZYgarra.setCurrentPosition(0); break;
            }
            break;
    }
}

//--------------------------------------------------------------
// Interrupcao I2C: curta de proposito.
void receberComando(int numBytes){
    ultimoContato = millis();
    while (numBytes >= 2 && Wire.available() >= 2){
        byte mec  = Wire.read();
        byte acao = Wire.read();
        numBytes -= 2;

        if (acao == ACAO_EMERGENCIA){
            pedidoEmergencia = true;          // tratado de imediato no loop
        }else{
            byte prox = (filaFim + 1) % TAM_FILA;
            if (prox != filaIni){             // fila cheia = descarta
                filaMec[filaFim]  = mec;
                filaAcao[filaFim] = acao;
                filaFim = prox;
            }
        }
    }
    while (Wire.available()) Wire.read();
}

void enviarStatus(){
    ultimoContato = millis();

    bool pendente = (filaIni != filaFim) || processando;
    byte base = 0;
    if (pendente)   base |= STATUS_MOVENDO;
    if (emergencia) base |= STATUS_EMERGENCIA;

    byte statusElevador = base;
    if (digitalRead(Zstart) == LOW) statusElevador |= STATUS_INICIO;
    if (digitalRead(Zend) == LOW)   statusElevador |= STATUS_FIM;
    if (elevadorMovendo())          statusElevador |= STATUS_MOVENDO;

    byte statusGarra = base;
    if (digitalRead(ZGstart) == LOW) statusGarra |= STATUS_INICIO;
    if (digitalRead(ZGend) == LOW)   statusGarra |= STATUS_FIM;
    if (garraMovendo())              statusGarra |= STATUS_MOVENDO;

    byte statusExtensao = base;
    if (digitalRead(ZExstart) == LOW) statusExtensao |= STATUS_INICIO;
    if (digitalRead(ZExend) == LOW)   statusExtensao |= STATUS_FIM;
    if (extensaoMovendo())            statusExtensao |= STATUS_MOVENDO;

    byte resp[3] = { statusElevador, statusGarra, statusExtensao };
    Wire.write(resp, 3);
}

//--------------------------------------------------------------
void setup(){

    pinMode(pinoEnable, OUTPUT);
    digitalWrite(pinoEnable, LOW);

    pinMode(Zstart, INPUT_PULLUP);
    pinMode(Zend, INPUT_PULLUP);
    pinMode(ZGstart, INPUT_PULLUP);
    pinMode(ZGend, INPUT_PULLUP);
    pinMode(ZExstart, INPUT_PULLUP);
    pinMode(ZExend, INPUT_PULLUP);

    AccelStepper *motores[4] = { &motorZ, &motor2Z, &motorZgarra, &motorZYgarra };
    for (byte i = 0; i < 4; i++){
        motores[i]->setMaxSpeed(VEL_MAX);
        motores[i]->setAcceleration(ACEL);
        motores[i]->setMinPulseWidth(5);
    }

    // setPinsInverted(direcao, step, enable)
    motorZ.setPinsInverted(INV_DIR_Z1, false, false);
    motor2Z.setPinsInverted(INV_DIR_Z2, false, false);
    motorZgarra.setPinsInverted(INV_DIR_GARRA, false, false);
    motorZYgarra.setPinsInverted(INV_DIR_EXT, false, false);

    Serial.begin(9600);   // opcional, so debug local
    Serial.println("Arduino Z pronto (escravo 0x09)");

    Wire.begin(Z_ADDR);   // por ultimo: so atende o mestre com tudo configurado
    Wire.onReceive(receberComando);
    Wire.onRequest(enviarStatus);
}

void loop(){

    if (pedidoEmergencia){
        pedidoEmergencia = false;
        entrarEmergencia();
    }

    // tira UM comando da fila (com interrupcoes desligadas)
    byte mec = 0, acao = 0;
    bool tem = false;
    noInterrupts();
    if (filaIni != filaFim){
        mec  = filaMec[filaIni];
        acao = filaAcao[filaIni];
        filaIni = (filaIni + 1) % TAM_FILA;
        tem = true;
        processando = true;
    }
    interrupts();

    if (tem){
        executarComando(mec, acao);
        processando = false;
    }

    // seguranca local
    vigiarGrupo(motorZ, motor2Z, Zstart, Zend);
    vigiarGrupo(motorZgarra, motorZgarra, ZGstart, ZGend);
    vigiarGrupo(motorZYgarra, motorZYgarra, ZExstart, ZExend);

    // watchdog do mestre
    if (!emergencia){
        noInterrupts();
        unsigned long t = ultimoContato;
        interrupts();
        if (t != 0 && algumMotorMovendo() && (millis() - t) > TIMEOUT_MESTRE){
            entrarEmergencia();
        }
    }

    motorZ.run();
    motor2Z.run();
    motorZgarra.run();
    motorZYgarra.run();
}
