/*
  ============================================================
   ARDUINO Y - COMANDO CENTRAL (MESTRE) - v.4 (revisado)
  ============================================================
  Mestre do barramento I2C. Guarda a maquina de estados, recebe
  comandos pela Serial e delega ordens aos escravos:
    - Arduino X (0x08): garra alinhadora
    - Arduino Z (0x09): elevador + garra da bateria + extensao

  Controla DIRETAMENTE: motores Y1/Y2 (puxadores), Y3/Y4
  (empurradores), sensor de pouso e fins de curso do eixo Y.

  Comandos Serial: ATV, ZR, EMR, ESC, RST
  Em ZR:           ZU, ZX, ZY, ZZ, ZPI

  MUDANCAS EM RELACAO A v.3
   - Comandos I2C enviados UMA vez por etapa (nao a cada loop).
   - Cada etapa espera o mecanismo terminar (status MOVENDO) em
     vez de assumir que terminou. TROCA_BATERIA virou uma
     sequencia passo a passo.
   - Timeout em todas as etapas -> emergencia.
   - Fins de curso do Y param os motores na hora (local).
   - Deteccao de falha de comunicacao I2C -> emergencia.
   - Emergencia real: para na hora, corta o enable (opcional) e
     fica travada ate o comando RST.
   - Leitura da Serial sem bloqueio (readStringUntil bloqueava
     ate 1s e travava os motores).
   - ESC durante uma operacao agora PARA os motores.

  O protocolo abaixo precisa ser IDENTICO nos tres codigos.
  Ligacao: SDA(A4)-SDA, SCL(A5)-SCL, GND comum e pull-ups
  externos de ~4,7k em SDA e SCL.
  ============================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

//====================== PROTOCOLO I2C ==========================
#define X_ADDR 0x08
#define Z_ADDR 0x09

const byte MEC_TODOS     = 0x00;   // comandos que valem para a placa toda
const byte MEC_ELEVADOR  = 0x01;
const byte MEC_GARRA     = 0x02;
const byte MEC_EXTENSAO  = 0x03;

const byte ACAO_MOVER       = 0x01;
const byte ACAO_RETORNAR    = 0x02;
const byte ACAO_PARAR       = 0x03;  // parada suave (desacelera)
const byte ACAO_ZERAR       = 0x04;
const byte ACAO_EMERGENCIA  = 0x05;  // para na hora (+ corta enable, se configurado)
const byte ACAO_HABILITAR   = 0x06;  // sai da emergencia

// Status do arduino Z (um byte por mecanismo)
const byte STATUS_INICIO      = 0x01;
const byte STATUS_FIM         = 0x02;
const byte STATUS_MOVENDO     = 0x04;
const byte STATUS_EMERGENCIA  = 0x08;

// Status do arduino X
const byte STATUS_X1_INICIO     = 0x01;
const byte STATUS_X1_FIM        = 0x02;
const byte STATUS_X2_INICIO     = 0x04;
const byte STATUS_X2_FIM        = 0x08;
const byte STATUS_X_MOVENDO     = 0x10;
const byte STATUS_X_EMERGENCIA  = 0x20;
//================================================================

// Puxadores do drone
const int Y1_STEP = 2;
const int Y1_DIR  = 3;
const int Y2_STEP = 4;
const int Y2_DIR  = 5;

// Empurradores
const int Y3_STEP = 6;
const int Y3_DIR  = 7;
const int Y4_STEP = 8;
const int Y4_DIR  = 9;

// SENSORES
const int Ystart    = A0;
const int Yend      = A1;
const int YstartEmp = A2;
const int sensorPouso = A3;   // LOW = drone pousou. Precisa de pull-up/pull-down fisico!

// Botoes (desabilitados por enquanto, como no original)
const int BotaoStart = 10;
const int BotaoStop  = 11;
const int BotaoReset = 12;

const int pinoEnable = 13;    // Enable do driver (ativo em LOW)

// Direcao dos motores. O "2o" de cada par e espelhado.
// Se algum motor girar ao contrario, troque o valor aqui.
const bool INV_DIR_Y1 = false;
const bool INV_DIR_Y2 = true;
const bool INV_DIR_Y3 = false;
const bool INV_DIR_Y4 = true;

// Na emergencia, corta o enable dos drivers deste arduino?
const bool DESLIGAR_DRIVER_NA_EMERGENCIA = true;

AccelStepper motorY(AccelStepper::DRIVER, Y1_STEP, Y1_DIR);
AccelStepper motor2Y(AccelStepper::DRIVER, Y2_STEP, Y2_DIR);
AccelStepper motorYEmpurrao(AccelStepper::DRIVER, Y3_STEP, Y3_DIR);
AccelStepper motor2YEmpurrao(AccelStepper::DRIVER, Y4_STEP, Y4_DIR);

//------------------------ CONFIGURACOES ----------------------------------
long passosY = 1600;
long passosYempurrar = 1600;

const float VEL_MAX = 800.0;
const float ACEL = 200.0;   // OBS: com ACEL baixa o motor precisa de muitos passos para frear

const unsigned long INTERVALO_I2C = 50;          // ms entre consultas de status
const byte MAX_FALHAS_I2C = 5;                   // consultas seguidas sem resposta

const unsigned long ESPERA_INICIO_TROCA = 1000;  // ms parado no topo antes da troca
const unsigned long ESPERA_EXPANSAO     = 10000; // ms antes de liberar o drone

// Timeouts (ms) - ajuste conforme a mecanica real
const unsigned long TO_HUNT        = 30000;
const unsigned long TO_Z           = 30000;
const unsigned long TO_PASSO_TROCA = 20000;
const unsigned long TO_RETORNO_Y   = 30000;
const unsigned long TO_EXPANSAO    = 30000;

//------------------------ ESTADO ------------------------------------------
enum EstadoAtualMotores{
    STAND_BY,
    ZERAMENTO,
    ATERRISSAGEM,
    HUNT,
    MOVENDO_Z,
    TROCA_BATERIA,
    RETORNO_Z,
    RETORNO_Y,
    EXPANSAO_X,
    STOP
};

EstadoAtualMotores estadoatual = STAND_BY;

unsigned long entradaEstado = 0;   // quando entrou no estado atual
bool primeiraVez = true;           // true na 1a passada de cada estado
bool emergenciaAtiva = false;

// controle do I2C
unsigned long ultimaConsultaI2C = 0;
unsigned long tComando = 0;        // instante do ultimo comando enviado
unsigned long tStatus = 0;         // instante da ultima leitura de status completa
byte falhasI2C = 0;
bool avisouI2C = false;

byte statusX         = 0;
byte statusZElevador = 0;
byte statusZGarra    = 0;
byte statusZExtensao = 0;

// sequencia de troca
byte passoTroca = 0;
bool passoEnviado = false;
unsigned long tPasso = 0;

String bufLinha = "";

//--------------------------------------------------------------
// SEQUENCIA DA TROCA DE BATERIA (mesma ordem do codigo original).
// Cada passo so avanca quando o mecanismo avisa que terminou.
// REVISAR com a mecanica: por ex., pegar a bateria carregada pode
// exigir outra posicao da extensao/elevador.
struct PassoTroca { byte mec; byte acao; };

const PassoTroca SEQ_TROCA[] = {
    { MEC_GARRA,    ACAO_RETORNAR },  //  1 garante que a garra esta aberta
    { MEC_EXTENSAO, ACAO_MOVER    },  //  2 extruda a garra Z
    { MEC_GARRA,    ACAO_MOVER    },  //  3 pega a bateria
    { MEC_EXTENSAO, ACAO_RETORNAR },  //  4 contrai a garra Z
    { MEC_ELEVADOR, ACAO_RETORNAR },  //  5 desce pro armazem
    { MEC_EXTENSAO, ACAO_MOVER    },  //  6 extruda
    { MEC_GARRA,    ACAO_RETORNAR },  //  7 solta a bateria velha
    { MEC_GARRA,    ACAO_MOVER    },  //  8 pega a bateria carregada
    { MEC_EXTENSAO, ACAO_RETORNAR },  //  9 contrai
    { MEC_ELEVADOR, ACAO_MOVER    },  // 10 sobe pro drone
    { MEC_EXTENSAO, ACAO_MOVER    },  // 11 extruda
    { MEC_GARRA,    ACAO_RETORNAR },  // 12 encaixa a bateria carregada
    { MEC_EXTENSAO, ACAO_RETORNAR }   // 13 contrai
};
const byte N_PASSOS_TROCA = sizeof(SEQ_TROCA) / sizeof(SEQ_TROCA[0]);

//--------------------------------------------------------------
// UTILIDADES DE ESTADO
void mudarEstado(EstadoAtualMotores novo){
    estadoatual = novo;
    entradaEstado = millis();
    primeiraVez = true;
}

bool estourouTempo(unsigned long ms){
    return (millis() - entradaEstado) > ms;
}

bool i2cOK(){
    return falhasI2C < MAX_FALHAS_I2C;
}

// true se o status lido e posterior ao ultimo comando enviado
bool statusFresco(){
    return tStatus > tComando;
}

//--------------------------------------------------------------
// COMUNICACAO COM OS ESCRAVOS
bool enviarBytes(byte addr, const byte *dados, byte n){
    for (byte t = 0; t < 3; t++){                 // ate 3 tentativas
        Wire.beginTransmission(addr);
        Wire.write(dados, n);
        if (Wire.endTransmission() == 0) return true;
    }
    if (falhasI2C < 255) falhasI2C++;
    return false;
}

bool enviarComandoX(byte acao){
    byte d[1] = { acao };
    bool ok = enviarBytes(X_ADDR, d, 1);
    tComando = millis();
    return ok;
}

bool enviarComandoZ(byte mecanismo, byte acao){
    byte d[2] = { mecanismo, acao };
    bool ok = enviarBytes(Z_ADDR, d, 2);
    tComando = millis();
    return ok;
}

bool lerStatusX(){
    if (Wire.requestFrom((uint8_t)X_ADDR, (uint8_t)1) == 1){
        statusX = Wire.read();
        return true;
    }
    while (Wire.available()) Wire.read();
    return false;
}

bool lerStatusZ(){
    if (Wire.requestFrom((uint8_t)Z_ADDR, (uint8_t)3) == 3){
        statusZElevador = Wire.read();
        statusZGarra    = Wire.read();
        statusZExtensao = Wire.read();
        return true;
    }
    while (Wire.available()) Wire.read();
    return false;
}

void atualizarStatusEscravos(){
    if (millis() - ultimaConsultaI2C < INTERVALO_I2C) return;
    ultimaConsultaI2C = millis();

    bool okX = lerStatusX();
    bool okZ = lerStatusZ();

    if (okX && okZ){
        falhasI2C = 0;
        tStatus = millis();
    }else if (falhasI2C < 255){
        falhasI2C++;
    }
}

byte statusDoMecanismo(byte mec){
    switch (mec){
        case MEC_ELEVADOR: return statusZElevador;
        case MEC_GARRA:    return statusZGarra;
        case MEC_EXTENSAO: return statusZExtensao;
    }
    return 0;
}

//--------------------------------------------------------------
// MOTORES LOCAIS
void pararImediato(AccelStepper &m){
    m.setCurrentPosition(m.currentPosition());   // alvo = posicao atual, velocidade 0
}

void pararSuave(AccelStepper &m){
    if (m.speed() == 0.0) m.moveTo(m.currentPosition());
    else m.stop();                                // desacelera ate parar
}

void pararImediatoLocal(){
    pararImediato(motorY);
    pararImediato(motor2Y);
    pararImediato(motorYEmpurrao);
    pararImediato(motor2YEmpurrao);
}

void pararYsuave(){
    pararSuave(motorY);
    pararSuave(motor2Y);
}

void pararYempurraSuave(){
    pararSuave(motorYEmpurrao);
    pararSuave(motor2YEmpurrao);
}

bool motoresYParados(){
    return motorY.distanceToGo() == 0 && motor2Y.distanceToGo() == 0;
}

bool empurradoresParados(){
    return motorYEmpurrao.distanceToGo() == 0 && motor2YEmpurrao.distanceToGo() == 0;
}

void moverY(){
    motorY.moveTo(passosY);
    motor2Y.moveTo(passosY);
}

void ZERO_Y(){
    motorY.setCurrentPosition(0);
    motorYEmpurrao.setCurrentPosition(0);
    motor2Y.setCurrentPosition(0);
    motor2YEmpurrao.setCurrentPosition(0);
}

// Fins de curso do Y: paragem imediata, feita a cada volta do loop.
// So olha o sensor do lado para onde o motor esta indo.
void vigiarFinsDeCursoY(){
    long d = motorY.distanceToGo();
    if (d == 0) d = motor2Y.distanceToGo();

    if (d > 0 && digitalRead(Yend) == LOW){
        pararImediato(motorY);
        pararImediato(motor2Y);
    }else if (d < 0 && digitalRead(Ystart) == LOW){
        motorY.setCurrentPosition(0);            // aproveita para re-zerar
        motor2Y.setCurrentPosition(0);
    }

    long e = motorYEmpurrao.distanceToGo();
    if (e == 0) e = motor2YEmpurrao.distanceToGo();

    if (e < 0 && digitalRead(YstartEmp) == LOW){
        motorYEmpurrao.setCurrentPosition(0);
        motor2YEmpurrao.setCurrentPosition(0);
    }
}

//--------------------------------------------------------------
// PARADAS
void pararTudoSuave(){
    pararYsuave();
    pararYempurraSuave();
    enviarComandoX(ACAO_PARAR);
    enviarComandoZ(MEC_ELEVADOR, ACAO_PARAR);
    enviarComandoZ(MEC_GARRA, ACAO_PARAR);
    enviarComandoZ(MEC_EXTENSAO, ACAO_PARAR);
}

void entrarEmergencia(){
    emergenciaAtiva = true;
    pararImediatoLocal();
    if (DESLIGAR_DRIVER_NA_EMERGENCIA) digitalWrite(pinoEnable, HIGH);

    enviarComandoX(ACAO_EMERGENCIA);
    enviarComandoZ(MEC_TODOS, ACAO_EMERGENCIA);

    mudarEstado(STAND_BY);
    Serial.println(F("Parada EMER"));
    Serial.println(F(" -> standing by (use RST para reabilitar)"));
}

void falhaEmergencia(const __FlashStringHelper *motivo){
    Serial.print(F("FALHA: "));
    Serial.println(motivo);
    entrarEmergencia();
}

void reabilitar(){
    if (!emergenciaAtiva){
        Serial.println(F(" | NADA A REABILITAR       |"));
        return;
    }
    digitalWrite(pinoEnable, LOW);
    enviarComandoX(ACAO_HABILITAR);
    enviarComandoZ(MEC_TODOS, ACAO_HABILITAR);
    delay(20);                                   // da tempo dos escravos processarem

    ultimaConsultaI2C = 0;
    atualizarStatusEscravos();

    if (falhasI2C == 0 && !(statusX & STATUS_X_EMERGENCIA) && !(statusZElevador & STATUS_EMERGENCIA)){
        emergenciaAtiva = false;
        Serial.println(F(" | REABILITADO. standing by |"));
    }else{
        if (DESLIGAR_DRIVER_NA_EMERGENCIA) digitalWrite(pinoEnable, HIGH);
        Serial.println(F(" | FALHA AO REABILITAR     |"));
    }
}

void homing_U(){   // reset universal (volta tudo para o zero)
    motorY.moveTo(0);
    motor2Y.moveTo(0);
    motorYEmpurrao.moveTo(0);
    motor2YEmpurrao.moveTo(0);

    enviarComandoX(ACAO_RETORNAR);
    enviarComandoZ(MEC_ELEVADOR, ACAO_RETORNAR);
    enviarComandoZ(MEC_GARRA, ACAO_RETORNAR);
    enviarComandoZ(MEC_EXTENSAO, ACAO_RETORNAR);
}

//--------------------------------------------------------------
// SERIAL (sem bloqueio)
bool lerLinhaSerial(String &saida){
    while (Serial.available()){
        char c = Serial.read();
        if (c == '\n' || c == '\r'){
            if (bufLinha.length() > 0){
                saida = bufLinha;
                bufLinha = "";
                return true;
            }
        }else if (bufLinha.length() < 20){
            bufLinha += c;
        }
    }
    return false;
}

void tratarComando(String c){
    c.trim();
    c.toLowerCase();

    if (c == "atv"){
        if (emergenciaAtiva)                 Serial.println(F(" | EM EMERGENCIA: USE RST   |"));
        else if (!i2cOK())                   Serial.println(F(" | SEM COMUNICACAO I2C     |"));
        else if (estadoatual == STAND_BY){
            Serial.println(F(" | ESPERANDO POUSO...      |"));
            mudarEstado(ATERRISSAGEM);
        }else                                Serial.println(F(" | JA EM BUSCA DO Hy-D-J   |"));

    }else if (c == "zr"){
        if (emergenciaAtiva)                 Serial.println(F(" | EM EMERGENCIA: USE RST   |"));
        else if (!i2cOK())                   Serial.println(F(" | SEM COMUNICACAO I2C     |"));
        else if (estadoatual == STAND_BY){
            Serial.println(F(" | ENTROU NO ZERENCIAMENTO |"));
            mudarEstado(ZERAMENTO);
        }else                                Serial.println(F(" | NAO EH POSSIVEL AGORA   |"));

    }else if (c == "emr"){
        entrarEmergencia();

    }else if (c == "rst"){
        reabilitar();

    }else if (c == "esc"){
        if (estadoatual == STAND_BY){
            Serial.println(F(" | JA ESTA EM STAND BY     |"));
        }else{
            if (estadoatual == ZERAMENTO) Serial.println(F(" | SAIU   DO ZERENCIAMENTO |"));
            pararTudoSuave();
            mudarEstado(STAND_BY);
            Serial.println(F("esc -> standing by"));
        }

    }else if (estadoatual == ZERAMENTO && c == "zu"){
        Serial.println(F("zerando eixos..."));
        homing_U();

    }else if (estadoatual == ZERAMENTO && c == "zx"){
        enviarComandoX(ACAO_RETORNAR);

    }else if (estadoatual == ZERAMENTO && c == "zy"){
        motorY.moveTo(0);
        motor2Y.moveTo(0);

    }else if (estadoatual == ZERAMENTO && c == "zz"){
        enviarComandoZ(MEC_ELEVADOR, ACAO_RETORNAR);

    }else if (estadoatual == ZERAMENTO && c == "zpi"){   // define o zero em TODOS os eixos
        ZERO_Y();
        enviarComandoX(ACAO_ZERAR);
        enviarComandoZ(MEC_ELEVADOR, ACAO_ZERAR);
        enviarComandoZ(MEC_GARRA, ACAO_ZERAR);
        enviarComandoZ(MEC_EXTENSAO, ACAO_ZERAR);
        Serial.println(F("zero definido em todos os eixos"));

    }else{
        Serial.println(F(" | comando desconhecido    |"));
    }
}

//--------------------------------------------------------------
void setup(){

    pinMode(pinoEnable, OUTPUT);
    digitalWrite(pinoEnable, LOW);

    pinMode(BotaoStart, INPUT_PULLUP);
    pinMode(BotaoStop, INPUT_PULLUP);
    pinMode(BotaoReset, INPUT_PULLUP);

    pinMode(Ystart, INPUT_PULLUP);
    pinMode(Yend, INPUT_PULLUP);
    pinMode(YstartEmp, INPUT_PULLUP);
    pinMode(sensorPouso, INPUT);

    AccelStepper *motores[4] = { &motorY, &motor2Y, &motorYEmpurrao, &motor2YEmpurrao };
    for (byte i = 0; i < 4; i++){
        motores[i]->setMaxSpeed(VEL_MAX);
        motores[i]->setAcceleration(ACEL);
        motores[i]->setMinPulseWidth(5);
    }

    // setPinsInverted(direcao, step, enable)
    motorY.setPinsInverted(INV_DIR_Y1, false, false);
    motor2Y.setPinsInverted(INV_DIR_Y2, false, false);
    motorYEmpurrao.setPinsInverted(INV_DIR_Y3, false, false);
    motor2YEmpurrao.setPinsInverted(INV_DIR_Y4, false, false);

    Serial.begin(9600);

    Wire.begin();                 // MESTRE do barramento
#ifdef WIRE_HAS_TIMEOUT
    Wire.setWireTimeout(3000, true);   // nao trava se um escravo cair
#endif

    Serial.println(F("ATV, ZR, EMR, ESC, RST"));
    Serial.println(F("(em ZR) zpi, zu, zx, zy, zz"));

    entradaEstado = millis();
}

//--------------------------------------------------------------
void loop(){

    String linha;
    if (lerLinhaSerial(linha)) tratarComando(linha);

    atualizarStatusEscravos();

    // ---- supervisao ----
    if (!emergenciaAtiva && ((statusX & STATUS_X_EMERGENCIA) || (statusZElevador & STATUS_EMERGENCIA))){
        Serial.println(F("Escravo reportou emergencia"));
        entrarEmergencia();
    }

    bool ativo = (estadoatual >= HUNT && estadoatual <= EXPANSAO_X);
    if (!i2cOK()){
        if (ativo && !emergenciaAtiva) falhaEmergencia(F("sem resposta I2C"));
        if (!avisouI2C){
            Serial.println(F("AVISO: sem resposta I2C de X ou Z"));
            avisouI2C = true;
        }
    }else{
        avisouI2C = false;
    }

    // ---- maquina de estados ----
    switch (estadoatual){

        case STAND_BY:
            break;

        case ZERAMENTO:
            if (primeiraVez){
                Serial.println(F("em zerenciamento"));
                primeiraVez = false;
            }
            break;

        case ATERRISSAGEM:
            if (primeiraVez){
                Serial.println(F("Aguardando pouso..."));
                primeiraVez = false;
            }
            if (digitalRead(sensorPouso) == LOW){
                Serial.println(F("Drone pousou"));
                mudarEstado(HUNT);
            }
            break;

        case HUNT:   // puxa o drone (Y) e fecha a garra (X) ao mesmo tempo
            if (primeiraVez){
                Serial.println(F("Hunting..."));
                moverY();
                enviarComandoX(ACAO_MOVER);
                primeiraVez = false;
            }
            if (estourouTempo(TO_HUNT)){ falhaEmergencia(F("timeout no HUNT")); break; }

            if (motoresYParados() && statusFresco() && !(statusX & STATUS_X_MOVENDO)){
                if (!((statusX & STATUS_X1_FIM) && (statusX & STATUS_X2_FIM)))
                    Serial.println(F("AVISO: garra X chegou ao alvo sem acionar fim de curso"));
                mudarEstado(MOVENDO_Z);
            }
            break;

        case MOVENDO_Z:
            if (primeiraVez){
                Serial.println(F("Subindo elevador"));
                enviarComandoZ(MEC_ELEVADOR, ACAO_MOVER);
                primeiraVez = false;
            }
            if (estourouTempo(TO_Z)){ falhaEmergencia(F("timeout subindo elevador")); break; }

            if (statusFresco() && !(statusZElevador & STATUS_MOVENDO)){
                if (!(statusZElevador & STATUS_FIM))
                    Serial.println(F("AVISO: elevador no alvo sem acionar fim de curso"));
                passoTroca = 0;
                passoEnviado = false;
                mudarEstado(TROCA_BATERIA);
            }
            break;

        case TROCA_BATERIA:
            if (millis() - entradaEstado < ESPERA_INICIO_TROCA) break;

            if (!passoEnviado){
                Serial.print(F("Troca de bateria: passo "));
                Serial.print(passoTroca + 1);
                Serial.print(F("/"));
                Serial.println(N_PASSOS_TROCA);
                enviarComandoZ(SEQ_TROCA[passoTroca].mec, SEQ_TROCA[passoTroca].acao);
                tPasso = millis();
                passoEnviado = true;
            }else{
                if (millis() - tPasso > TO_PASSO_TROCA){ falhaEmergencia(F("timeout em passo da troca")); break; }

                if (statusFresco() && !(statusDoMecanismo(SEQ_TROCA[passoTroca].mec) & STATUS_MOVENDO)){
                    passoTroca++;
                    passoEnviado = false;
                    if (passoTroca >= N_PASSOS_TROCA){
                        Serial.println(F("Troca concluida"));
                        mudarEstado(RETORNO_Z);
                    }
                }
            }
            break;

        case RETORNO_Z:
            if (primeiraVez){
                Serial.println(F("Descendo elevador"));
                enviarComandoZ(MEC_ELEVADOR, ACAO_RETORNAR);
                primeiraVez = false;
            }
            if (estourouTempo(TO_Z)){ falhaEmergencia(F("timeout descendo elevador")); break; }

            if (statusFresco() && !(statusZElevador & STATUS_MOVENDO)){
                if (!(statusZElevador & STATUS_INICIO))
                    Serial.println(F("AVISO: elevador no zero sem acionar fim de curso"));
                mudarEstado(RETORNO_Y);
            }
            break;

        case RETORNO_Y:   // volta o drone arrastando-o ate a posicao de lancamento
            if (primeiraVez){
                Serial.println(F("Retornando Y"));
                motorY.moveTo(0);
                motor2Y.moveTo(0);
                motorYEmpurrao.moveTo(passosYempurrar);
                motor2YEmpurrao.moveTo(passosYempurrar);
                primeiraVez = false;
            }
            if (estourouTempo(TO_RETORNO_Y)){ falhaEmergencia(F("timeout em RETORNO_Y")); break; }

            if (motoresYParados()){
                if (digitalRead(Ystart) == LOW){
                    pararYempurraSuave();
                    mudarEstado(EXPANSAO_X);
                }else{
                    falhaEmergencia(F("Y chegou ao alvo sem acionar fim de curso inicial"));
                }
            }
            break;

        case EXPANSAO_X:   // libera o drone lateralmente
            if (millis() - entradaEstado < ESPERA_EXPANSAO) break;

            if (primeiraVez){
                Serial.println(F("Liberando drone"));
                enviarComandoX(ACAO_RETORNAR);
                motorYEmpurrao.moveTo(0);
                motor2YEmpurrao.moveTo(0);
                primeiraVez = false;
            }
            if ((millis() - tComando) > TO_EXPANSAO){ falhaEmergencia(F("timeout em EXPANSAO_X")); break; }

            if (empurradoresParados() && statusFresco() && !(statusX & STATUS_X_MOVENDO)){
                if (!((statusX & STATUS_X1_INICIO) && (statusX & STATUS_X2_INICIO)))
                    Serial.println(F("AVISO: garra X no zero sem acionar fim de curso"));
                if (digitalRead(YstartEmp) != LOW)
                    Serial.println(F("AVISO: empurradores no zero sem acionar fim de curso"));
                mudarEstado(STOP);
            }
            break;

        case STOP:
            pararTudoSuave();
            Serial.println(F("Parada normal"));
            Serial.println(F(" -> standing by"));
            mudarEstado(STAND_BY);
            break;
    }

    vigiarFinsDeCursoY();

    motorY.run();
    motor2Y.run();
    motorYEmpurrao.run();
    motor2YEmpurrao.run();
}
