/*
  ============================================================
   ARDUINO Y - COMANDO CENTRAL (MESTRE) - BASEADO NA v.3
  ============================================================
  Este e o arduino "mestre": guarda toda a maquina de estados,
  recebe os comandos via Serial e comanda os arduinos X e Z
  atraves do barramento I2C (biblioteca Wire).

  Controla DIRETAMENTE (fica tudo neste arduino):
    - Motores Y1/Y2 (puxadores do drone)
    - Motores Y3/Y4 (empurradores)
    - Sensor de pouso
    - Fins de curso do eixo Y

  NAO controla mais os motores X e Z - em vez disso, manda
  comandos pelo I2C para os arduinos escravos:
    - Arduino X (endereco 0x08) -> garra alinhadora
    - Arduino Z (endereco 0x09) -> elevador + garra de bateria + extensao

  IMPORTANTE: as constantes de protocolo (enderecos, codigos de
  mecanismo/acao, bits de status) precisam ser IDENTICAS nos tres
  codigos (Y, X e Z). Se mudar aqui, mude nos outros dois tambem.

  Ligacao entre as placas: SDA (A4) com SDA, SCL (A5) com SCL,
  e GND de todas as placas interligados entre si.
  ============================================================
*/

#include <Wire.h>
#include <AccelStepper.h>

//====================== PROTOCOLO I2C ==========================
#define X_ADDR 0x08
#define Z_ADDR 0x09

// "Mecanismo" alvo do comando (usado so pelo arduino Z, que tem 3 partes)
const byte MEC_ELEVADOR  = 0x01;
const byte MEC_GARRA     = 0x02;
const byte MEC_EXTENSAO  = 0x03;

// Acoes possiveis (iguais para X e Z)
const byte ACAO_MOVER     = 0x01;  // vai ate a posicao alvo (fecha/sobe/extrude)
const byte ACAO_RETORNAR  = 0x02;  // volta pra posicao 0 (abre/desce/recolhe)
const byte ACAO_PARAR     = 0x03;  // para na posicao atual
const byte ACAO_ZERAR     = 0x04;  // define a posicao atual como zero

// Bits do byte de status devolvido pelo arduino Z (um byte por mecanismo)
const byte STATUS_INICIO  = 0x01;  // fim de curso do inicio acionado
const byte STATUS_FIM     = 0x02;  // fim de curso do fim acionado
const byte STATUS_MOVENDO = 0x04;  // motor ainda em movimento

// Bits do byte de status devolvido pelo arduino X (garra tem 2 lados)
const byte STATUS_X1_INICIO = 0x01;
const byte STATUS_X1_FIM    = 0x02;
const byte STATUS_X2_INICIO = 0x04;
const byte STATUS_X2_FIM    = 0x08;
const byte STATUS_X_MOVENDO = 0x10;
//================================================================

//Controle dos motores Y (ficam neste arduino) ------------------------------
//Y "Puxadores" do drone
const int Y1_STEP = 2;
const int Y1_DIR  = 3;

const int Y2_STEP = 4;
const int Y2_DIR  = 5;

//Empurradores Y
const int Y3_STEP = 6;
const int Y3_DIR  = 7;

const int Y4_STEP = 8;
const int Y4_DIR  = 9;

//SENSORES (Y e pouso, os unicos que sobraram neste arduino) ---------------
const int Ystart    = A0;
const int Yend      = A1;
const int YstartEmp = A2;

const int sensorPouso = A3;    //Pode ser um sensor de pressao

//------------------------------------------------------------------------------
//Entradas de comando (mantidas, desabilitadas por enquanto como no original)
const int BotaoStart = 10;
const int BotaoStop  = 11;
const int BotaoReset = 12;

const int pinoEnable = 13;     //Enable do driver CNC deste arduino
//------------------------------------------------------------------------------

AccelStepper motorY(AccelStepper::DRIVER, Y1_STEP, Y1_DIR);
AccelStepper motorYEmpurrao(AccelStepper::DRIVER, Y3_STEP, Y3_DIR);

AccelStepper motor2Y(AccelStepper::DRIVER, Y2_STEP, Y2_DIR);
AccelStepper motor2YEmpurrao(AccelStepper::DRIVER, Y4_STEP, Y4_DIR);

//------------------------CONFIGURACOES-------------------------------------||
String comando = "";
bool printExecu = false;

long passosY = 1600;
long passosYempurrar = 1600;

unsigned long tempoEsperaZ = 0;
unsigned long tempoEsperaExp = 0;

const float VEL_MAX = 800.0;
const float ACEL = 200.0;

// controle de polling do barramento I2C, pra nao floodar o barramento
unsigned long ultimaConsultaI2C = 0;
const unsigned long INTERVALO_I2C = 50; //ms

// guarda o ultimo status lido de cada escravo
byte statusX         = 0;
byte statusZElevador = 0;
byte statusZGarra    = 0;
byte statusZExtensao = 0;

//------------------------CONFIGURACOES-------------------------------------||

enum EstadoAtualMotores{
    STAND_BY,       //HUB de comandos e espera inputs
    ZERAMENTO,      //Zera os eixos e para funcionarem corretamente depois
    ATERRISSAGEM,   //Aguarda o pouso do drone
    HUNT,           //Arrasta o drone ate o ponto de troca de bateria & fecha a pinÃ§a
    MOVENDO_Z,      //Eleva o elevador para a troca da bateria
    TROCA_BATERIA,  //Sequencia de troca da bateria
    RETORNO_Z,      //Desce o elevador com a bateria vazia e a guarda-a
    RETORNO_Y,      //Retorna o drone (arrastando-o) a posicao de lancamento
    EXPANSAO_X,     //Libera ele (drone) lateralmente para lift off!
    STOP,           //Para a bagaca toda :/
    EMER_STT        //Parada de emergencia ahh
};

EstadoAtualMotores estadoatual = STAND_BY;

//--------------------------------------------------------------
//FUNCOES DE COMUNICACAO COM OS ESCRAVOS ------------------------

void enviarComandoX(byte acao){
    Wire.beginTransmission(X_ADDR);
    Wire.write(acao);
    Wire.endTransmission();
}

byte lerStatusX(){
    Wire.requestFrom(X_ADDR, 1);
    if (Wire.available()) return Wire.read();
    return 0;
}

void enviarComandoZ(byte mecanismo, byte acao){
    Wire.beginTransmission(Z_ADDR);
    Wire.write(mecanismo);
    Wire.write(acao);
    Wire.endTransmission();
}

void lerStatusZ(){
    Wire.requestFrom(Z_ADDR, 3);
    if (Wire.available() == 3){
        statusZElevador = Wire.read();
        statusZGarra    = Wire.read();
        statusZExtensao = Wire.read();
    }
}

// consulta os dois escravos, respeitando o intervalo minimo entre consultas
void atualizarStatusEscravos(){
    if (millis() - ultimaConsultaI2C < INTERVALO_I2C) return;
    ultimaConsultaI2C = millis();

    statusX = lerStatusX();
    lerStatusZ();
}

//--------------------------------------------------------------
//----------------------SETA OS PONTOS (zero) INICIAIS----------------
void ZERO_Y(){
    motorY.setCurrentPosition(0);
    motorYEmpurrao.setCurrentPosition(0);
    motor2Y.setCurrentPosition(0);
    motor2YEmpurrao.setCurrentPosition(0);
}

void homing_U(){         //reset Universal
    motorY.moveTo(0);
    motorYEmpurrao.moveTo(0);
    motor2Y.moveTo(0);
    motor2YEmpurrao.moveTo(0);

    enviarComandoX(ACAO_RETORNAR);
    enviarComandoZ(MEC_ELEVADOR, ACAO_RETORNAR);
    enviarComandoZ(MEC_GARRA, ACAO_RETORNAR);
    enviarComandoZ(MEC_EXTENSAO, ACAO_RETORNAR);
}

////////////////////////////////////////////////////////////////

void moverY (){
    motorY.moveTo(passosY);
    motor2Y.moveTo(passosY);
}

////////////////////////////////////////////////////////////////

void pararY(){
    motorY.moveTo(motorY.currentPosition());
    motor2Y.moveTo(motor2Y.currentPosition());
}

void pararYempurra(){
    motorYEmpurrao.moveTo(motorYEmpurrao.currentPosition());
    motor2YEmpurrao.moveTo(motor2YEmpurrao.currentPosition());
}

//--------------------------------------------------------------

void setup(){

    Wire.begin();          //este arduino eh o MESTRE do barramento I2C

    pinMode(pinoEnable, OUTPUT);
    digitalWrite(pinoEnable, LOW);

    pinMode(BotaoStart, INPUT_PULLUP);
    pinMode(BotaoStop, INPUT_PULLUP);
    pinMode(BotaoReset, INPUT_PULLUP);

    motorY.setMaxSpeed(VEL_MAX);
    motorY.setAcceleration(ACEL);

    motor2Y.setMaxSpeed(VEL_MAX);
    motor2Y.setAcceleration(ACEL);

    motorYEmpurrao.setMaxSpeed(VEL_MAX);
    motorYEmpurrao.setAcceleration(ACEL);

    motor2YEmpurrao.setMaxSpeed(VEL_MAX);
    motor2YEmpurrao.setAcceleration(ACEL);

    pinMode(Ystart, INPUT_PULLUP);
    pinMode(Yend, INPUT_PULLUP);
    pinMode(YstartEmp, INPUT_PULLUP);
    pinMode(sensorPouso, INPUT);

    motorY.setPinsInverted(false, true, false);
    motorYEmpurrao.setPinsInverted(false, true, false);

    motor2Y.setPinsInverted(true, false, true);
    motor2YEmpurrao.setPinsInverted(true, false, true);

    Serial.begin(9600);

    Serial.println("ATV, ZR, EMR, ESC");
    Serial.println("zpi, zu, zx, zy, zz");
}

////////////////////////////////////////////////////////////////

void loop()
{
    if (Serial.available()){
        comando = Serial.readStringUntil('\n');
        comando.trim();
        comando.toLowerCase();

        if (comando == "atv" && estadoatual == STAND_BY) {
            Serial.println(" | ESPERANDO POUSO...      |");
            estadoatual = ATERRISSAGEM;

        } else if (comando == "atv" && estadoatual != STAND_BY){
            Serial.println(" | JA EM BUSCA DO Hy-D-J   |");

        } else if (comando == "zr" && estadoatual == STAND_BY){
            estadoatual = ZERAMENTO;
            Serial.println(" | ENTROU NO ZERENCIAMENTO |");

        } else if (comando == "zr" && estadoatual != STAND_BY){
            Serial.println(" | NAO EH POSSIVEL AGORA   |");

        } else if (comando == "emr" && estadoatual != STOP){
            estadoatual = EMER_STT;
            Serial.println(" | PARADA DE EMERGENCIA    |");

        } else if (comando == "emr" && estadoatual == STOP){
            estadoatual = EMER_STT;
            Serial.println(" | JA ESTA PARADO          |");

        } else if (comando == "zu" && estadoatual == ZERAMENTO){    //Move todos os eixos para a posicao inicial (0)
            Serial.println("zerando eixos...");
            homing_U();

        } else if (comando == "zx" && estadoatual == ZERAMENTO){    //Manda a garra X voltar (escravo 0x08)
            enviarComandoX(ACAO_RETORNAR);

        } else if (comando == "zy" && estadoatual == ZERAMENTO){    //Zera os eixos Y (local)
            motorY.moveTo(0);
            motor2Y.moveTo(0);

        } else if (comando == "zz" && estadoatual == ZERAMENTO){    //Manda o elevador Z voltar (escravo 0x09)
            enviarComandoZ(MEC_ELEVADOR, ACAO_RETORNAR);

        } else if (comando == "zpi" && estadoatual == ZERAMENTO){   //define ponto zero em TODOS os eixos
            ZERO_Y();
            enviarComandoX(ACAO_ZERAR);
            enviarComandoZ(MEC_ELEVADOR, ACAO_ZERAR);
            enviarComandoZ(MEC_GARRA, ACAO_ZERAR);
            enviarComandoZ(MEC_EXTENSAO, ACAO_ZERAR);

        } else if (comando == "esc" && estadoatual == ZERAMENTO){
            estadoatual = STAND_BY;
            Serial.println("esc-ed {estadoatual} -> standing by");
            Serial.println(" | SAIU   DO ZERENCIAMENTO |");

        } else if (comando == "esc" && estadoatual != STAND_BY){
            estadoatual = STAND_BY;
            Serial.println("quit {estadoatual} -> standing by");

        } else {
            Serial.println(" | comando desconhecido    |");
        }
    }

    atualizarStatusEscravos();    //consulta X e Z pelo I2C (respeitando o intervalo minimo)

    switch (estadoatual){

        case STAND_BY: //Estado de espera onde comandos podem ser executados

            printExecu = false;
            break;

        case ZERAMENTO: //Estado de gerenciamento dos pontos iniciais

            if (!printExecu){
                Serial.println("em zerenciamento");
                printExecu = true;
            }
            break;

        case ATERRISSAGEM: //Estado de espera do pouso do drone

            if (comando == "esc" && estadoatual != STAND_BY){
                Serial.println("Leaving from landing waiting sequence");
                estadoatual = STAND_BY;

            }else{
                if (digitalRead(sensorPouso) == HIGH){
                    if (!printExecu){
                        Serial.println("Aguardando pouso...");
                        printExecu = true;
                    }

                }else if (digitalRead(sensorPouso) == LOW){
                    Serial.println("Drone pousou");
                    estadoatual = HUNT;
                }
            }
            break;

        case HUNT: { //Move a barra Y e manda a garra X fechar ao mesmo tempo

            if (!printExecu){
                Serial.println("Hunting...");
                printExecu = true;
            }

            //---- eixo Y (local) ----
            if (digitalRead(Yend) == LOW){
                pararY();
            }else{
                moverY();
            }

            //---- garra X (via I2C) ----
            bool x1_fim = statusX & STATUS_X1_FIM;
            bool x2_fim = statusX & STATUS_X2_FIM;

            if (x1_fim && x2_fim){
                enviarComandoX(ACAO_PARAR);
            }else{
                enviarComandoX(ACAO_MOVER);
            }

            //---- so avanca de estado quando Y e as duas pincas de X terminaram ----
            if (digitalRead(Yend) == LOW && x1_fim && x2_fim){
                estadoatual = MOVENDO_Z;
            }
            break;
        }

        case MOVENDO_Z:

            if (statusZElevador & STATUS_FIM){
                enviarComandoZ(MEC_ELEVADOR, ACAO_PARAR);
                tempoEsperaZ = millis();
                estadoatual = TROCA_BATERIA;
            }else{
                enviarComandoZ(MEC_ELEVADOR, ACAO_MOVER);
            }
            break;

        case TROCA_BATERIA:
            // Sequencia de troca de bateria (comandos mandados via I2C pro arduino Z).
            // OBS: assim como no codigo original, os tempos abaixo ainda precisam
            // ser revisados/testados - aqui eles apenas disparam a sequencia apos
            // 1s de espera inicial.
            if (millis() - tempoEsperaZ >= 1000) {
                Serial.println("...espera simulada...");

                enviarComandoZ(MEC_GARRA, ACAO_RETORNAR);        //garante que a garra esta aberta
                enviarComandoZ(MEC_EXTENSAO, ACAO_MOVER);        //extruda a garra Z
                enviarComandoZ(MEC_GARRA, ACAO_MOVER);           //pega a bateria
                enviarComandoZ(MEC_EXTENSAO, ACAO_RETORNAR);     //contrai a garra Z
                enviarComandoZ(MEC_ELEVADOR, ACAO_RETORNAR);     //desce pro armazem de baterias
                enviarComandoZ(MEC_EXTENSAO, ACAO_MOVER);        //extruda a garra Z
                enviarComandoZ(MEC_GARRA, ACAO_RETORNAR);        //solta a bateria velha
                enviarComandoZ(MEC_GARRA, ACAO_MOVER);           //pega a bateria carregada
                enviarComandoZ(MEC_EXTENSAO, ACAO_RETORNAR);     //contrai a garra Z
                enviarComandoZ(MEC_ELEVADOR, ACAO_MOVER);        //sobe pro drone novamente
                enviarComandoZ(MEC_EXTENSAO, ACAO_MOVER);        //extruda a garra Z
                enviarComandoZ(MEC_GARRA, ACAO_RETORNAR);        //encaixa bateria carregada
                enviarComandoZ(MEC_EXTENSAO, ACAO_RETORNAR);     //contrai a garra Z

                estadoatual = RETORNO_Z;
            }
            break;

        case RETORNO_Z:

            if (statusZElevador & STATUS_INICIO){
                enviarComandoZ(MEC_ELEVADOR, ACAO_PARAR);
                estadoatual = RETORNO_Y;
            }else{
                enviarComandoZ(MEC_ELEVADOR, ACAO_RETORNAR);
            }
            break;

        case RETORNO_Y: //Braco Principal

            if (digitalRead(Ystart) == LOW){
                pararY();
                pararYempurra();
                estadoatual = EXPANSAO_X;

            }else{
                motorY.moveTo(0);
                motor2Y.moveTo(0);

                motorYEmpurrao.moveTo(passosYempurrar);
                motor2YEmpurrao.moveTo(passosYempurrar);
            }

            tempoEsperaExp = millis();
            break;

        case EXPANSAO_X:
            if (millis() - tempoEsperaExp >= 10000){

                if (digitalRead(Ystart) == LOW){        //so libera com o fim de curso Y
                    enviarComandoX(ACAO_RETORNAR);

                    motorYEmpurrao.moveTo(0);
                    motor2YEmpurrao.moveTo(0);

                    if ((statusX & STATUS_X1_INICIO) && (statusX & STATUS_X2_INICIO)){
                        enviarComandoX(ACAO_PARAR);
                        estadoatual = STOP;
                    }

                    if (digitalRead(YstartEmp) == LOW){
                        pararYempurra();
                    }
                }
            }
            break;

        case STOP:

            enviarComandoX(ACAO_PARAR);
            pararY();
            pararYempurra();
            enviarComandoZ(MEC_ELEVADOR, ACAO_PARAR);
            enviarComandoZ(MEC_GARRA, ACAO_PARAR);
            enviarComandoZ(MEC_EXTENSAO, ACAO_PARAR);

            Serial.println("Parada normal");
            Serial.println(" -> standing by");

            estadoatual = STAND_BY;
            break;

        case EMER_STT:

            motorY.stop();
            motorYEmpurrao.stop();
            motor2Y.stop();
            motor2YEmpurrao.stop();

            enviarComandoX(ACAO_PARAR);
            enviarComandoZ(MEC_ELEVADOR, ACAO_PARAR);
            enviarComandoZ(MEC_GARRA, ACAO_PARAR);
            enviarComandoZ(MEC_EXTENSAO, ACAO_PARAR);

            Serial.println("Parada EMER");
            estadoatual = STAND_BY;
            Serial.println(" -> standing by");

            break;
    }

    motorY.run();
    motor2Y.run();
    motorYEmpurrao.run();
    motor2YEmpurrao.run();
}