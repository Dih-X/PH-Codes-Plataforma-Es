#include <AccelStepper.h>     

//Existe opcoes de controlar dois motores pra cada eixo
//(Algumas poucas opcoes e limitadas) Falta implementar por completo

//Mas o ideal seria controlar dois motores do mesmo exio
//por conexao pre-programada (Ex.: motorX -> 1 & 2)

//O ideal mesmo na vdd seria conectar motores de mesmo eixo juntos
//externamente, e não alguma gambiarra não tão viavel por codigo...

//Controle dos motores: --------------------------------------------------------------
//Y "Puxadores" do drone
const int Y1_STEP = 3;          //2
const int Y1_DIR = 6;           //3
const int Y2_STEP = 3;          //4
const int Y2_DIR = 6;           //5
//Z Elevador da bateria
const int Z1_STEP = 4;          //6
const int Z1_DIR = 7;           //7
const int Z2_STEP = 4;          //8
const int Z2_DIR = 7;           //9
//Garras no Z trocam a bateria
const int Zgarra_STEP = 10;     //10   // Deve-se verificar
const int Zgarra_DIR = 15;      //15   // Se disponivel
//X - Alinhadores da pinça
const int X1_STEP = 2;          //11   // A DEFINIR ENTRADAS    
const int X1_DIR = 5;           //12   // Comunicacao entre arduinos?
const int X2_STEP = 2;          //13   // Talvez seja nescessario visto a falta de entradas*
const int X2_DIR = 5;           //14   //
//SENSORES---------------------------------------------------------------------------
//Inicio
const int Xstart = 16;          //Conferir se eh viavel
const int Ystart = 26;          //Conferir se eh viavel
const int Zstart = 27;          //Conferir se eh viavel
//Final
const int Xend = 17;            //Conferir se eh viavel
const int Yend = 28;            //Conferir se eh viavel
const int Zend = 29;            //Conferir se eh viavel

const int sensorPouso   = A0;   //Pode ser um sensor de pressao
const int sensor1Proxi1 = A1;   //Ultrassonico, como exemplo
const int sensor2Proxi1 = A2;   //Ultrassonico, como exemplo
const int sensor3Proxi1 = A3;   //Ultrassonico, como exemplo
//Fim SENSORES-----------------------------------------------------------------------
//--------------------------------------------------------------
//Entradas de comando
const int BotaoStart = 30;   //verifcar se existe tanta pinagem
const int BotaoStop = 31;    //disponivel assim
const int BotaoReset = 32;

const int pinoEnable = 8;
//------------------------------------------------------------------------------
//MOTORES-----------------------------------------------------------------------
AccelStepper motorX(AccelStepper::DRIVER, X1_STEP, X1_DIR); //AccelStepper::DRIVER
AccelStepper motorY(AccelStepper::DRIVER, Y1_STEP, Y1_DIR);
AccelStepper motorZ(AccelStepper::DRIVER, Z1_STEP, Z1_DIR);

//opcao de segundo m/eixo com controle separado
AccelStepper motor2X(AccelStepper::DRIVER, X2_STEP, X2_DIR); //AccelStepper::DRIVER
AccelStepper motor2Y(AccelStepper::DRIVER, Y2_STEP, Y2_DIR);
AccelStepper motor2Z(AccelStepper::DRIVER, Z2_STEP, Z2_DIR);

//------------------------CONFIGURACOES-------------------------------------||

//Ajustes////////////////////////////////////////////////////////
int delayPassos = 800;  //(Us) micro segundos
String comando = "";
bool printExecu = false;
//Distancias/////////////////////////////////////////////////////
long passosX = 800;     //dist. movimento 200passos
long passosY = 1600;    //200 por padrao (sem M0,M1,M2)
long passosZ = 1600;    //200 == 1 volta (sem os M's (ON))

unsigned long tempoEsperaZ = 0;

//Velocidades////////////////////////////////////////////////////
const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 200.0;

//------------------------CONFIGURACOES-------------------------------------||

enum EstadoAtualMotores{
    STAND_BY,       //HUB de comandos e espera inputs
    ZERAMENTO,      //Zera os eixos e para funcionarem corretamente depois
    ATERRISAGEM,    //Aguarda o pouso do drone
    HUNT,           //Arrasta o drone ate o ponto de troca de bateria & fecha a pinça
    //RETRACAO_X,     //Fecha a pinca p/alinhar o drone ao centro da plataforma
    MOVENDO_Z,      //Eleva o elevador para a troca da bateria
    TROCA_BATERIA,  //Tempo com funcao para a troca da bateria
    RETORNO_Z,      //Desce o elevador com a bateria vazia e a guarda-a
    RETORNO_Y,      //Retorna o drone (arrastando-o) a posicao de lancamento
    EXPANSAO_X,     //Libera ele (drone) lateralmente para lift off!
    STOP,           //Para a bagaca toda :/
    EMER_STT
};

EstadoAtualMotores estadoatual = STAND_BY; 

//STAND_BY > ZERAMENTO > ATERRISAGEM > MOVER Y > FECHAR GARRA X > MOVER Y ATE Z > SUBIR Z >
//GARRA Z FECHA > Z DESCE > GARRA Z ABRE > TROCA BATERIA VAZIA POR CHEIA > GARRA FECHA > Z SOBE
//GARRA Z ABRE > TROCA DE BATERIA REALIZADA >RETORNAR Y PUXANDO O DRONE > ABRIR GARRA EM X > STAND_BY

//--------------------------------------------------------------
//FUNCOES///////////////////////////////////////////////////////

void stepMotor(int stepPin){
    digitalWrite(stepPin, HIGH);
    delay(delayPassos); //delayPassos           //Nao acho q seja necessaria
    digitalWrite(stepPin, LOW);
    delay(delayPassos); //delayPassos
}

////////////////////////////////////////////////////////////////
//----------------------SETA OS PONTOS INICIAIS----------------
void ZERO_X(){
    if (Xstart == true){
        motorX.setCurrentPosition(0);
    }
}

void ZERO_Y(){
    if (Ystart == true){
        motorY.setCurrentPosition(0);
    }
}

void ZERO_Z(){
    if (Zstart == true){
        motorZ.setCurrentPosition(0);
    }
}

void homing_U(){        //reset Universal
    motorX.moveTo(0);
    motorY.moveTo(0);
    motorZ.moveTo(0);
}

/*void homingZ(){                   //referencia caso de ruim
    digitalWrite(Z1_DIR, LOW);
    digitalWrite(Z2_DIR, LOW);

    while (digitalRead(Zstart) != LOW){
        stepMotor(Z1_STEP);
        stepMotor(Z2_STEP);
    }
}*/

////////////////////////////////////////////////////////////////

void moverY (){
    motorY.moveTo(passosY);
}

void moverZ (){
    motorZ.moveTo(passosZ);
}

void moverX (){
    motorX.moveTo(passosX);
}

////////////////////////////////////////////////////////////////

void pararX(){
    if (Xend == true){
        motorX.moveTo(motorX.currentPosition());
    }
}

void pararY(){
    if (Yend == true){
        motorY.moveTo(motorY.currentPosition());
    }
}

void pararZ(){
    if (Zend == true){
        motorZ.moveTo(motorZ.currentPosition());
    }
}

////////////////////////////////////////////////////////////////
void fecharGarraBateria(){
    digitalWrite(Zgarra_DIR, HIGH);     //Fechar
    stepMotor(Zgarra_STEP);
}

void abrirGarraBateria(){
    digitalWrite(Zgarra_DIR, LOW);      //Abrir
    stepMotor(Zgarra_STEP);
}
////////////////////////////////////////////////////////////////
//--------------------------------------------------------------

void setup(){

    pinMode(pinoEnable, OUTPUT);
    digitalWrite(pinoEnable, LOW);

    pinMode(BotaoStart, INPUT_PULLUP);
    pinMode(BotaoStop, INPUT_PULLUP);
    pinMode(BotaoReset, INPUT_PULLUP);

    motorX.setMaxSpeed(VEL_MAX);
    motorX.setAcceleration(ACEL);

    motor2X.setMaxSpeed(VEL_MAX);   //2M
    motor2X.setAcceleration(ACEL);  //2M

    motorY.setMaxSpeed(VEL_MAX);
    motorY.setAcceleration(ACEL);

    motor2Y.setMaxSpeed(VEL_MAX);   //2M
    motor2Y.setAcceleration(ACEL);  //2M

    motorZ.setMaxSpeed(VEL_MAX);
    motorZ.setAcceleration(ACEL);

    motor2Z.setMaxSpeed(VEL_MAX);   //2M
    motor2Z.setAcceleration(ACEL);  //2M

    pinMode(Zgarra_STEP, OUTPUT);
    pinMode(Zgarra_DIR, OUTPUT);

    pinMode(Xstart, INPUT_PULLUP);
    pinMode(Ystart, INPUT_PULLUP);
    pinMode(Zstart, INPUT_PULLUP);

    pinMode(Xend, INPUT_PULLUP);
    pinMode(Yend, INPUT_PULLUP);
    pinMode(Zend, INPUT_PULLUP);

    motorX.setPinsInverted(false, true, false);
    motorY.setPinsInverted(false, true, false);
    motorZ.setPinsInverted(false, true, false);  //direcao "normal"

    motor2X.setPinsInverted(true, false, true);
    motor2Y.setPinsInverted(true, false, true);  //inverte a direcao usando AccelStepper
    motor2Z.setPinsInverted(true, false, true);  //caso o 2o motor seja conectado separadamente

    Serial.begin(9600);
    
    Serial.println("ATV, ZR, EMR, ESC");
    Serial.println("zpi, zu, zx, zy, zz");

    /*
    Serial.println("=======================================================");
    Serial.println(" > MAIN CMDS:                                      < |");
    Serial.println(" | Digite ATV para acionar                           |");
    Serial.println(" | Digite ZR  para entrar no zeraciamento (debuger)  |");
    Serial.println(" | Digite EMR para parada de emergencia              |");
    Serial.println(" | Digite ESC para sair do sistema (ou estado atual) |");
    Serial.println(" |                                                   |");
    Serial.println(" > IN DEBUGERS (zerenciamento):                    < |");
    Serial.println(" | Digite ZPI para setar ponto zero inicial          |");
    Serial.println(" | zu -> zerar eixos                                 |");
    Serial.println(" | zx -> zerar eixo - X                              |");
    Serial.println(" | zy -> zerar eixo - Y                              |");
    Serial.println(" | zz -> zerar eixo - Z                              |");
    Serial.println("=======================================================");
    */
}

////////////////////////////////////////////////////////////////

void loop()
{
    //possivelmente fazer um grande "IF" p/ decidir se aciona ou nao    |  (refeito)
    //fazer intervalos para os botoes agirem caso nescessario           |   (feito)   |   (botoes removidos)
    //fazer comandos por string                                         |   (feito)
    
    if (Serial.available()){                          // beffier if command central script
        comando = Serial.readStringUntil('\n');      // cmds -> atv, zr, zpi, emr, esc
        comando.trim();                             // zu, zx, zy, zz, esczr
        comando.toLowerCase();                     //

        if (comando == "atv" && estadoatual == STAND_BY) {
            Serial.println(" | ESPERANDO POUSO...      |");
            estadoatual = ATERRISAGEM;

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

        } else if (comando == "zu" && estadoatual == ZERAMENTO){
            Serial.println("zerando eixos...");
            homing_U();

        } else if (comando == "zx" && estadoatual == ZERAMENTO){
            motorX.moveTo(0);

        } else if (comando == "zy" && estadoatual == ZERAMENTO){
            motorY.moveTo(0);

        } else if (comando == "zz" && estadoatual == ZERAMENTO){
            motorZ.moveTo(0);

        } else if (comando == "zpi" && estadoatual == ZERAMENTO){  //&& (motorX.currentPosition != motorX.setCurrentPosition)
            ZERO_X();
            ZERO_Y();
            ZERO_Z();

        } else if (comando == "esc" && estadoatual == ZERAMENTO){
            estadoatual = STAND_BY;
            Serial.println("esc-ed {estadoatual} -> standing by");
            Serial.println(" | SAIU   DO ZERENCIAMENTO |");
            
        } else if (comando == "esc" && estadoatual != STAND_BY){
            estadoatual = STAND_BY;                               
            Serial.println("quit {estadoatual} -> standing by");
            //std::cout << "Quited " << estadoatual << " state\n"
        
        } else {
            Serial.println(" | comando desconhecido    |");

            //else if (comando == any && estadoatual == ){
            //Serial.println(" | comando desconhecido    |");
        }
    }
    
    switch (estadoatual){
        
        case STAND_BY:
            printExecu = false;
            break;
        case ZERAMENTO:                     //zr -> zu, zx, zy, zz, zpi, esczr ->
            if (!printExecu){
                Serial.println("em zerenciamento");
                printExecu = true;
            }
            break;
        case ATERRISAGEM:

            if (comando == "esc" && estadoatual != STAND_BY){
                estadoatual = STAND_BY;        
                Serial.println("esc-ed from landing");
            }else{
                if (digitalRead(sensorPouso) == HIGH){    //Verificar se funciona dps                                                                  
                    if (!printExecu){
                        Serial.println("Aguardando pouso...");
                        printExecu = true;
                    }

                }else if (digitalRead(sensorPouso) == LOW){
                    Serial.println("Drone pousou");
                    estadoatual = HUNT;
                }
            }
                /*while (digitalRead(sensorPouso) == HIGH){    //Verificar se funciona dps                                                                  
                    if (!printExecu){
                        Serial.println("Aguardando pouso...");
                        printExecu = true;
                    }
                }*/
            //sensorPouso == LOW; //para testar
            break;
        case HUNT:
            moverY();
            moverX();
            Serial.println("Hunting");
            if (Yend == true){
                pararY();
            }
            if (Xend == true){
                pararX();
            }
            estadoatual = MOVENDO_Z;
            break;
        case MOVENDO_Z:
            //Serial.println("movendo Z");
            motorZ.moveTo(passosZ);
            if (Zend == true){
                pararZ();
                tempoEsperaZ = millis();
                estadoatual = TROCA_BATERIA;
            }
            break; 
        case TROCA_BATERIA:
            if (millis() - tempoEsperaZ >= 10000) {
                Serial.println("...espera simulada...");    //Revisar (Refinar)
                fecharGarraBateria();
                millis() - tempoEsperaZ >= 2000;
                abrirGarraBateria();
                millis() - tempoEsperaZ >= 2000;
                fecharGarraBateria();
                millis() - tempoEsperaZ >= 2000;
                abrirGarraBateria();
                millis() - tempoEsperaZ >= 2000;
                estadoatual = RETORNO_Z;
            }
            //tempo pra troca, pensar mais depois (chamar funcao de troca)   | (feito)    
            //acionar motor de troca
            //contrai > desce > expande > pega nova > contrai > sobe > expande > troca realizada
            break;
        case RETORNO_Z:
            motorZ.moveTo(0);
            if (digitalRead(Zstart) == LOW){
                pararZ();
                estadoatual = RETORNO_Y;
            }
            break;
        case RETORNO_Y:                         //Braco Principal
            motorY.moveTo(0);
            if(digitalRead(Ystart) == LOW){
                pararY();
                estadoatual = EXPANSAO_X;
            }
            break;
        case EXPANSAO_X:
            if (digitalRead(Ystart) == LOW){
                motorX.moveTo(0);

                if(digitalRead(Xstart) == LOW){
                    pararX();
                    estadoatual = STOP;
                }
            }
            break;
        case STOP:
            pararX();
            pararY();
            pararZ();
            Serial.println("Parada normal");
            estadoatual = STAND_BY;
            Serial.println("standing by");
            break;
        case EMER_STT:
            motorX.moveTo(motorX.currentPosition());
            motorY.moveTo(motorY.currentPosition());
            motorZ.moveTo(motorZ.currentPosition());
            Serial.println("Parada EMER");
            estadoatual = STAND_BY;
            Serial.println("standing by");
            break;
    }    
    motorX.run();
    motorY.run();
    motorZ.run();
}