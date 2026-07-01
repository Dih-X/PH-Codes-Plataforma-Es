#include <AccelStepper.h>     

//Existe opcoes de controlar dois motores pra cada eixo
//(Algumas poucas opcoes e limitadas) Falta implementar por completo

//REVER AS VARIAVEIS, visto que ha muitas incertezas no projeto ainda...

//Sera dois Arduinos, dois motores por parte movel (exceto garra Z)

//Controle dos motores: --------------------------------------------------------------
//Y "Puxadores" do drone
const int Y1_STEP = 3;          //2
const int Y1_DIR = 6;           //3

const int Y2_STEP = 3;          //4
const int Y2_DIR = 6;           //5

    //Empurradores Y

const int Y3_STEP = 3;          //2
const int Y3_DIR = 6;           //3

const int Y4_STEP = 3;          //4
const int Y4_DIR = 6;           //5
 
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
const int X2start = 16;  

const int Ystart = 26;          //Conferir se eh viavel

const int Zstart = 27;          //Conferir se eh viavel
const int ZGstart = 28;         

//Final
const int Xend = 17;            //Conferir se eh viavel
const int X2end = 17;  

const int Yend = 18;
const int YstartEmp = 28;       //Conferir se eh viavel

const int Zend = 29;            //Conferir se eh viavel 
const int ZGend = 30;

const int sensorPouso = A0;       //Pode ser um sensor de pressao
//const int sensor1Proxi1 = A1;   //Ultrassonico, como exemplo
//const int sensor2Proxi1 = A2;   //Ultrassonico, como exemplo
//const int sensor3Proxi1 = A3;   //Ultrassonico, como exemplo

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
AccelStepper motorYEmpurrao(AccelStepper::DRIVER, Y3_STEP, Y3_DIR);
AccelStepper motorZ(AccelStepper::DRIVER, Z1_STEP, Z1_DIR);

//opcao de segundo motor/eixo com controle separado | TERA QUE SER IMPLEMENTADA
AccelStepper motor2X(AccelStepper::DRIVER, X2_STEP, X2_DIR); //AccelStepper::DRIVER
AccelStepper motor2Y(AccelStepper::DRIVER, Y2_STEP, Y2_DIR);
AccelStepper motor2YEmpurrao(AccelStepper::DRIVER, Y4_STEP, Y4_DIR); 
AccelStepper motor2Z(AccelStepper::DRIVER, Z2_STEP, Z2_DIR);

//Garra da bateria
AccelStepper motorZgarra(AccelStepper::DRIVER, Zgarra_STEP, Zgarra_DIR); 

//------------------------CONFIGURACOES-------------------------------------||

//Ajustes////////////////////////////////////////////////////////
int delayPassos = 800;  //(Us) micro segundos
String comando = "";
bool printExecu = false;
//Distancias/////////////////////////////////////////////////////
long passosX = 800;             //dist. movimento 200passos                                                       | Dist = metade da plataforma - largura da base do drone
long passosY = 1600;            //200 por padrao (sem M0,M1,M2)   | O quanto que o drone sera puxado              | Dist = ate o final - profundidade da base do drone (talvez nn seja nescessario)
long passosYempurrar = 1600;    //200 == 1 volta (sem os M's (ON) | O quanto que o drone sera empurrado           | Dist = ate o comeco - profundidade da base do drone
long passosZ = 1600;            //M's = Modulos de subdivisao (embaixo dos capacitores)                           | Dist = altura desejada onde se possa trocar a bateria do drone
long passosZgarra = 800;        //passosYempurrar precisa ser calculado com precisao, do contrario nao funcionara | Passos = Distancia por 1 volta completa, 1VC = 200 passos sem M's adicionais

//Desenvolver Formula matematica que possa calcular automaticamente a distancia a ser percorrida e transforme em passos do motor
//Ex: Distancia (5.00cm) --> levar em conta diametro de objeto de rotacao do motor (Ex: uma engrenagem presa no eixo de rotacao do motor)
//Entao atraves de um processo se eh obtida a quantidade de passos necessaria para anda X distancia
//Levando em conta que o Nema 17 sem encaixes no eixo e sem jumps M da 1 volta completa em 200 passos 

unsigned long tempoEsperaZ = 0;
unsigned long tempoEsperaExp = 0;

//Velocidades////////////////////////////////////////////////////
const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 200.0;

//------------------------CONFIGURACOES-------------------------------------||

enum EstadoAtualMotores{
    STAND_BY,       //HUB de comandos e espera inputs
    ZERAMENTO,      //Zera os eixos e para funcionarem corretamente depois
    ATERRISAGEM,    //Aguarda o pouso do drone
    HUNT,           //Arrasta o drone ate o ponto de troca de bateria & fecha a pinça
    MOVENDO_Z,      //Eleva o elevador para a troca da bateria
    TROCA_BATERIA,  //Tempo com funcao para a troca da bateria
    RETORNO_Z,      //Desce o elevador com a bateria vazia e a guarda-a

    RETORNO_Y,      //Retorna o drone (arrastando-o) a posicao de lancamento

    EXPANSAO_X,     //Libera ele (drone) lateralmente para lift off!
    STOP,           //Para a bagaca toda :/
    EMER_STT
};

EstadoAtualMotores estadoatual = STAND_BY; 

// STAND_BY > ZERAMENTO > ATERRISAGEM > HUNT (MOVER Y > FECHAR GARRA X (AO MESMO TEMPO)) > MOVER [Y] ATE [Y EMPURRAR] > SUBIR Z >
// GARRA Z FECHA > Z DESCE > GARRA Z ABRE > TROCA BATERIA VAZIA POR CHEIA > GARRA FECHA > Z SOBE
// GARRA Z ABRE > TROCA DE BATERIA REALIZADA > RETORNAR [Y] PUXANDO O DRONE COM [A GARRA X] > BARRA [Y DE EMPURRAO] > EMPURRA O DRONE
// ABRIR GARRA EM X > STAND_BY

//--------------------------------------------------------------
//FUNCOES///////////////////////////////////////////////////////

/*void stepMotor(int stepPin){
    digitalWrite(stepPin, HIGH);
    delay(delayPassos); //delayPassos           //Nao acho q seja necessaria v2
    digitalWrite(stepPin, LOW);                 //Talvez seja (para movimento continuo indeterminado)
    delay(delayPassos); //delayPassos           //No entando pode baguncar a contagem de passos e ate tornalos inuteis...
}*/

////////////////////////////////////////////////////////////////
//----------------------SETA OS PONTOS INICIAIS----------------
void ZERO_X(){
    if (Xstart == true){
        motorX.setCurrentPosition(0);
        motor2X.setCurrentPosition(0);
    }
}

void ZERO_Y(){
    if (Ystart == true){
        motorY.setCurrentPosition(0);
        motorYEmpurrao.setCurrentPosition(0);

        motor2Y.setCurrentPosition(0);
        motor2YEmpurrao.setCurrentPosition(0);
    }
}

void ZERO_Z(){
    if (Zstart == true){
        motorZ.setCurrentPosition(0);
        motor2Z.setCurrentPosition(0);

        motorZgarra.setCurrentPosition(0);
    }
}
 
void homing_U(){         //reset Universal
    motorX.moveTo(0);
    motorY.moveTo(0);
    motorZ.moveTo(0);
    motorYEmpurrao.moveTo(0);

    motor2X.moveTo(0);
    motor2Y.moveTo(0);
    motor2Z.moveTo(0);
    motor2YEmpurrao.moveTo(0);
    
    motorZgarra.moveTo(0);
}


////////////////////////////////////////////////////////////////

void moverY (){                 //vao se mover uma distancia e terao que parar, sem fim de curso, apenas na volta (origem) 
    motorY.moveTo(passosY);
    motor2Y.moveTo(passosY);
}

void moverYEmpurrar (){         //vao se mover uma distancia e terao que parar, sem fim de curso, apenas na volta (origem) 
    motorYEmpurrao.moveTo(passosYempurrar);
    motor2YEmpurrao.moveTo(passosYempurrar);
}

void moverZ (){
    motorZ.moveTo(passosZ);
    motor2Z.moveTo(passosZ);
}

void moverX (){
    motorX.moveTo(passosX);
    motor2X.moveTo(passosX);
}

////////////////////////////////////////////////////////////////

void pararX(){
    if (Xend == true){
        motorX.moveTo(motorX.currentPosition());
        motor2X.moveTo(motorX.currentPosition());
    }
}

void pararY(){
    if (Yend == true){
        motorY.moveTo(motorY.currentPosition());
        motor2Y.moveTo(motorY.currentPosition());
    }
}

void pararYempurra(){
    if (Yend == true){
        motorYEmpurrao.moveTo(motorYEmpurrao.currentPosition());
        motor2YEmpurrao.moveTo(motorYEmpurrao.currentPosition());
    }
}

void pararZ(){
    if (Zend == true){
        motorZ.moveTo(motorZ.currentPosition());
        motor2Z.moveTo(motorZ.currentPosition());
    }
}

void pararZgarra(){
    if (Zend == true){
        motorZ.moveTo(motorZ.currentPosition());
    }
}

////////////////////////////////////////////////////////////////
void fecharGarraBateria(){
    motorZgarra.moveTo(passosZgarra);
    
    if (ZGend == true){
        motorZgarra.moveTo(motorZgarra.currentPosition());
    }
    //digitalWrite(Zgarra_DIR, HIGH);     //Fechar
    //stepMotor(Zgarra_STEP);
}

void abrirGarraBateria(){
    motorZgarra.moveTo(0);
    //digitalWrite(Zgarra_DIR, LOW);      //Abrir
    //stepMotor(Zgarra_STEP);
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

    motorYEmpurrao.setMaxSpeed(VEL_MAX);
    motorYEmpurrao.setAcceleration(ACEL);

    motor2Y.setMaxSpeed(VEL_MAX);   //2M
    motor2Y.setAcceleration(ACEL);  //2M

    motor2YEmpurrao.setMaxSpeed(VEL_MAX);
    motor2YEmpurrao.setAcceleration(ACEL);

    motorZ.setMaxSpeed(VEL_MAX);
    motorZ.setAcceleration(ACEL);

    motor2Z.setMaxSpeed(VEL_MAX);   //2M
    motor2Z.setAcceleration(ACEL);  //2M

    motorZgarra.setMaxSpeed(VEL_MAX);
    motorZgarra.setAcceleration(ACEL);

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
    motorZ.setPinsInverted(false, true, false);             //direcao "normal"
    motorYEmpurrao.setPinsInverted(false, true, false);

    motor2X.setPinsInverted(true, false, true);
    motor2Y.setPinsInverted(true, false, true);             //inverte a direcao usando AccelStepper
    motor2Z.setPinsInverted(true, false, true);             //caso o 2o motor seja conectado separadamente
    motorYEmpurrao.setPinsInverted(true, false, true);

    motorZgarra.setPinsInverted(false, true, false);        //INVERTER CASO ESTEJA NO SENTIDO ERRADO

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
    //possivelmente fazer um grande "IF" p/ decidir se aciona ou nao    |  (refeito)  |
    //fazer intervalos para os botoes agirem caso nescessario           |   (feito)   |        (botoes removidos)         |   Por enquanto anyways...
    //fazer comandos por string                                         |   (feito)   |   Precisam de revisao constante   |
    
    if (Serial.available()){                          // 'beffier' if command central script
        comando = Serial.readStringUntil('\n');      // cmds -> atv, zr, zpi, emr, esc
        comando.trim();                             // zu, zx, zy, zz, esczr
        comando.toLowerCase();                     //
        
        /////////////////////////////////////////////////////////////////////////////
        
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

            /////////////////////////////////////////////////////////////////////////////

        } else if (comando == "emr" && estadoatual != STOP){
            estadoatual = EMER_STT;
            Serial.println(" | PARADA DE EMERGENCIA    |");

        } else if (comando == "emr" && estadoatual == STOP){
            estadoatual = EMER_STT;
            Serial.println(" | JA ESTA PARADO          |");

            /////////////////////////////////////////////////////////////////////////////

        } else if (comando == "zu" && estadoatual == ZERAMENTO){    //Move todos os eixos/garras para a posicao inicial (0)
            Serial.println("zerando eixos...");
            homing_U();

        } else if (comando == "zx" && estadoatual == ZERAMENTO){    //Zera o eixo garra X
            motorX.moveTo(0);

        } else if (comando == "zy" && estadoatual == ZERAMENTO){    //Zera os eixos Y
            motorY.moveTo(0);

        } else if (comando == "zz" && estadoatual == ZERAMENTO){    //Zera o eixo e garra Z
            motorZ.moveTo(0);

        } else if (comando == "zpi" && estadoatual == ZERAMENTO){   //&& (motorX.currentPosition != motorX.setCurrentPosition)
            ZERO_X();                                               //define ponto zero | Garra X
            ZERO_Y();                                               //define ponto zero | Barra Y [G.X], e Barra Y [EMP]
            ZERO_Z();                                               //define ponto zero | Eixo Z, e Garra Z

        } else if (comando == "esc" && estadoatual == ZERAMENTO){   //Sai desse modo
            estadoatual = STAND_BY;
            Serial.println("esc-ed {estadoatual} -> standing by");
            Serial.println(" | SAIU   DO ZERENCIAMENTO |");

            /////////////////////////////////////////////////////////////////////////////

        } else if (comando == "esc" && estadoatual != STAND_BY){
            estadoatual = STAND_BY;                               
            Serial.println("quit {estadoatual} -> standing by");
            //std::cout << "Exited " << estadoatual << " state\n"
        
        } else {
            Serial.println(" | comando desconhecido    |");

            //else if (comando == any && estadoatual == ){
            //Serial.println(" | comando desconhecido    |");
        }
    }
    
    switch (estadoatual){
        
        case STAND_BY: //Estado de espera onde comandos podem ser executados

            printExecu = false;
            break;

        case ZERAMENTO: //Estado de gerenciamento dos pontos iniciais | mover para pontos iniciais | zr -> zu, zx, zy, zz, zpi, esczr ->

            if (!printExecu){
                Serial.println("em zerenciamento");
                printExecu = true;
            }

            break;
            
        case ATERRISAGEM: //Estado de espera onde o codigo esta em execucao

            if (comando == "esc" && estadoatual != STAND_BY){               //Abortar espera de aterrissagem (volta pra stand by)
                Serial.println("Leaving from landing waiting sequence");
                estadoatual = STAND_BY;

            }else{
                if (digitalRead(sensorPouso) == HIGH){          //Verifica se o drone pousou ou nao   
                    //Verificar se funciona dps
                    if (!printExecu){
                        Serial.println("Aguardando pouso...");   
                        printExecu = true;
                    }

                }else if (digitalRead(sensorPouso) == LOW){     //drone pousou -> estado passa para Hunt  
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
            
        case HUNT:  //Move a barra Y com as Garras X simultaneamente em direcao a barra de empurrao Y para capturar o drone e alinha-lo

            moverY();
            moverX();
            
            if (!printExecu){
                Serial.println("Hunting...");  
                printExecu = true;
            }

            if (Yend == true){
                pararY();
            }

            if (Xend == true && X2end == true){  //a Garra Z so para quando ambas as pincas tocam no drone dos dois lados, cada uma de cada lado
                pararX();
            }

            estadoatual = MOVENDO_Z;

            break;
            
        case MOVENDO_Z:
            //Serial.println("movendo Z"); 
            //motorZ.moveTo(passosZ); 
            moverZ();

            if (Zend == true){
                pararZ();
                tempoEsperaZ = millis();
                estadoatual = TROCA_BATERIA;
            }
            break;
            
        case TROCA_BATERIA:                                 //Revisar (Refinar) | Revisao 1
            if (millis() - tempoEsperaZ >= 10000) {         //Talvez o tempo de espera tenha que ser a soma total do tempo interno dentro do if
                Serial.println("...espera simulada...");
             
                fecharGarraBateria();                       //pega a bateria 
                millis() - tempoEsperaZ >= 5000;            //espera a garra fechar   
            
                motorZ.moveTo(0);                           //desce pro armazem de baterias
                millis() - tempoEsperaZ >= 10000;
            
                abrirGarraBateria();                        //solta a bateria velha num lugar
                millis() - tempoEsperaZ >= 5000;
            
                fecharGarraBateria();                       //pega a bateria carregada
                millis() - tempoEsperaZ >= 5000;

                motorZ.moveTo(passosZ);                     //sobe pro drone novamente (com bateria cheia)
                millis() - tempoEsperaZ >= 10000;
 
                abrirGarraBateria();                        //encaixa bateria carregada
                millis() - tempoEsperaZ >= 5000;
 
                estadoatual = RETORNO_Z;
            }

            //tempo pra troca, pensar mais depois (chamar funcao de troca)   | (feito)     
            //acionar motor de troca                                         | (feito I guess) 
            //contrai > desce > expande > pega nova > contrai > sobe > expande > troca realizada

            break;

        case RETORNO_Z:
            motorZ.moveTo(0);
            motor2Z.moveTo(0);
            
            if (digitalRead(Zstart) == LOW){
                pararZ();
                estadoatual = RETORNO_Y;
            }

            break;

        case RETORNO_Y:                         //Braco Principal
            motorY.moveTo(0);
            motor2Y.moveTo(0);

            motorYEmpurrao.moveTo(passosYempurrar);     //terao que parar sozinhos apos a distancia certa ter sido percorrida
            motor2YEmpurrao.moveTo(passosYempurrar);    //na vdd poderam parar junto com BY1c/GX visto que andaram juntos 

            if(digitalRead(Ystart) == LOW){
                pararY();
                pararYempurra();

                estadoatual = EXPANSAO_X;
            }
            
            //pararYempurra();
            
            /*motorY.moveTo(0);
            if(digitalRead(Ystart) == LOW){     //barra Y empurrao
                pararY();
                estadoatual = EXPANSAO_X;
            }*/
            tempoEsperaExp = millis();

            break;
        case EXPANSAO_X:
            if (millis() - tempoEsperaExp >= 10000){
                millis() - tempoEsperaExp >= 2000;

                if (digitalRead(Ystart) == LOW){        //so libera c o fim de curso Y
                    motorX.moveTo(0);                   //for ativado/pressionado

                    motorYEmpurrao.moveTo(0);
                    motor2YEmpurrao.moveTo(0);

                    if(digitalRead(Xstart) == LOW){     //falso nesse vosso caso
                        pararX();
                        estadoatual = STOP;
                    }

                    if(digitalRead(YstartEmp) == LOW){
                        pararYempurra();
                    }
                }
            }    
            break;

        case STOP:

            pararX();
            pararY();
            pararZ();
            
            Serial.println("Parada normal");
            estadoatual = STAND_BY;
            Serial.println(" -> standing by");

            break;
            
        case EMER_STT:

            motorX.stop();
            motorY.stop();
            motorYEmpurrao.stop();
            motorZ.stop();
            
            motor2X.stop();
            motor2Y.stop();
            motor2YEmpurrao.stop();
            motor2Z.stop();
            
            //motorX.moveTo(motorX.currentPosition());
            //motorY.moveTo(motorY.currentPosition());
            //motorZ.moveTo(motorZ.currentPosition());

            Serial.println("Parada EMER");
            estadoatual = STAND_BY;
            Serial.println(" -> standing by");

            break;

    }    
    motorX.run();
    motorY.run();
    motorZ.run();
}