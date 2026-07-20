#include <AccelStepper.h>

//Controle dos motores: --------------------------------------------------------------
//X - Alinhadores da pinça
const int X1_STEP = 2;          //11   // A DEFINIR ENTRADAS    
const int X1_DIR = 5;           //12   // Comunicacao entre arduinos?
const int X2_STEP = 2;          //13   // Talvez seja nescessario visto a falta de entradas*
const int X2_DIR = 5;           //14   //

//SENSORES---------------------------------------------------------------------------
//Inicio
const int Xstart = 16;          //Conferir se eh viavel
const int X2start = 16;

//Final
const int Xend = 17;            //Conferir se eh viavel
const int X2end = 17;                

//Fim SENSORES------------------------------------------------------------------

const int pinoEnable = 8;

AccelStepper motorX(AccelStepper::DRIVER, X1_STEP, X1_DIR);
AccelStepper motor2X(AccelStepper::DRIVER, X2_STEP, X2_DIR);

String comando = "";
bool printExecu = false;

long passosX = 800;

//Velocidades////////////////////////////////////////////////////
const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 200.0;

//------------------------CONFIGURACOES-------------------------------------||

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
    STOP,           
    EMER_STT        
};

EstadoAtualMotores estadoatual = STAND_BY; 

void ZERO_X(){ 
    if (Xstart == true){
        motorX.setCurrentPosition(0);  
        motor2X.setCurrentPosition(0); 
    }
}

void homing_U(){         //reset Universal
    motorX.moveTo(0);
    motor2X.moveTo(0);
}

////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////

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

    pinMode(Xstart, INPUT_PULLUP);
    pinMode(Xend, INPUT_PULLUP);
    
    motorX.setPinsInverted(false, true, false);
    motor2X.setPinsInverted(true, false, true);

    Serial.begin(9600);

    Serial.println("ATV, ZR, EMR, ESC");    
    Serial.println("zpi, zu, zx, zy, zz");  

}

////////////////////////////////////////////////////////////////

void loop()
{
    if (Serial.available()){                          // 'beffier' if command central script
        comando = Serial.readStringUntil('\n');      // cmds -> atv, zr, zpi, emr, esc
        comando.trim();                             // zu, zx, zy, zz, esczr
        comando.toLowerCase();                     //
        
        /////////////////////////////////////////////////////////////////////////////
        
        /*if (comando == "atv" && estadoatual == STAND_BY) {
            Serial.println(" | ESPERANDO POUSO...      |");
            estadoatual = ATERRISSAGEM;

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
            ZERO_Z();                                               //define ponto zero | Eixo Z
            ZERO_Zvador();                                          //define zero point | Garra e Extensao Z

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
        }*/
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
            
        case ATERRISSAGEM: //Estado de espera onde o codigo esta em execucao

            if (comando == "esc" && estadoatual != STAND_BY){               //Abortar espera de aterrissagem (volta pra stand by)
                Serial.println("Leaving from landing waiting sequence");    
                estadoatual = STAND_BY;
                //pararZ();       
                //pararZgarra();

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

            //sensorPouso == LOW; //para testar 
            break;
            
        case HUNT:  //Move a barra Y com as Garras X simultaneamente em direcao a barra de empurrao Y para capturar o drone e alinha-lo

            if (!printExecu){
                Serial.println("Hunting...");  
                printExecu = true;
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if (Yend == true){      //Talvez seja necessario fazer com que o if acione um tempo depois que o motor partir para nao causar possiveis travamentos 
                pararY();           //Apos X tempo o if eh liberado e entao se eh possivel usar o fim de curso para parar o motor... 
                estadoatual = MOVENDO_Z; 
            }else{
                moverY();           //Enviar Sinal RX/TX para acionar Sub. El. Z
            }
            
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            if (Xend == true && X2end == true){  //a Garra X so para quando ambas as pincas tocam no drone dos dois lados, cada uma de cada lado
                pararX();
            }else{
                moverX();
            }                       //Enviar Sinal RX/TX para acionar Sub. X (?)

            break;
            
        case MOVENDO_Z:
            break;
            
        case TROCA_BATERIA:                               
            break;

        case RETORNO_Z:
            break;

        case RETORNO_Y:                       
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
            
            Serial.println("Parada normal");
            Serial.println(" -> standing by");
            
            estadoatual = STAND_BY;
            break;
            
        case EMER_STT:

            motorX.stop();
            motor2X.stop();

            Serial.println("Parada EMER");
            estadoatual = STAND_BY;
            Serial.println(" -> standing by");

            break;
    }    
    
    motorX.run();
    motor2X.run();
     
}