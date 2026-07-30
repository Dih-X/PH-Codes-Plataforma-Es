#include <AccelStepper.h>     //CORRETO
//#include time               //2 nemas 17 | 1 nema 14
//#include <Stepper.h>        //Diferente
//Controle dos motores: --------------------------------------------------------------
//Y "Puxadores" do drone
const int Y1_STEP = 2;
const int Y1_DIR = 3;
const int Y2_STEP = 4;
const int Y2_DIR = 5;
//Z Elevador da bateria
const int Z1_STEP = 6;
const int Z1_DIR = 7;
const int Z2_STEP = 8;
const int Z2_DIR = 9;
//Garras no Z trocam a bateria
const int Zgarra_STEP = 10;      //Deve-se verificar
const int Zgarra_DIR = 15;       //Se disponivel

//X - Alinhadores "pinca" lol
const int X1_STEP = 11;          // A DEFINIR ENTRADAS    
const int X1_DIR = 12;           // Comunicacao entre arduinos?
const int X2_STEP = 13;          // Talvez seja nescessario visto a falta de entradas*
const int X2_DIR = 14;           //
//SENSORES---------------------------------------------------------------------------
//Inicio
const int Xstart = 16;          //Conferir se eh viavel
const int Ystart = 26;          //Conferir se eh viavel
const int Zstart = 27;          //Conferir se eh viavel
//Final
const int Xend = 17;            //Conferir se eh viavel
const int Yend = 28;            //Conferir se eh viavel
const int Zend = 29;            //Conferir se eh viavel

const int sensorPouso = A0;     //Pode ser um sensor de pressao
const int sensor1Proxi1 = A1;   //Ultrassonico, como exemplo
const int sensor2Proxi1 = A2;   //Ultrassonico, como exemplo
const int sensor3Proxi1 = A3;   //Ultrassonico, como exemplo
//Fim SENSORES-----------------------------------------------------------------------
//--------------------------------------------------------------
//Entradas de comando
const int BotaoStart = 30;   //verifcar se existe tanta pinagem
const int BotaoStop = 31;    //disponivel assim
const int BotaoReset = 32;
//--------------------------------------------------------------
//Ajustes////////////////////////////////////////////////////////
int delayPassos = 800; //(Us) micro segundos

enum EstadoAtualMotores{
    STAND_BY,       //HUB de comandos e espera inputs
    ZERAMENTO,      //Zera os eixos e para funcionarem corretamente depois
    ATERRISAGEM,    //Aguarda o pouso do drone
    MOVENDO_Y,      //Arrasta o drone ate o ponto de troca de bateria
    RETRACAO_X,     //Fecha a pinca p/alinhar o drone ao centro da plataforma
    MOVENDO_Z,      //Eleva o elevador para a troca da bateria
    TROCA_BATERIA,  //Tempo com funcao para a troca da bateria
    RETORNO_Z,      //Desce o elevador com a bateria vazia e a guarda-a
    RETORNO_Y,      //Retorna o drone (arrastando-o) a posicao de lancamento
    EXPANSAO_X,     //Libera ele (drone) lateralmente para lift off!
    STOP            //Para a bagaca toda :/
};

EstadoAtualMotores estadoatual = STAND_BY; 

//STAND_BY > ZERAMENTO > ATERRISAGEM > MOVER Y > FECHAR GARRA X > MOVER Y ATE Z > SUBIR Z >
//GARRA Z FECHA > Z DESCE > GARRA Z ABRE > TROCA BATERIA VAZIA POR CHEIA > GARRA FECHA > Z SOBE
//GARRA Z ABRE > TROCA DE BATERIA REALIZADA >RETORNAR Y PUXANDO O DRONE > ABRIR GARRA EM X > STAND_BY

////////////////////////////////////////////////////////////////

void setup(){
    pinMode(BotaoStart, INPUT_PULLUP);
    pinMode(BotaoStop, INPUT_PULLUP);
    pinMode(BotaoReset, INPUT_PULLUP);

    pinMode(X1_STEP, OUTPUT);
    pinMode(X2_STEP, OUTPUT);
    pinMode(X1_DIR, OUTPUT);
    pinMode(X2_DIR, OUTPUT);
    
    pinMode(Y1_STEP, OUTPUT);
    pinMode(Y2_STEP, OUTPUT);
    pinMode(Y1_DIR, OUTPUT);
    pinMode(Y2_DIR, OUTPUT);

    pinMode(Z1_STEP, OUTPUT);
    pinMode(Z2_STEP, OUTPUT);
    pinMode(Z1_DIR, OUTPUT);
    pinMode(Z2_DIR, OUTPUT);

    pinMode(Zgarra_STEP, OUTPUT);
    pinMode(Zgarra_DIR, OUTPUT);

    pinMode(Xstart, INPUT_PULLUP);
    pinMode(Ystart, INPUT_PULLUP);
    pinMode(Zstart, INPUT_PULLUP);

    pinMode(Xend, INPUT_PULLUP);
    pinMode(Yend, INPUT_PULLUP);
    pinMode(Zend, INPUT_PULLUP);

    Serial.begin(9600);
}

//--------------------------------------------------------------
////////////////////////////////////////////////////////////////

void stepMotor(int stepPin){
    digitalWrite(stepPin, HIGH);
    delay(delayPassos); //delayPassos
    digitalWrite(stepPin, LOW);
    delay(delayPassos); //delayPassos
}

////////////////////////////////////////////////////////////////

void homingY(){
    digitalWrite(Y1_DIR, LOW);
    digitalWrite(Y2_DIR, LOW);

    while (digitalRead(Ystart) != LOW){
        stepMotor(Y1_STEP);
        stepMotor(Y2_STEP);
    }
}

void homingZ(){
    digitalWrite(Z1_DIR, LOW);
    digitalWrite(Z2_DIR, LOW);

    while (digitalRead(Zstart) != LOW){
        stepMotor(Z1_STEP);
        stepMotor(Z2_STEP);
    }
}

void homingX(){
    digitalWrite(X1_DIR, LOW);
    digitalWrite(X2_DIR, LOW);

    while (digitalRead(Xstart) != LOW){
        stepMotor(X1_STEP);
        stepMotor(X2_STEP);
    }
}

////////////////////////////////////////////////////////////////

void moverY (bool frente){
    digitalWrite(Y1_DIR, frente);
    digitalWrite(Y2_DIR, frente);

    stepMotor(Y1_STEP);
    stepMotor(Y2_STEP);
}

void moverZ (bool frente){
    digitalWrite(Z1_DIR, frente);
    digitalWrite(Z2_DIR, frente);

    stepMotor(Z1_STEP);
    stepMotor(Z2_STEP);
}

void moverX (bool frente){
    digitalWrite(X1_DIR, frente);
    digitalWrite(X2_DIR, frente);

    stepMotor(X1_STEP);
    stepMotor(X2_STEP);
}

//void pararX(){}
//void pararY(){}
//void pararZ(){}
////////////////////////////////////////////////////////////////
void pararX(){
    digitalWrite(X1_DIR, HIGH);
    digitalWrite(X2_DIR, HIGH);
}

void pararY(){
    digitalWrite(Y1_DIR, HIGH);
    digitalWrite(Y2_DIR, HIGH);
}

void pararZ(){
    digitalWrite(Z1_DIR, HIGH);
    digitalWrite(Z2_DIR, HIGH);
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

void loop()
{
    //possivelmente fazer um grande "IF" p/ decidir se aciona ou nao (desfeito)
    //fazer intervalos para os botoes agirem caso nescessario (feito)
    //fazer comandos por string
    
    switch (estadoatual){ 
        
        case STAND_BY:
            if (digitalRead(BotaoStart) == LOW){
                estadoatual = ZERAMENTO;

            } else if (digitalRead(BotaoReset) == LOW){
                estadoatual = ZERAMENTO;

            } else if (digitalRead(BotaoStop) == LOW){
                estadoatual = STOP;

            }

            break;

        case ZERAMENTO:
            homingY();
            homingZ();
            homingX();
            estadoatual = ATERRISAGEM;

            break;       
            
        case ATERRISAGEM:
            while (digitalRead(sensorPouso) == LOW){    //Verificar se funciona dps                                                                  
                Serial.println("Aguardando pouso...");
            }

            if (digitalRead(sensorPouso) == LOW){
                Serial.println("Drone pousou");
                estadoatual = MOVENDO_Y;                   
            } 
            break;

        case MOVENDO_Y:
            moverY(true);
                
            if (analogRead(sensor2Proxi1) > 100){
                pararY(); 
            }

            estadoatual = RETRACAO_X;
            break;

        case RETRACAO_X:

            moverX(true);
            if (analogRead(sensor1Proxi1) > 5){
                pararX();
            }

            if (digitalRead(BotaoStop) == LOW){
                estadoatual = STOP;
            }

            estadoatual = MOVENDO_Z;
            break;

        case MOVENDO_Z:
            moverZ(true);

            if (digitalRead(Zend) == LOW){
                pararZ();
                estadoatual = TROCA_BATERIA;
            }
            break;
                
        case TROCA_BATERIA:
            fecharGarraBateria();
            delay(3000);  
            abrirGarraBateria();
            delay(3000);  
            fecharGarraBateria();
            delay(3000);  
            abrirGarraBateria();
            delay(3000);                //tempo pra troca, pensar mais depois (chamar funcao de troca)
                                
            estadoatual = RETORNO_Z;    //acionar motor de troca
            break;                      //contrai > desce > expande > pega nova > contrai > sobe > expande > troca realizada

        case RETORNO_Z:
            moverZ (false);
            if (digitalRead(Zstart) == LOW){
                pararZ();
                estadoatual = RETORNO_Y;
            }
            break;
                
        case RETORNO_Y:
            moverY(false);

            if(digitalRead(Ystart) == LOW){
                pararY();
                estadoatual = STOP;
            }
            break;
                
        case STOP:
            pararX();
            pararY();
            pararZ();
            delay(1000);
            estadoatual = STAND_BY;

            break;
            
    }    
} 