#include <AccelStepper.h>

//Controle dos motores: --------------------------------------------------------------

const int X1_STEP = 2;          //2-3
const int X1_DIR  = 5;          //3-6

const int pinoEnable = 8;

AccelStepper motorX(AccelStepper::DRIVER, X1_STEP, X1_DIR);

//Velocidades////////////////////////////////////////////////////
const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 400.0;

String comando = "";
long passosX = 1600;    //distancia que "anda" rotaciona.. na real eh os passos de volta.. da umas 8 voltas completas +/-

void setup() {

    Serial.begin(9600);

    pinMode(pinoEnable, OUTPUT);
    digitalWrite(pinoEnable, LOW);

    delay(500);

    motorX.setMaxSpeed(VEL_MAX);
    motorX.setAcceleration(ACEL);
    
    Serial.println("-----------------------------------------------------------");
    Serial.println("    >                    Digite:                     <     ");
    Serial.println("    >          zerar, acionar, return, stop          <     ");
    Serial.println("-----------------------------------------------------------");
    Serial.println(" --> zerar: cria o ponto inicial do motor                  ");
    Serial.println(" --> acionar: aciona o motor                               ");
    Serial.println(" --> return: retorna o motor para a posicao inicial        ");
    Serial.println("-----------------------------------------------------------");
    
}

/*
    Serial.println("-----------------------------------------------------------");
    Serial.println("    >                    Digite:                     <     ");
    Serial.println("    >          zerar, acionar, return, stop          <     ");
    Serial.println("-----------------------------------------------------------");
    Serial.println(" --> zerar: cria o ponto inicial do motor                  ");
    Serial.println(" --> acionar: aciona o motor                               ");
    Serial.println(" --> return: retorna o motor para a posicao inicial        ");
    Serial.println("-----------------------------------------------------------");

    Serial.println(" | Motor andando para a posicao...                       | ");
    Serial.println(" | Ponto zero do motor definido!                         | ");
    Serial.println(" | Motor retornando para a posicao inicial...            | ");
    Serial.println(" | Motor parando no meio do trajeto!                     | ");
    Serial.println(" | Comando Desconhecido <?>                              | ");
*/

void loop(){

    if (Serial.available()){                          // 'beffier' if command central script
        comando = Serial.readStringUntil('\n');      // cmds -> atv, zr, zpi, emr, esc
        comando.trim();                             // zu, zx, zy, zz, esczr
        comando.toLowerCase();
        
        if(comando == "acionar"){
            motorX.moveTo(passosX);
            Serial.println(" | Motor andando para a posicao...                       | ");

        } else if(comando == "zerar"){
            motorX.setCurrentPosition(0);
            Serial.println(" | Ponto zero do motor definido!                         | ");

        }else if(comando == "return"){
            motorX.moveTo(0);
            Serial.println(" | Motor retornando para a posicao inicial...            | ");

        }else if(comando == "stop"){
            motorX.moveTo(motorX.currentPosition());
            Serial.println(" | Motor parando no meio do trajeto!                     | ");

        }else{
            Serial.println(" | Comando Desconhecido <?>                              | ");
        }
    }
    motorX.run();
}