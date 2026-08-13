#include <AccelStepper.h>

//Controle dos motores: --------------------------------------------------------------

const int X1_STEP = 3;          //2
const int X1_DIR = 6;           //3

AccelStepper motorX(AccelStepper::DRIVER, X1_STEP, X1_DIR);

const int pinoEnable = 8;

//Velocidades////////////////////////////////////////////////////
const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 200.0;

String comando = "";
long passosX = 800;

void setup() {

    Serial.begin(9600);

    pinMode(pinoEnable, OUTPUT);
    digitalWrite(pinoEnable, LOW);

    delay(500);

    motorX.setMaxSpeed(VEL_MAX);
    motorX.setAcceleration(ACEL);
    
    Serial.println("-----------------------------------------------------------");
    Serial.println("    >             zerar, acionar, return             <     ");
    Serial.println("-----------------------------------------------------------");
    Serial.println(" --> zerar: cria o ponto inicial do motor                  ");
    Serial.println(" --> acionar: aciona o motor                               ");
    Serial.println(" --> return: retorna o motor para a posicao inicial        ");
    Serial.println("-----------------------------------------------------------");
    
}

void loop(){

    if (Serial.available()){                          // 'beffier' if command central script
        comando = Serial.readStringUntil('\n');      // cmds -> atv, zr, zpi, emr, esc
        comando.trim();                             // zu, zx, zy, zz, esczr
        comando.toLowerCase();
        
        if(comando == "acionar"){
            motorX.moveTo(passosX);
            Serial.println("Motor andando para a posicao...");

        } else if(comando == "zerar"){
            motorX.setCurrentPosition(0);
            Serial.println("Ponto zero do motor definido!");

        }else if(comando == "return"){
            motorX.moveTo(0);
            Serial.println("Motor retornando para a posicao inicial...");

        }
    }

    motorX.run();

}