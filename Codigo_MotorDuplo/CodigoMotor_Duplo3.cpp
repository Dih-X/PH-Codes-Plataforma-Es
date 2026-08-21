#include <AccelStepper.h>

const int Z1_STEP = 3;  //6   |   Motor 1
const int Z1_DIR = 6;

const int Z2_STEP = 4;  //8   |   Motor 2
const int Z2_DIR = 7;   //9

const int pinoEnable = 8;  //Essencial para o funcionamento do CNC

AccelStepper motorZ(AccelStepper::DRIVER, Z1_STEP, Z1_DIR);
AccelStepper motor2Z(AccelStepper::DRIVER, Z2_STEP, Z2_DIR);
 
String comando = ""; 

unsigned long tempoEsperaZ = 0;   

const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 400.0; 
 
bool zerr = false; 
bool printExecu = false;

long passosZ = 200;                      
float valor_diametroZ;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Espera > ou Inicia ou entra nas configs.
//    Config. = altera a distancia que o motor deve percorrer
//    Inicia = inicia o movimento do motor
// Selecionar > Percorrer mais distancia ou retornar ao ponto zero
// Deve voltar ao ponto zero independente da distancia percorrida caso seja modificada para andar mais que o pre-setado

enum EstadoDuploMotor{    //---> (comandos de controle) || str, config, rtrn, adv
  STAND_BY,               //---> estado de espera
  CONFIG,                 //---> pra colocar a distancia a andar
  MOVER,                  //---> comeca a andar
  SELECT,                 //---> andar mais ou retornar apos a primeira andada
  ZERADOR                 //---> nao sei ao certo/talvez seja usado
};

EstadoDuploMotor estadoatual = STAND_BY;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moverZ(){
  motorZ.moveTo(passosZ);
  motor2Z.moveTo(passosZ);
}

void homing_U() {  //reset Universal para a posicao zero
  motorZ.moveTo(0);
  motor2Z.moveTo(0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calcularDistancia(float diametro, int passos){
  
  //diametro = float valor_diametro;
  float circunferencia = 3.14 * diametro;
  float distancia = passos * circunferencia;
  int passos_Andar = distancia / circunferencia;

  return passos_Andar;
}

//calcularDistancia(300, 200); Ex.Guia

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(pinoEnable, OUTPUT);
  digitalWrite(pinoEnable, LOW);

  motorZ.setMaxSpeed(VEL_MAX);
  motorZ.setAcceleration(ACEL);

  motor2Z.setMaxSpeed(VEL_MAX);   //2M
  motor2Z.setAcceleration(ACEL);  //2M
    
  pinMode(Zgarra_STEP, OUTPUT);
  pinMode(Zgarra_DIR, OUTPUT);
   
  pinMode(Zstart, INPUT_PULLUP);
  pinMode(Zend, INPUT_PULLUP);
    
  motorZ.setPinsInverted(false, true, false); 
  motor2Z.setPinsInverted(true, false, true); 
  
  Serial.begin(9600); 
  delay(100);

  Serial.println(" | AGUARDANDO ORDENS            |");
  Serial.println(" |\                             |");
  Serial.println(" | \                            |");
  Serial.println(" | |str, config, rtrn, adv      |");
  //Serial.println(" | |zera -> zu, zz, zpi, esc    |");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() { 
  
  tempoEsperaEX = millis();
  tempoEsperaZ = millis();

  if (Serial.available()) {                   
    comando = Serial.readStringUntil('\n');   
    comando.trim();                          
    comando.toLowerCase();  
 
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    /*

    (comandos de controle) || str, config, rtrn, adv

    enum EstadoDuploMotor{           
      STAND_BY,     ---> estado de espera
      CONFIG,       ---> pra colocar a distancia a andar
      MOVER,        ---> comeca a andar
      SELECT,       ---> andar mais ou retornar apos a primeira andada
      ZERADOR       ---> nao sei ao certo/talvez nn seja usado
    };
    */

    if (comando == "str" && estadoatual == STAND_BY) { 
      estadoatual = MOVER;

      Serial.println(" > Em locomocao... "); 

    } else if (comando == "config" && estadoatual == STAND_BY) {
      estadoatual = CONFIG; 
  
    } else if (comando == "rtrn" && estadoatual == STAND_BY) {  // <-- revisar aqui
      
      estadoatual = ZERADOR;  
      zerr = true;             
    }
  }
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // | STAND_BY, CONFIG, MOVER, SELECT, ZERADOR |
  
  switch (estadoatual){
    case STAND_BY:

      printExecu = false;
      break;
      
    case ZERADOR:  

      if (!printExecu){
        Serial.println(" | RETORNANDO...               |");
        printExecu = true;
      }
      
      void homing_U();
      estadoatual = STAND_BY;

      break;

    case MOVER:

      moverZ();
      estadoatual = SELECT;

      break; 

    case SELECT:  //avancar ou retornar

      if (comando == "adv" && estadoatual == SELECT){

        passosZ = passosZ*2;                                      //Revisar se eh funcional

        moverZ();
        estadoatual = SELECT;

      } else if (comando == "rtrn" && estadoatual == SELECT){
        homing_U();
        estadoatual = STAND_BY;
      }

      break;

    case CONFIG:  //definir distancia a percorrer
 
      if (comando == int numeroD && estadoatual == CONFIG){     //Revisar
        calcularDistancia(numeroD, passosz);
      }

      break;
  }

  motorZ.run();
  motor2Z.run();

}
