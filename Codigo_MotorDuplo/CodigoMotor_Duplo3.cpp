#include <AccelStepper.h>

const int Z1_STEP = 3;  //6   |   Motor 1
const int Z1_DIR = 6;

const int Z2_STEP = 4;  //8   |   Motor 2
const int Z2_DIR = 7;   //9

//Garras no Z trocam a bateria
const int Zgarra_STEP = 10;  //10   
const int Zgarra_DIR = 15;

const int pinoEnable = 8;  //Essencial para o funcionamento do CNC | Devera ser acionado para cada CNC posteriormente

AccelStepper motorZ(AccelStepper::DRIVER, Z1_STEP, Z1_DIR);
AccelStepper motor2Z(AccelStepper::DRIVER, Z2_STEP, Z2_DIR);
//AccelStepper motorZgarra(AccelStepper::DRIVER, Zgarra_STEP, Zgarra_DIR);  //Garra Open/Close
//AccelStepper motorZYgarra(AccelStepper::DRIVER, ZY_STEP, ZY_DIR);
 
String comando = ""; 

unsigned long tempoEsperaZ = 0;   
unsigned long tempoEsperaEX = 0;  

const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 400.0; 
 
bool zerr = false; 
bool printExecu = false;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//long passosX = 200;
//long passosY = 200;
long passosZ = 200;

//long passosZgarra = 200;    
//long passosZYgarra = 200;   
//long passosYempurrar = 200; 

//float valor_diametroX;                          //Possivelmente seria melhor transformar em uma funcao
//float valor_diametroY;                         //Ter atencao ao diametro do eixo + o da engrenagem adicional
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

void homing_U() {  //reset Universal
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
  Serial.println(" | |str, config:                |");
  //Serial.println(" | |zera -> zu, zz, zpi, esc    |");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() { 
  
  tempoEsperaEX = millis();
  tempoEsperaZ = millis();

  if (Serial.available()) {                    // 'beffier' if command central script // 
    comando = Serial.readStringUntil('\n');   // cmds -> atv, zr, zpi, emr, esc      //  
    comando.trim();                          // zu, zx, zy, zz, esczr               // 
    comando.toLowerCase();  
 
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    /*                      (comandos de controle) || str, config, rtrn, adv
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

      /*
      if (digitalRead(Zstart) == LOW) {
        pararZ();
        estadoatual = STAND_BY; //Envia resposta ao proximo arduino
      } else {
        motorZ.moveTo(0);
        motor2Z.moveTo(0);
        abrirGarraBateria();
      }
      */

      //passos = passos*2; //dobra a distancia a percorrer

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
  motorZ.run();
  motor2Z.run();
}
