#include <AccelStepper.h>

const int Z1_STEP = 3;  //6
const int Z1_DIR = 6;
const int Z2_STEP = 4;  //8
const int Z2_DIR = 7;   //9

//Empurrador da Garra Z
const int ZY_STEP = 99;         // Deve-se verificar
const int ZY_DIR = 98;  

//Garras no Z trocam a bateria
const int Zgarra_STEP = 10;  //10   
const int Zgarra_DIR = 15;

const int Zstart = 27;   //Conferir se eh viavel
const int ZGstart = 28;
const int ZExstart = 29;

const int Yend = 33;

const int Zend = 32;  //Conferir se eh viavel
const int ZGend = 30;
const int ZExend = 31;

const int pinoEnable = 8;  

AccelStepper motorZ(AccelStepper::DRIVER, Z1_STEP, Z1_DIR);
AccelStepper motor2Z(AccelStepper::DRIVER, Z2_STEP, Z2_DIR);
AccelStepper motorZgarra(AccelStepper::DRIVER, Zgarra_STEP, Zgarra_DIR);  
AccelStepper motorZYgarra(AccelStepper::DRIVER, ZY_STEP, ZY_DIR);
 
String comando = ""; 
 
long passosZ = 1600;       
long passosZgarra = 800;   
long passosZYgarra = 800;  

unsigned long tempoEsperaZ = 0;   
unsigned long tempoEsperaEX = 0;  

const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 200.0; 
 
bool zerr = false; 
bool printExecu = false;

/*enum EstadoProtoMotor {
  STAND_BY,
  MOVENDO_Z,
  TROCA_BATERIA,
  RETORNO_Z,
  ZERAMENTO,
  STOP,
  EMER_STT
};*/

enum EstadoAtualMotores {
  STAND_BY,       //HUB de comandos e espera inputs
  ZERAMENTO,      //Zera os eixos e para funcionarem corretamente depois
  ATERRISSAGEM,   //Aguarda o pouso do drone
  HUNT,           //Arrasta o drone ate o ponto de troca de bateria & fecha a pinça
  MOVENDO_Z,      //Eleva o elevador para a troca da bateria
  TROCA_BATERIA,  //Tempo com funcao para a troca da bateria
  RETORNO_Z,      //Desce o elevador com a bateria vazia e a guarda-a
  RETORNO_Y,      //Retorna o drone (arrastando-o) a posicao de lancamento
  EXPANSAO_X,     //Libera ele (drone) lateralmente para lift off!
  STOP,           //Para a bagaca toda :/
  EMER_STT        //Parada de emergencia ahh
};

//STAND_BY > ZERAMENTO > ATERRISSAGEM > HUNT > MOVENDO_Z >
//TROCA_BATERIA > RETORNO_Z > RETORNO_Y >
//EXPANSAO_X > STOP > EMER_STT

EstadoAtualMotores estadoatual = STAND_BY;

///////////////////////////////////////////////////////////////////////////////////////

void ZERO_Z() {
  if (Zstart == true) {
    motorZ.setCurrentPosition(0);
    motor2Z.setCurrentPosition(0);
  }
}

void ZERO_Zvador() {
  if (ZGstart == true) {
    motorZgarra.setCurrentPosition(0);
  }

  if (ZExstart == true) {
    motorZYgarra.setCurrentPosition(0);
  }
}
void homing_U() {  //reset Universal

  motorZ.moveTo(0);
  motor2Z.moveTo(0);

  motorZgarra.moveTo(0);
  motorZYgarra.moveTo(0);
}

///////////////////////////////////////////////////////////////////////////////////////

void moverZ() {
  motorZ.moveTo(passosZ);
  motor2Z.moveTo(passosZ);
}

void pararZ() {
  if (Zend == true) {
    motorZ.moveTo(motorZ.currentPosition());
    motor2Z.moveTo(motorZ.currentPosition());
  }
}

void pararZgarra() {
  if (Zend == true) {
    motorZgarra.moveTo(motorZgarra.currentPosition());
    motorZYgarra.moveTo(motorZYgarra.currentPosition());
  }
}

///////////////////////////////////////////////////////////////////////////////////////

void fecharGarraBateria() {
  if (ZGend == true) {
    motorZgarra.stop();
  } else {
    motorZgarra.moveTo(passosZgarra);
  }
}

void abrirGarraBateria() {
  if (ZGstart == true) {
    motorZgarra.stop();
  } else {
    motorZgarra.moveTo(0);
  }
}

///////////////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(pinoEnable, OUTPUT);
  digitalWrite(pinoEnable, LOW);

  motorZ.setMaxSpeed(VEL_MAX);
  motorZ.setAcceleration(ACEL);

  motor2Z.setMaxSpeed(VEL_MAX);   //2M
  motor2Z.setAcceleration(ACEL);  //2M
  
  motorZgarra.setMaxSpeed(VEL_MAX);
  motorZgarra.setAcceleration(ACEL);
    
  pinMode(Zgarra_STEP, OUTPUT);
  pinMode(Zgarra_DIR, OUTPUT);
   
  pinMode(Zstart, INPUT_PULLUP);
  pinMode(Zend, INPUT_PULLUP);
    
  motorZ.setPinsInverted(false, true, false); 
  motor2Z.setPinsInverted(true, false, true); 
  
  motorZgarra.setPinsInverted(false, true, false);  //INVERTER CASO ESTEJA NO SENTIDO ERRADO
    
  Serial.begin(9600);  

  delay(100);

} 

void loop() { 
  
  tempoEsperaEX = millis();     //nao sei se ha problema ou nao em deixar os millis aqui
  tempoEsperaZ = millis();      

  if (Serial.available()) {                    // 'beffier' if command central script // 
    comando = Serial.readStringUntil('\n');   // cmds -> atv, zr, zpi, emr, esc      //  
    comando.trim();                          // zu, zx, zy, zz, esczr               // 
    comando.toLowerCase();  
 
    ///////////////////////////////////////////////////////////////////////
    
    if (Yend == true){
      estadoatual = MOVENDO_Z;
    }

    /*if (comando == "atv" && estadoatual == STAND_BY) { 

      estadoatual = MOVENDO_Z;  
      Serial.println(" | ELEVADOR EM MOVIMENTO | "); 

    } else if (comando == "stop" && estadoatual != STAND_BY) {
  
      estadoatual = EMER_STT; 
  
    } else if (comando == "zera" && estadoatual == STAND_BY) {
      
      estadoatual = ZERAMENTO;   
      zerr = true;             
      
    } else if (comando == "zu" && estadoatual == ZERAMENTO){    //Move todos os eixos/garras para a posicao inicial (0)
      
      Serial.println("zerando eixos..."); 
      homing_U();
      millis() - tempoEsperaEX >= 8000;   

    } else if (comando == "zz" && estadoatual == ZERAMENTO){    //Zera o eixo e garra Z 
      
      motorZ.moveTo(0);      
      motor2Z.moveTo(0);     
      motorZgarra.moveTo(0); 
      motorZYgarra.moveTo(0);
      abrirGarraBateria();    
      
      millis() - tempoEsperaEX >= 8000;
      
    } else if (comando == "esc" && estadoatual == ZERAMENTO){   //Sai desse modo
      
      estadoatual = STAND_BY;
      Serial.print("esc-ed ");
      Serial.print (estadoatual);
      Serial.println(" -> standing by");
      Serial.println(" | SAIU   DO ZERENCIAMENTO |");

    } else if (comando == "zpi" && estadoatual == ZERAMENTO){   
      
      ZERO_Z();                                     //define ponto zero | Eixo Z
      ZERO_Zvador();

    }*/

    ///////////////////////////////////////////////////////////////////////
  }

  //STAND_BY > ZERAMENTO > ATERRISSAGEM > HUNT > MOVENDO_Z >
  //TROCA_BATERIA > RETORNO_Z > RETORNO_Y >
  //EXPANSAO_X > STOP > EMER_STT

  switch (estadoatual) {
    case STAND_BY:

      printExecu = false;
      break;
      
    case ZERAMENTO:  

      if (!printExecu){
        Serial.println("em zerenciamento");
        printExecu = true;
      }
       
      break;

    case MOVENDO_Z:

      if (Zend == true) {
        pararZ();
        estadoatual = TROCA_BATERIA;

      } else {
        moverZ();
      }
      
      break; 
      
    case TROCA_BATERIA: 

      if (millis() - tempoEsperaZ >= 1000) {       //Talvez o tempo de espera tenha que ser a soma total do tempo interno dentro do if
        //Serial.println("...espera simulada..."); //10000 millis() | antes

        abrirGarraBateria();  //serve como garantia de que a garra estara aberta

        motorZYgarra.moveTo(passosZYgarra);  //extruda a garra Z
        millis() - tempoEsperaZ >= 6000;

        fecharGarraBateria();             //pega a bateria
        millis() - tempoEsperaZ >= 5000;  //espera a garra fechar

        motorZYgarra.moveTo(0);  //contrai a garra Z
        millis() - tempoEsperaZ >= 6000;

        motorZ.moveTo(0);  //desce pro armazem de baterias
        millis() - tempoEsperaZ >= 10000;

        motorZYgarra.moveTo(passosZYgarra);  //extruda a garra Z
        millis() - tempoEsperaZ >= 6000;

        abrirGarraBateria();  //solta a bateria velha num lugar 
        millis() - tempoEsperaZ >= 5000;

        fecharGarraBateria();  //pega a bateria carregada 
        millis() - tempoEsperaZ >= 5000;

        motorZYgarra.moveTo(0);  //contrai a garra Z 
        millis() - tempoEsperaZ >= 6000;

        motorZ.moveTo(passosZ);  //sobe pro drone novamente (com bateria cheia) 
        millis() - tempoEsperaZ >= 10000;

        motorZYgarra.moveTo(passosZYgarra);  //extruda a garra Z 
        millis() - tempoEsperaZ >= 6000;

        abrirGarraBateria();  //encaixa bateria carregada 
        millis() - tempoEsperaZ >= 5000;

        motorZYgarra.moveTo(0);  //contrai a garra Z 
        millis() - tempoEsperaZ >= 6000;

        estadoatual = RETORNO_Z;
      }
      
      break; 

    case RETORNO_Z:

      if (digitalRead(Zstart) == LOW) {
        pararZ();
        estadoatual = STAND_BY; //Envia resposta ao proximo arduino

      } else {

        motorZ.moveTo(0);
        motor2Z.moveTo(0);
        motorZgarra.moveTo(0);
        motorZYgarra.moveTo(0);
        abrirGarraBateria();
        
      }

      break;

     /////////////////////////////////////////////////////////////

    case EMER_STT:
       
      motorZ.stop();
      motor2Z.stop();
 
      motorZgarra.stop();
      motorZYgarra.stop();
 
      Serial.println("Parada EMER");
      estadoatual = STAND_BY;
      Serial.println(" -> standing by");

      break;
    case ATERRISSAGEM:
      break;
    case HUNT:
      break;
    case RETORNO_Y:
      break;
    case EXPANSAO_X:
      break;
    case STOP:
      break;
  }

  motorZ.run();
  motor2Z.run();

  motorZgarra.run();
  motorZYgarra.run();
  
}
