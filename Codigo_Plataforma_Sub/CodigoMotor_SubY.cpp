#include <AccelStepper.h>

//Controle dos motores: --------------------------------------------------------------
//Y "Puxadores" do drone
const int Y1_STEP = 3;  //2
const int Y1_DIR = 6;   //3

const int Y2_STEP = 3;  //4
const int Y2_DIR = 6;   //5

//Empurradores Y

const int Y3_STEP = 3;  //2
const int Y3_DIR = 6;   //3

const int Y4_STEP = 3;  //4
const int Y4_DIR = 6;   //5

const int Ystart = 26;  //Conferir se eh viavel
const int Yend = 18;
const int YstartEmp = 28;  //Conferir se eh viavel

AccelStepper motorY(AccelStepper::DRIVER, Y1_STEP, Y1_DIR);
AccelStepper motorYEmpurrao(AccelStepper::DRIVER, Y3_STEP, Y3_DIR);

AccelStepper motor2Y(AccelStepper::DRIVER, Y2_STEP, Y2_DIR);
AccelStepper motor2YEmpurrao(AccelStepper::DRIVER, Y4_STEP, Y4_DIR);

const int sensorPouso = A0; 

long passosY = 1600;          //200 por padrao (sem M0,M1,M2)   | O quanto que o drone sera puxado              | Dist = ate o final - profundidade da base do drone (talvez nn seja nescessario)
long passosYempurrar = 1600;  //200 == 1 volta (sem os M's (ON) | O quanto que o drone sera empurrado           | Dist = ate o comeco - profundidade da base do drone

//Velocidades////////////////////////////////////////////////////

const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 200.0;

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

void ZERO_Y() {
  if (Ystart == true) {
    motorY.setCurrentPosition(0);
    motorYEmpurrao.setCurrentPosition(0);

    motor2Y.setCurrentPosition(0);
    motor2YEmpurrao.setCurrentPosition(0);
  }
}

void homing_U() {  //reset Universal
  motorY.moveTo(0);
  motorYEmpurrao.moveTo(0);

  motor2Y.moveTo(0);
  motor2YEmpurrao.moveTo(0);
}

void moverY() {             //vao se mover uma distancia e terao que parar, o fim de curso soh acionara quando o drone estiver sendo levado junto
  motorY.moveTo(passosY);   //do contrario nao havera o contato do fim de curso e pode dar ruim, por isso ele tera que ter uma distancia de deslocamento limitada
  motor2Y.moveTo(passosY);  //Ha fim de curso na origem (obviamente)
}

void moverYEmpurrar() {  //vao se mover uma distancia e terao que parar, sem fim de curso, apenas na volta (origem)
  motorYEmpurrao.moveTo(passosYempurrar);
  motor2YEmpurrao.moveTo(passosYempurrar);
}

void pararY() {
  if (Yend == true) {
    motorY.moveTo(motorY.currentPosition());
    motor2Y.moveTo(motorY.currentPosition());
  }
}

void pararYempurra() {
  if (Yend == true) {
    motorYEmpurrao.moveTo(motorYEmpurrao.currentPosition());
    motor2YEmpurrao.moveTo(motorYEmpurrao.currentPosition());
  }
}

void setup() {

  pinMode(pinoEnable, OUTPUT);
  digitalWrite(pinoEnable, LOW);

  pinMode(BotaoStart, INPUT_PULLUP);
  pinMode(BotaoStop, INPUT_PULLUP);
  pinMode(BotaoReset, INPUT_PULLUP);

  motorY.setMaxSpeed(VEL_MAX);
  motorY.setAcceleration(ACEL);

  motorYEmpurrao.setMaxSpeed(VEL_MAX);
  motorYEmpurrao.setAcceleration(ACEL);

  motor2Y.setMaxSpeed(VEL_MAX);   //2M
  motor2Y.setAcceleration(ACEL);  //2M

  motor2YEmpurrao.setMaxSpeed(VEL_MAX);
  motor2YEmpurrao.setAcceleration(ACEL);

  pinMode(Ystart, INPUT_PULLUP);
  pinMode(Yend, INPUT_PULLUP);

  motorY.setPinsInverted(false, true, false);  //direcao "normal"
  motorYEmpurrao.setPinsInverted(false, true, false);

  motor2Y.setPinsInverted(true, false, true);  //inverte a direcao usando AccelStepper
  motorYEmpurrao.setPinsInverted(true, false, true);

  Serial.begin(9600);

  Serial.println("ATV, ZR, EMR, ESC");
  Serial.println("zpi, zu, zx, zy, zz");
}   
    
void loop() {
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  if (Serial.available()) {                  // 'beffier' if command central script
    comando = Serial.readStringUntil('\n');  // cmds -> atv, zr, zpi, emr, esc
    comando.trim();                          // zu, zx, zy, zz, esczr
    comando.toLowerCase();                   //

    /////////////////////////////////////////////////////////////////////////////
    
    if (comando == "atv" && estadoatual == STAND_BY) {
      Serial.println(" | ESPERANDO POUSO...      |");
      estadoatual = ATERRISSAGEM;

    } else if (comando == "atv" && estadoatual != STAND_BY) {
      Serial.println(" | JA EM BUSCA DO Hy-D-J   |");

    } else if (comando == "zr" && estadoatual == STAND_BY) {
      estadoatual = ZERAMENTO;
      Serial.println(" | ENTROU NO ZERENCIAMENTO |");

    } else if (comando == "zr" && estadoatual != STAND_BY) {

      Serial.println(" | NAO EH POSSIVEL AGORA   |");

      /////////////////////////////////////////////////////////////////////////////

    } else if (comando == "emr" && estadoatual != STOP) {
      estadoatual = EMER_STT;
      Serial.println(" | PARADA DE EMERGENCIA    |");

    } else if (comando == "emr" && estadoatual == STOP) {
      estadoatual = EMER_STT;
      Serial.println(" | JA ESTA PARADO          |");

      /////////////////////////////////////////////////////////////////////////////

    } else if (comando == "zu" && estadoatual == ZERAMENTO) {  //Move todos os eixos/garras para a posicao inicial (0)
      Serial.println("zerando eixos...");
      homing_U();

    } else if (comando == "zx" && estadoatual == ZERAMENTO) {  //Zera o eixo garra X
      motorX.moveTo(0);

    } else if (comando == "zy" && estadoatual == ZERAMENTO) {  //Zera os eixos Y
      motorY.moveTo(0);

    } else if (comando == "zz" && estadoatual == ZERAMENTO) {  //Zera o eixo e garra Z
      motorZ.moveTo(0);

    } else if (comando == "zpi" && estadoatual == ZERAMENTO) {  //&& (motorX.currentPosition != motorX.setCurrentPosition)
      ZERO_X();                                                 //define ponto zero | Garra X
      ZERO_Y();                                                 //define ponto zero | Barra Y [G.X], e Barra Y [EMP]
      ZERO_Z();                                                 //define ponto zero | Eixo Z
      ZERO_Zvador();                                            //define zero point | Garra e Extensao Z

    } else if (comando == "esc" && estadoatual == ZERAMENTO) {  //Sai desse modo
      estadoatual = STAND_BY;
      Serial.println("esc-ed {estadoatual} -> standing by");
      Serial.println(" | SAIU   DO ZERENCIAMENTO |");

      /////////////////////////////////////////////////////////////////////////////

    } else if (comando == "esc" && estadoatual != STAND_BY) {
      estadoatual = STAND_BY;
      Serial.println("quit {estadoatual} -> standing by");
      //std::cout << "Exited " << estadoatual << " state\n"

    } else {
      Serial.println(" | comando desconhecido    |");

      //else if (comando == any && estadoatual == ){
      //Serial.println(" | comando desconhecido    |");
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //STAND_BY > ZERAMENTO > ATERRISSAGEM > HUNT > MOVENDO_Z >
  //TROCA_BATERIA > RETORNO_Z > RETORNO_Y >
  //EXPANSAO_X > STOP > EMER_STT

  switch (estadoatual) {

    case STAND_BY:  //Estado de espera onde comandos podem ser executados

      printExecu = false;
      break;

    case ZERAMENTO:  //Estado de gerenciamento dos pontos iniciais | mover para pontos iniciais | zr -> zu, zx, zy, zz, zpi, esczr ->

      if (!printExecu) {
        Serial.println("em zerenciamento");
        printExecu = true;
      }

      break;

    case ATERRISSAGEM:  //Estado de espera onde o codigo esta em execucao

      if (comando == "esc" && estadoatual != STAND_BY) {  //Abortar espera de aterrissagem (volta pra stand by)
        Serial.println("Leaving from landing waiting sequence");
        estadoatual = STAND_BY;

      } else {
        if (digitalRead(sensorPouso) == HIGH) {  //Verifica se o drone pousou ou nao
          //Verificar se funciona dps
          if (!printExecu) {
            Serial.println("Aguardando pouso...");
            printExecu = true;
          }

        } else if (digitalRead(sensorPouso) == LOW) {  //drone pousou -> estado passa para Hunt
          Serial.println("Drone pousou");
          estadoatual = HUNT;                          //Enviar Sinal RX/TX para acionar Sub. Y (???)
        }
      }

      //sensorPouso == LOW; //para testar
      break;

    case HUNT:  //Move a barra Y com as Garras X simultaneamente em direcao a barra de empurrao Y para capturar o drone e alinha-lo

      if (!printExecu) {
        Serial.println("Hunting...");
        printExecu = true;
      }

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      if (Yend == true) {         //Talvez seja necessario fazer com que o if acione um tempo depois que o motor partir para nao causar possiveis travamentos
        pararY();                 //Apos X tempo o if eh liberado e entao se eh possivel usar o fim de curso para parar o motor...
        estadoatual = MOVENDO_Z;  //Enviar Sinal RX/TX para acionar Sub. El. Z
      } else {
        moverY();                 //Enviar Sinal RX/TX para acionar Sub. X
      }

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      if (Xend == true && X2end == true) {  //a Garra X so para quando ambas as pincas tocam no drone dos dois lados, cada uma de cada lado
        pararX();
      } else {
        moverX();
      }

      break;

    case MOVENDO_Z:

      if (Zend == true) {
        pararZ();
        tempoEsperaZ = millis();
        estadoatual = TROCA_BATERIA;
      } else {
        moverZ();
      }

      break;
      
    case TROCA_BATERIA:
      break;

    case RETORNO_Z:

      if (digitalRead(Zstart) == LOW) {
        pararZ();
        estadoatual = RETORNO_Y;

      } else {
        motorZ.moveTo(0);
        motor2Z.moveTo(0);
      }

      break;

    case RETORNO_Y:  //Braco Principal

      if (digitalRead(Ystart) == LOW) {
        pararY();
        pararYempurra();

        estadoatual = EXPANSAO_X;

      } else {
        motorY.moveTo(0);
        motor2Y.moveTo(0);

        motorYEmpurrao.moveTo(passosYempurrar);   //terao que parar sozinhos apos a distancia certa ter sido percorrida
        motor2YEmpurrao.moveTo(passosYempurrar);  //na vdd poderam parar junto com BY1c/GX visto que andaram juntos
      }

      /*motorY.moveTo(0);
        if(digitalRead(Ystart) == LOW){     //barra Y empurrao
          pararY();
          estadoatual = EXPANSAO_X;
        }*/

      tempoEsperaExp = millis();

      break;

    case EXPANSAO_X:
      if (millis() - tempoEsperaExp >= 10000) {
        millis() - tempoEsperaExp >= 2000;

        if (digitalRead(Ystart) == LOW) {  //so libera c o fim de curso Y
          motorX.moveTo(0);                //for ativado/pressionado

          motorYEmpurrao.moveTo(0);
          motor2YEmpurrao.moveTo(0);

          if (digitalRead(Xstart) == LOW) {  //falso nesse vosso caso
            pararX();
            estadoatual = STOP;
          }

          if (digitalRead(YstartEmp) == LOW) {
            pararYempurra();
          }
        }
      }
      break;

    case STOP:
      pararY();
      pararYempurra();

      Serial.println("Parada normal");
      Serial.println(" -> standing by");

      estadoatual = STAND_BY;
      break;

    case EMER_STT:
      motorY.stop();
      motorYEmpurrao.stop();
      
      motor2Y.stop();
      motor2YEmpurrao.stop();

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
