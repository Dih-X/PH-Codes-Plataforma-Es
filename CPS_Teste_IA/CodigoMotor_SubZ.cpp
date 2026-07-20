#include <AccelStepper.h>

#if defined(HAVE_HWSERIAL1)
HardwareSerial &comPort = Serial1;
#else
HardwareSerial &comPort = Serial;
#endif

const int Z1_STEP = 3;
const int Z1_DIR = 6;
const int Z2_STEP = 4;
const int Z2_DIR = 7;

//Empurrador da Garra Z
const int ZY_STEP = 99;
const int ZY_DIR = 98;

//Garras no Z trocam a bateria
const int Zgarra_STEP = 10;
const int Zgarra_DIR = 15;

const int Zstart = 27;
const int ZGstart = 28;
const int ZExstart = 29;
const int Yend = 33;
const int Zend = 32;
const int ZGend = 30;
const int ZExend = 31;

const int pinoEnable = 8;
const int BotaoStart = 30;
const int BotaoStop = 31;
const int BotaoReset = 32;

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
const float VEL_MAX = 800.0;
const float ACEL = 200.0;
bool zerr = false;
bool printExecu = false;

enum EstadoAtualMotores {
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

String estadoParaTexto(EstadoAtualMotores estado) {
  switch (estado) {
    case STAND_BY: return "STAND_BY";
    case ZERAMENTO: return "ZERAMENTO";
    case ATERRISSAGEM: return "ATERRISSAGEM";
    case HUNT: return "HUNT";
    case MOVENDO_Z: return "MOVENDO_Z";
    case TROCA_BATERIA: return "TROCA_BATERIA";
    case RETORNO_Z: return "RETORNO_Z";
    case RETORNO_Y: return "RETORNO_Y";
    case EXPANSAO_X: return "EXPANSAO_X";
    case STOP: return "STOP";
    case EMER_STT: return "EMER_STT";
  }
  return "STAND_BY";
}

EstadoAtualMotores estadoPorTexto(const String &texto) {
  String t = texto;
  t.trim();
  t.toUpperCase();
  if (t == "STAND_BY") return STAND_BY;
  if (t == "ZERAMENTO") return ZERAMENTO;
  if (t == "ATERRISSAGEM") return ATERRISSAGEM;
  if (t == "HUNT") return HUNT;
  if (t == "MOVENDO_Z") return MOVENDO_Z;
  if (t == "TROCA_BATERIA") return TROCA_BATERIA;
  if (t == "RETORNO_Z") return RETORNO_Z;
  if (t == "RETORNO_Y") return RETORNO_Y;
  if (t == "EXPANSAO_X") return EXPANSAO_X;
  if (t == "STOP") return STOP;
  if (t == "EMER_STT") return EMER_STT;
  return STAND_BY;
}

void enviarEstadoRemoto(EstadoAtualMotores novoEstado) {
  String mensagem = "STATE:" + estadoParaTexto(novoEstado);
  comPort.println(mensagem);
  Serial.println("TX -> " + mensagem);
}

void aplicarEstadoRemoto(const String &mensagem) {
  String msg = mensagem;
  msg.trim();
  if (msg.startsWith("STATE:")) {
    msg.remove(0, 6);
  }
  msg.trim();
  msg.toUpperCase();
  EstadoAtualMotores novoEstado = estadoPorTexto(msg);
  estadoatual = novoEstado;
  Serial.print("SubZ recebeu estado remoto: ");
  Serial.println(estadoParaTexto(estadoatual));
}

void processarComandosRemotos() {
  while (comPort.available()) {
    String msg = comPort.readStringUntil('\n');
    msg.trim();
    if (msg.length() == 0) {
      continue;
    }
    if (msg.startsWith("ACK:")) {
      Serial.println("ACK recebido: " + msg);
      continue;
    }
    aplicarEstadoRemoto(msg);
  }
}

void ZERO_Z() {
  if (digitalRead(Zstart) == LOW) {
    motorZ.setCurrentPosition(0);
    motor2Z.setCurrentPosition(0);
  }
}

void ZERO_Zvador() {
  if (digitalRead(ZGstart) == LOW) {
    motorZgarra.setCurrentPosition(0);
  }
  if (digitalRead(ZExstart) == LOW) {
    motorZYgarra.setCurrentPosition(0);
  }
}

void homing_U() {
  motorZ.moveTo(0);
  motor2Z.moveTo(0);
  motorZgarra.moveTo(0);
  motorZYgarra.moveTo(0);
}

void moverZ() {
  motorZ.moveTo(passosZ);
  motor2Z.moveTo(passosZ);
}

void pararZ() {
  if (digitalRead(Zend) == LOW) {
    motorZ.moveTo(motorZ.currentPosition());
    motor2Z.moveTo(motor2Z.currentPosition());
  }
}

void pararZgarra() {
  if (digitalRead(Zend) == LOW) {
    motorZgarra.moveTo(motorZgarra.currentPosition());
    motorZYgarra.moveTo(motorZYgarra.currentPosition());
  }
}

void fecharGarraBateria() {
  if (digitalRead(ZGend) == LOW) {
    motorZgarra.stop();
  } else {
    motorZgarra.moveTo(passosZgarra);
  }
}

void abrirGarraBateria() {
  if (digitalRead(ZGstart) == LOW) {
    motorZgarra.stop();
  } else {
    motorZgarra.moveTo(0);
  }
}

void setup() {
  pinMode(pinoEnable, OUTPUT);
  digitalWrite(pinoEnable, LOW);

  motorZ.setMaxSpeed(VEL_MAX);
  motorZ.setAcceleration(ACEL);
  motor2Z.setMaxSpeed(VEL_MAX);
  motor2Z.setAcceleration(ACEL);
  motorZgarra.setMaxSpeed(VEL_MAX);
  motorZgarra.setAcceleration(ACEL);

  pinMode(Zgarra_STEP, OUTPUT);
  pinMode(Zgarra_DIR, OUTPUT);
  pinMode(Zstart, INPUT_PULLUP);
  pinMode(Zend, INPUT_PULLUP);
  pinMode(ZGstart, INPUT_PULLUP);
  pinMode(ZGend, INPUT_PULLUP);
  pinMode(ZExstart, INPUT_PULLUP);
  pinMode(ZExend, INPUT_PULLUP);
  pinMode(Yend, INPUT_PULLUP);

  motorZ.setPinsInverted(false, true, false);
  motor2Z.setPinsInverted(true, false, true);
  motorZgarra.setPinsInverted(false, true, false);

  Serial.begin(9600);
  comPort.begin(9600);
  Serial.println("SubZ ativo");
}

void loop() {
  processarComandosRemotos();

  if (Serial.available()) {
    comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toLowerCase();
    if (comando == "atv") {
      estadoatual = ATERRISSAGEM;
    }
  }

  switch (estadoatual) {
    case STAND_BY:
      printExecu = false;
      break;

    case ZERAMENTO:
      if (!printExecu) {
        Serial.println("em zerenciamento");
        printExecu = true;
      }
      break;

    case MOVENDO_Z:
      if (digitalRead(Zend) == LOW) {
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

    /*case TROCA_BATERIA:
      if (millis() - tempoEsperaZ >= 1000) {
        abrirGarraBateria();
        motorZYgarra.moveTo(passosZYgarra);
        fecharGarraBateria();
        motorZYgarra.moveTo(0);
        motorZ.moveTo(0);
        estadoatual = RETORNO_Z;
      }
      break;*/

    case RETORNO_Z:
      if (digitalRead(Zstart) == LOW) {
        pararZ();
        estadoatual = STAND_BY;
      } else {
        motorZ.moveTo(0);
        motor2Z.moveTo(0);
        motorZgarra.moveTo(0);
        motorZYgarra.moveTo(0);
        abrirGarraBateria();
      }
      break;

    case EMER_STT:
      motorZ.stop();
      motor2Z.stop();
      motorZgarra.stop();
      motorZYgarra.stop();
      Serial.println("Parada EMER");
      estadoatual = STAND_BY;
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
