#include <AccelStepper.h>

#if defined(HAVE_HWSERIAL1)
HardwareSerial &comPort = Serial1;
#else
HardwareSerial &comPort = Serial;
#endif

//Controle dos motores: --------------------------------------------------------------
//X - Alinhadores da pinça
const int X1_STEP = 2;
const int X1_DIR = 5;
const int X2_STEP = 2;
const int X2_DIR = 5;

//SENSORES---------------------------------------------------------------------------
//Inicio
const int Xstart = 16;
const int X2start = 16;

//Final
const int Xend = 17;
const int X2end = 17;

//Fim SENSORES------------------------------------------------------------------
const int pinoEnable = 8;
const int BotaoStart = 30;
const int BotaoStop = 31;
const int BotaoReset = 32;
const int sensorPouso = A0;

AccelStepper motorX(AccelStepper::DRIVER, X1_STEP, X1_DIR);
AccelStepper motor2X(AccelStepper::DRIVER, X2_STEP, X2_DIR);

String comando = "";
bool printExecu = false;
long passosX = 800;

//Velocidades////////////////////////////////////////////////////
const float VEL_MAX = 800.0;
const float ACEL = 200.0;

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
  Serial.print("SubX recebeu estado remoto: ");
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

void ZERO_X() {
  if (digitalRead(Xstart) == LOW) {
    motorX.setCurrentPosition(0);
    motor2X.setCurrentPosition(0);
  }
}

void homing_U() {
  motorX.moveTo(0);
  motor2X.moveTo(0);
}

void moverX() {
  motorX.moveTo(passosX);
  motor2X.moveTo(passosX);
}

void pararX() {
  if (digitalRead(Xend) == LOW && digitalRead(X2end) == LOW) {
    motorX.moveTo(motorX.currentPosition());
    motor2X.moveTo(motor2X.currentPosition());
  }
}

void setup() {
  pinMode(pinoEnable, OUTPUT);
  digitalWrite(pinoEnable, LOW);

  pinMode(BotaoStart, INPUT_PULLUP);
  pinMode(BotaoStop, INPUT_PULLUP);
  pinMode(BotaoReset, INPUT_PULLUP);

  motorX.setMaxSpeed(VEL_MAX);
  motorX.setAcceleration(ACEL);

  motor2X.setMaxSpeed(VEL_MAX);
  motor2X.setAcceleration(ACEL);

  pinMode(Xstart, INPUT_PULLUP);
  pinMode(Xend, INPUT_PULLUP);
  pinMode(X2start, INPUT_PULLUP);
  pinMode(X2end, INPUT_PULLUP);

  motorX.setPinsInverted(false, true, false);
  motor2X.setPinsInverted(true, false, true);

  Serial.begin(9600);
  comPort.begin(9600);

  Serial.println("SubX ativo");
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
      ZERO_X();
      break;

    case ATERRISSAGEM:
      if (!printExecu) {
        Serial.println("Aguardando comando mestre");
        printExecu = true;
      }
      break;

    case HUNT:
      if (!printExecu) {
        Serial.println("Hunting...");
        printExecu = true;
      }
      if (digitalRead(Xend) == LOW && digitalRead(X2end) == LOW) {
        pararX();
        estadoatual = MOVENDO_Z;
      } else {
        moverX();
      }
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
      moverX();
      break;

    case STOP:
      pararX();
      Serial.println("Parada normal");
      estadoatual = STAND_BY;
      break;

    case EMER_STT:
      motorX.stop();
      motor2X.stop();
      Serial.println("Parada EMER");
      estadoatual = STAND_BY;
      break;
  }

  motorX.run();
  motor2X.run();
}
