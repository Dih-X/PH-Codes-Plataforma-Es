#include <AccelStepper.h>

#if defined(HAVE_HWSERIAL1)
HardwareSerial &comPort = Serial1;
#else
HardwareSerial &comPort = Serial;
#endif

//Controle dos motores: --------------------------------------------------------------
//Y "Puxadores" do drone
const int Y1_STEP = 3;
const int Y1_DIR = 6;

const int Y2_STEP = 3;
const int Y2_DIR = 6;

//Empurradores Y
const int Y3_STEP = 3;
const int Y3_DIR = 6;

const int Y4_STEP = 3;
const int Y4_DIR = 6;

const int Ystart = 26;
const int Yend = 18;
const int YstartEmp = 28;

const int pinoEnable = 8;
const int BotaoStart = 30;
const int BotaoStop = 31;
const int BotaoReset = 32;
const int sensorPouso = A0;

AccelStepper motorY(AccelStepper::DRIVER, Y1_STEP, Y1_DIR);
AccelStepper motorYEmpurrao(AccelStepper::DRIVER, Y3_STEP, Y3_DIR);
AccelStepper motor2Y(AccelStepper::DRIVER, Y2_STEP, Y2_DIR);
AccelStepper motor2YEmpurrao(AccelStepper::DRIVER, Y4_STEP, Y4_DIR);

long passosY = 1600;
long passosYempurrar = 1600;

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
String comando = "";
bool printExecu = false;
unsigned long tempoEsperaExp = 0;

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
    msg.trim();
    msg.toUpperCase();
    Serial.print("SubY recebeu estado remoto: ");
    Serial.println(msg);
  } else if (msg.startsWith("ACK:")) {
    Serial.println("ACK recebido: " + msg);
  }
}

void processarComandosRemotos() {
  while (comPort.available()) {
    String msg = comPort.readStringUntil('\n');
    msg.trim();
    if (msg.length() == 0) {
      continue;
    }
    aplicarEstadoRemoto(msg);
  }
}

void ZERO_Y() {
  if (digitalRead(Ystart) == LOW) {
    motorY.setCurrentPosition(0);
    motorYEmpurrao.setCurrentPosition(0);
    motor2Y.setCurrentPosition(0);
    motor2YEmpurrao.setCurrentPosition(0);
  }
}

void homing_U() {
  motorY.moveTo(0);
  motorYEmpurrao.moveTo(0);
  motor2Y.moveTo(0);
  motor2YEmpurrao.moveTo(0);
}

void moverY() {
  motorY.moveTo(passosY);
  motor2Y.moveTo(passosY);
}

void moverYEmpurrar() {
  motorYEmpurrao.moveTo(passosYempurrar);
  motor2YEmpurrao.moveTo(passosYempurrar);
}

void pararY() {
  if (digitalRead(Yend) == LOW) {
    motorY.moveTo(motorY.currentPosition());
    motor2Y.moveTo(motor2Y.currentPosition());
  }
}

void pararYempurra() {
  if (digitalRead(Yend) == LOW) {
    motorYEmpurrao.moveTo(motorYEmpurrao.currentPosition());
    motor2YEmpurrao.moveTo(motor2YEmpurrao.currentPosition());
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

  motor2Y.setMaxSpeed(VEL_MAX);
  motor2Y.setAcceleration(ACEL);
  motor2YEmpurrao.setMaxSpeed(VEL_MAX);
  motor2YEmpurrao.setAcceleration(ACEL);

  pinMode(Ystart, INPUT_PULLUP);
  pinMode(Yend, INPUT_PULLUP);
  pinMode(YstartEmp, INPUT_PULLUP);

  motorY.setPinsInverted(false, true, false);
  motorYEmpurrao.setPinsInverted(false, true, false);
  motor2Y.setPinsInverted(true, false, true);
  motor2YEmpurrao.setPinsInverted(true, false, true);

  Serial.begin(9600);
  comPort.begin(9600);

  Serial.println("SubY principal ativo");
}

void loop() {
  processarComandosRemotos();

  if (Serial.available()) {
    comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toLowerCase();

    if (comando == "atv" && estadoatual == STAND_BY) {
      Serial.println(" | ESPERANDO POUSO...      | ");
      estadoatual = ATERRISSAGEM;
      enviarEstadoRemoto(estadoatual);
    } else if (comando == "zr" && estadoatual == STAND_BY) {
      estadoatual = ZERAMENTO;
      enviarEstadoRemoto(estadoatual);
      Serial.println(" | ENTROU NO ZERENCIAMENTO | ");
    } else if (comando == "emr") {
      estadoatual = EMER_STT;
      enviarEstadoRemoto(estadoatual);
      Serial.println(" | PARADA DE EMERGENCIA    | ");
    } else if (comando == "esc") {
      estadoatual = STAND_BY;
      enviarEstadoRemoto(estadoatual);
      Serial.println(" | SAIU   DO ZERENCIAMENTO | ");
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

    case ATERRISSAGEM:
      if (digitalRead(sensorPouso) == HIGH) {
        if (!printExecu) {
          Serial.println("Aguardando pouso...");
          printExecu = true;
        }
      } else if (digitalRead(sensorPouso) == LOW) {
        Serial.println("Drone pousou");
        estadoatual = HUNT;
        enviarEstadoRemoto(estadoatual);
      }
      break;

    case HUNT:
      if (!printExecu) {
        Serial.println("Hunting...");
        printExecu = true;
      }
      if (digitalRead(Yend) == LOW) {
        pararY();
        estadoatual = MOVENDO_Z;
        enviarEstadoRemoto(estadoatual);
      } else {
        moverY();
      }
      break;

    case MOVENDO_Z:
      break;

    case TROCA_BATERIA:
      break;

    case RETORNO_Z:
      if (digitalRead(Ystart) == LOW) {
        pararY();
        estadoatual = RETORNO_Y;
        enviarEstadoRemoto(estadoatual);
      } else {
        motorY.moveTo(0);
        motor2Y.moveTo(0);
      }
      break;

    case RETORNO_Y:
      if (digitalRead(Ystart) == LOW) {
        pararY();
        pararYempurra();
        estadoatual = EXPANSAO_X;
        enviarEstadoRemoto(estadoatual);
      } else {
        motorY.moveTo(0);
        motor2Y.moveTo(0);
        motorYEmpurrao.moveTo(passosYempurrar);
        motor2YEmpurrao.moveTo(passosYempurrar);
      }
      tempoEsperaExp = millis();
      break;

    case EXPANSAO_X:
      if (millis() - tempoEsperaExp >= 10000) {
        if (digitalRead(Ystart) == LOW) {
          motorYEmpurrao.moveTo(0);
          motor2YEmpurrao.moveTo(0);
          if (digitalRead(YstartEmp) == LOW) {
            pararYempurra();
          }
          estadoatual = STOP;
          enviarEstadoRemoto(estadoatual);
        }
      }
      break;

    case STOP:
      pararY();
      pararYempurra();
      Serial.println("Parada normal");
      estadoatual = STAND_BY;
      enviarEstadoRemoto(estadoatual);
      break;

    case EMER_STT:
      motorY.stop();
      motorYEmpurrao.stop();
      motor2Y.stop();
      motor2YEmpurrao.stop();
      Serial.println("Parada EMER");
      estadoatual = STAND_BY;
      enviarEstadoRemoto(estadoatual);
      break;
  }

  motorY.run();
  motor2Y.run();
  motorYEmpurrao.run();
  motor2YEmpurrao.run();
}
