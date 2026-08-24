/*
Verifique se o código está funcional e se ele é capaz de receber a
informação de distância a ser percorrida e calcular e andar tal distância
baseado no calculo do diametro do eixo, ou aparato que estiver no eixo do
motor (uma engrenagem por exemplo) e a quantidade de passos que o motor
leva para dar uma volta completa, 200 passos no nosso caso. Crie um novo
arquivo na mesma pasta com o mesmo nome do original mas com numerção
de 3.1
*/

#include <AccelStepper.h>

const int Z1_STEP = 3;
const int Z1_DIR = 6;
const int Z2_STEP = 4;
const int Z2_DIR = 7;
const int PINO_ENABLE = 8;

const long PASSOS_POR_VOLTA = 200;
const float VEL_MAX = 800.0;
const float ACEL = 400.0;
const float PI_APROXIMADO = 3.14159265;

AccelStepper motorZ(AccelStepper::DRIVER, Z1_STEP, Z1_DIR);
AccelStepper motor2Z(AccelStepper::DRIVER, Z2_STEP, Z2_DIR);

enum EstadoDuploMotor {
  STAND_BY,
  MOVENDO,
  SELECT,
  ZERADOR
};

EstadoDuploMotor estadoAtual = STAND_BY;
long passosAlvo = 0;
long passosPorAvanco = PASSOS_POR_VOLTA;
float diametroMm = 0.0;
float distanciaMm = 0.0;

void moverPara(long alvo) {
  passosAlvo = alvo;
  motorZ.moveTo(passosAlvo);
  motor2Z.moveTo(passosAlvo);
  estadoAtual = MOVENDO;
}

bool movimentoConcluido() {
  return motorZ.distanceToGo() == 0 && motor2Z.distanceToGo() == 0;
}

bool configurarMovimento(String parametros) {
  int separador = parametros.indexOf(' ');
  if (separador < 0) {
    return false;
  }

  String diametroTexto = parametros.substring(0, separador);
  String distanciaTexto = parametros.substring(separador + 1);
  diametroTexto.trim();
  distanciaTexto.trim();

  float novoDiametroMm = diametroTexto.toFloat();
  float novaDistanciaMm = distanciaTexto.toFloat();
  if (novoDiametroMm <= 0.0 || novaDistanciaMm <= 0.0) {
    return false;
  }

  float circunferenciaMm = PI_APROXIMADO * novoDiametroMm;
  float passosCalculados = (novaDistanciaMm / circunferenciaMm) * PASSOS_POR_VOLTA;
  long novoAlvo = (long)(passosCalculados + 0.5);
  if (novoAlvo < 1) {
    novoAlvo = 1;
  }

  diametroMm = novoDiametroMm;
  distanciaMm = novaDistanciaMm;
  passosAlvo = novoAlvo;
  passosPorAvanco = PASSOS_POR_VOLTA;

  Serial.print("Configurado: ");
  Serial.print(passosAlvo);
  Serial.println(" passos.");
  return true;
}

void processarComando(String comando) {
  if (comando.startsWith("config ")) {
    if (estadoAtual == STAND_BY) {
      if (configurarMovimento(comando.substring(7))) {
        Serial.println("Diametro e distancia aceitos.");
      } else {
        Serial.println("Uso: config <diametro_mm> <distancia_mm>");
      }
    } else {
      Serial.println("Configure somente parado.");
    }
    return;
  }

  if (comando == "str" && estadoAtual == STAND_BY) {
    if (passosAlvo > 0) {
      moverPara(passosAlvo);
      Serial.println("Movendo...");
    } else {
      Serial.println("Configure primeiro: config <diametro_mm> <distancia_mm>");
    }
  } else if (comando == "adv" && estadoAtual == SELECT) {
    moverPara(passosAlvo + passosPorAvanco);
    Serial.println("Avanco de uma volta.");
  } else if (comando == "rtrn") {
    moverPara(0);
    estadoAtual = ZERADOR;
    Serial.println("Retornando ao zero...");
  }
}

void setup() {
  pinMode(PINO_ENABLE, OUTPUT);
  digitalWrite(PINO_ENABLE, LOW);

  motorZ.setMaxSpeed(VEL_MAX);
  motorZ.setAcceleration(ACEL);
  motor2Z.setMaxSpeed(VEL_MAX);
  motor2Z.setAcceleration(ACEL);

  motorZ.setPinsInverted(false, true, false);
  motor2Z.setPinsInverted(true, false, true);
  motorZ.setCurrentPosition(0);
  motor2Z.setCurrentPosition(0);

  Serial.begin(9600);
  Serial.println("Pronto.");
  Serial.println("config <diametro_mm> <distancia_mm>");
  Serial.println("str | adv | rtrn");
}

void loop() {
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toLowerCase();
    processarComando(comando);
  }

  motorZ.run();
  motor2Z.run();

  if (estadoAtual == MOVENDO && movimentoConcluido()) {
    estadoAtual = SELECT;
    Serial.println("Movimento concluido. Use adv ou rtrn.");
  } else if (estadoAtual == ZERADOR && movimentoConcluido()) {
    passosAlvo = 0;
    motorZ.setCurrentPosition(0);
    motor2Z.setCurrentPosition(0);
    estadoAtual = STAND_BY;
    Serial.println("Posicao zero.");
  }
}