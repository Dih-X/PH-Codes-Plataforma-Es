#include <Servo.h>

Servo meuServo;

const int pinoServo = 9;

void setup(){
    meuServo.attach(pinoServo);
}

void loop(){
    for (int posicao = 0; posicao <= 180; posicao += 1){
        meuServo.write(posicao);
        delay(15);
    }

    for (int posicao = 180; posicao >= 0; posicao -= 1){
        meuServo.write(posicao);
        delay(15);
    }
}
