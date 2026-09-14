#include <Arduino.h>
#include <time.h>

long passosX = 200;
long passosY = 200;
long passosZ = 200;

long passosZgarra = 200;    
long passosZYgarra = 200;   
long passosYempurrar = 200; 

float valor_diametroX;
float valor_diametroY;
float valor_diametroZ;

float diametroX = valor_diametroX;
float diametroY = valor_diametroY;
float diametroZ = valor_diametroZ;

float circunferencia_X = 3.14 * diametroX;
float circunferencia_Y = 3.14 * diametroY;
float circunferencia_Z = 3.14 * diametroZ;

int distanciaX = passosX * circunferencia_X;
int distanciaY = passosY * circunferencia_Y;
int distanciaZ = passosZ * circunferencia_Z;

int passosX_Adar = distanciaX / circunferencia_X;
int passosY_Adar = distanciaY / circunferencia_Y;
int passosZ_Adar = distanciaZ / circunferencia_Z;

void setup(){
    Serial.begin(9600);
}

void loop(){

    Serial.print("Dados X: ");
    Serial.println(passosX_Adar);
    Serial.println(diametroX);
    Serial.println(circunferencia_X);
    Serial.println(distanciaX);

    Serial.print("Dados Y: ");
    Serial.println(passosY_Adar);
    Serial.println(diametroY);
    Serial.println(circunferencia_Y);
    Serial.println(distanciaY);

    Serial.print("Dados Z: ");
    Serial.println(passosZ_Adar);
    Serial.println(diametroZ);
    Serial.println(circunferencia_Z);
    Serial.println(distanciaZ);

    delay(1000);

}