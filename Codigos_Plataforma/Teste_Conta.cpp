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

int passosX = distanciaX / circunferencia_X;
int passosY = distanciaY / circunferencia_Y;
int passosZ = distanciaZ / circunferencia_Z;

void setup(){
    Serial.begin(9600);
}

void loop(){
    Serial.print("Passos X: ");
    Serial.println(passosX);
    Serial.print("Passos Y: ");
    Serial.println(passosY);
    Serial.print("Passos Z: ");
    Serial.println(passosZ);
    
    delay(1000); // Aguarda 1 segundo antes de repetir
}