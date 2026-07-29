long passosX = 200;
long passosY = 200;
long passosZ = 200;

long passosZgarra = 200;    
long passosZYgarra = 200;   
long passosYempurrar = 200; 

float valor_diametroX;      //Possivelmente seria melhor transformar em uma funcao
float valor_diametroY;      //Ter atencao ao diametro do eixo + o da engrenagem adicional
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


    if (comando == "zu" && estadoatual == PERSONALIZACAO){    //Move todos os eixos/garras para a posicao inicial (0)
        Serial.println("zerando eixos...");
        homing_U();

    } else if (comando == "zx" && estadoatual == PERSONALIZACAO){    //Zera o eixo garra X
        motorX.moveTo(0);

    } else if (comando == "zy" && estadoatual == PERSONALIZACAO){    //Zera os eixos Y
        motorY.moveTo(0);

    } else if (comando == "zz" && estadoatual == PERSONALIZACAO){    //Zera o eixo e garra Z 
        motorZ.moveTo(0);

    } else if (comando == "zpi" && estadoatual == PERSONALIZACAO){
        ZERO_X();                                               

    } else if (comando == "esc" && estadoatual == PERSONALIZACAO){   //Sai desse modo
        estadoatual = STAND_BY;
        Serial.println("esc-ed {estadoatual} -> standing by");
        Serial.println(" | SAIU   DO ZERENCIAMENTO |");

case PERSONALIZACAO: //Estado de espera onde comandos podem ser executados

    if (!printExecu){
        Serial.println("em AJUSTES");
        printExecu = true;
    }

    break;
