#include <cmath>

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

String setagem = "";

enum Estado_Dados{
    Nonium,
    Diam_X,
    Diam_Y,
    Diam_Z,
};

Estado_Dados estadoDadosAgora = Nonium; 

void loop (){

    if (Serial.available()){

        Serial.println(" | Digite o valor do diametro do eixo X |");

        setagem = Serial.parseFloat();

        if (comando == "edx" && estadoatual == PERSONALIZACAO){
            
            estadoDadosAgora = Diam_X;
        
        }
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
    case PERSONALIZACAO: //Estado de espera onde comandos podem ser executados

        if (!printExecu){
            Serial.println(" em AJUSTES/PERSONALIZACAO ");      //fazendo igual ao ZERENCIAMENTO
            Serial.println(" cusX, cusY, cusZ, esc")            //no entanto com edicao/adicao dos
            printExecu = true;                                  //diametros dos eixos e calculos de distancia
            
        }
        
        if (comando == cusX){
            estadoDadosAgora = Diam_X;

        } else if (comando == "esc" && estadoatual == PERSONALIZACAO){
            estadoatual = STAND_BY;
            Serial.println(" | SAIU DA PERSONALIZACAO |");
            
        }

        switch (Estado_Dados){
            
            case Diam_X:

                if (Serial.available()){

                    Serial.println(" | Digite o valor do diametro do eixo X |");
                    setagem = Serial.parseFloat();

                    valor_diametroX = static_cast<int>(std::round(setagem));

                    //setagem = 0;

                } else if (comando == "esc" && estadoatual == PERSONALIZACAO){   //Sai desse modo
                    
                    estadoatual = STAND_BY;
                    
                    Serial.println(" | SAIU DA PERSONALIZACAO |");

                } else {

                    //estadoDadosAgora = Nonium;

                }

                break;

            case Diam_Y:

                if (Serial.available()){

                    Serial.println(" | Digite o valor do diametro do eixo Y |");
                    setagem = Serial.parseFloat();

                    valor_diametroY = static_cast<int>(std::round(setagem));

                } else if (comando == "esc" && estadoatual == PERSONALIZACAO){   //Sai desse modo
                    
                    estadoatual = STAND_BY;
                
                    Serial.println(" | SAIU DA PERSONALIZACAO |");

                } else {

                    //estadoDadosAgora = Nonium;

                }

                break;

            case Diam_Z:

                if (Serial.available()){

                    Serial.println(" | Digite o valor do diametro do eixo Z |");
                    setagem = Serial.parseFloat();

                    valor_diametroZ = static_cast<int>(std::round(setagem));

                    //setagem = 0;

                } else if (comando == "esc" && estadoatual == PERSONALIZACAO){   //Sai desse modo
                    
                    estadoatual = STAND_BY;
                    
                    Serial.println(" | SAIU DA PERSONALIZACAO |");

                } else {

                    //estadoDadosAgora = Nonium;

                }

                break;

        }
        
        break;

}