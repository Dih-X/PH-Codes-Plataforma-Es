#include <AccelStepper.h>

//Controle dos motores: --------------------------------------------------------------
//Y "Puxadores" do drone
const int Y1_STEP = 3;          //2
const int Y1_DIR = 6;           //3

const int Y2_STEP = 3;          //4
const int Y2_DIR = 6;           //5

//Empurradores Y

const int Y3_STEP = 3;          //2
const int Y3_DIR = 6;           //3

const int Y4_STEP = 3;          //4
const int Y4_DIR = 6;           //5

const int Ystart = 26;          //Conferir se eh viavel
const int Yend = 18;
const int YstartEmp = 28;       //Conferir se eh viavel  

AccelStepper motorY(AccelStepper::DRIVER, Y1_STEP, Y1_DIR);
AccelStepper motorYEmpurrao(AccelStepper::DRIVER, Y3_STEP, Y3_DIR);

AccelStepper motor2Y(AccelStepper::DRIVER, Y2_STEP, Y2_DIR);
AccelStepper motor2YEmpurrao(AccelStepper::DRIVER, Y4_STEP, Y4_DIR); 

long passosY = 1600;            //200 por padrao (sem M0,M1,M2)   | O quanto que o drone sera puxado              | Dist = ate o final - profundidade da base do drone (talvez nn seja nescessario)
long passosYempurrar = 1600;    //200 == 1 volta (sem os M's (ON) | O quanto que o drone sera empurrado           | Dist = ate o comeco - profundidade da base do drone

//Velocidades////////////////////////////////////////////////////

const float VEL_MAX = 800.0;  //acho q so vai ate 1000 (1k)
const float ACEL = 200.0;

enum EstadoAtualMotores{
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

EstadoAtualMotores estadoatual = STAND_BY; 

void ZERO_Y(){
    if (Ystart == true){
        motorY.setCurrentPosition(0);
        motorYEmpurrao.setCurrentPosition(0);

        motor2Y.setCurrentPosition(0);
        motor2YEmpurrao.setCurrentPosition(0);
    }
}

void homing_U(){         //reset Universal
  
  motorY.moveTo(0);
  motorYEmpurrao.moveTo(0);

  motor2Y.moveTo(0);
  motor2YEmpurrao.moveTo(0);
    
}

void moverY (){               //vao se mover uma distancia e terao que parar, o fim de curso soh acionara quando o drone estiver sendo levado junto
  motorY.moveTo(passosY);     //do contrario nao havera o contato do fim de curso e pode dar ruim, por isso ele tera que ter uma distancia de deslocamento limitada
  motor2Y.moveTo(passosY);    //Ha fim de curso na origem (obviamente)
}

void moverYEmpurrar (){       //vao se mover uma distancia e terao que parar, sem fim de curso, apenas na volta (origem) 
  motorYEmpurrao.moveTo(passosYempurrar);
  motor2YEmpurrao.moveTo(passosYempurrar);
}

void pararY(){
    if (Yend == true){
        motorY.moveTo(motorY.currentPosition());
        motor2Y.moveTo(motorY.currentPosition());
    }
}
 
void pararYempurra(){
    if (Yend == true){
        motorYEmpurrao.moveTo(motorYEmpurrao.currentPosition());
        motor2YEmpurrao.moveTo(motorYEmpurrao.currentPosition());
    }
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
