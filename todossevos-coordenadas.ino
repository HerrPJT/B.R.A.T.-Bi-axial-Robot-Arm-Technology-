#include <ESP32Servo.h>
#include <math.h>

// Definição dos pinos dos servos
const int pinoServoDeBaixo   = 17; // Ombro (Vertical inferior)
const int pinoServoDeCima    = 18; // Cotovelo (Vertical superior)
const int pinoServoDaBase    = 19; // Eixo de rotação da Base (Novo)
const int pinoServoDaGarra   = 21; // Atuador da Garra (Novo)

// Criação dos objetos Servo
Servo servoDeBaixo;
Servo servoDeCima;
Servo servoDaBase;
Servo servoDaGarra;

// Comprimentos dos braços em cm
const float L1 = 20.0;
const float L2 = 28.0;

// Limites físicos de alcance 3D (em cm)
const float DIST_MIN = 14.0;
const float DIST_MAX = 48.0;

// --- VARIÁVEIS PARA A SUAVIZAÇÃO ---
// Posições de destino (Alvos)
float alvoServoDeBaixo = 90.0;     
float alvoServoDeCima  = 90.0;       
float alvoServoDaBase  = 90.0;  // 90° assume que o robô começa virado para a frente
float alvoServoDaGarra = 90.0;  // Posição média/neutra da garra

// Posições em tempo real (Onde os servos estão a passar neste instante)
float atualServoDeBaixo = 90.0;    
float atualServoDeCima  = 90.0;      
float atualServoDaBase  = 90.0;
float atualServoDaGarra = 90.0;

const float FA_SUAVIZACAO = 0.005; 

unsigned long ultimoTempo = 0;
const int intervaloAtualizacao = 20; // 50Hz

void setup() {
  Serial.begin(9600);
  
  // Configuração dos 4 servos com tempos de pulso padrão para os TD8125
  servoDeBaixo.attach(pinoServoDeBaixo, 500, 2500);
  servoDeCima.attach(pinoServoDeCima, 500, 2500);
  servoDaBase.attach(pinoServoDaBase, 500, 2500);
  servoDaGarra.attach(pinoServoDaGarra, 500, 2500);
  
  // Inicialização segura na posição central/neutra
  servoDeBaixo.write((int)atualServoDeBaixo);
  servoDeCima.write((int)atualServoDeCima);
  servoDaBase.write((int)atualServoDaBase);
  servoDaGarra.write((int)atualServoDaGarra);
  
  Serial.println("Braço Robótico 3D Inicializado com Suavização.");
  Serial.println("Envia os comandos no formato: X,Y,Z,Garra");
  Serial.println("Exemplo (Frente, Centro, Cima, Aberta): 24,0,24,120");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float garra = 0.0;
    
    // Processa a leitura das 4 variáveis separadas por vírgula
    if (sscanf(input.c_str(), "%f,%f,%f,%f", &x, &y, &z, &garra) == 4) {
      atualizarAlvo3D(x, y, z, garra);
    } else if (input.length() > 0) {
      Serial.println("ERRO: Formato inválido! Usa: X,Y,Z,Garra");
    }
  }

  // Atualização síncrona e suave dos motores
  unsigned long tempoAtual = millis();
  if (tempoAtual - ultimoTempo >= intervaloAtualizacao) {
    ultimoTempo = tempoAtual;
    
    // Interpolação de movimento para os 4 servos
    atualServoDeBaixo += (alvoServoDeBaixo - atualServoDeBaixo) * FA_SUAVIZACAO;
    atualServoDeCima  += (alvoServoDeCima - atualServoDeCima) * FA_SUAVIZACAO;
    atualServoDaBase   += (alvoServoDaBase - atualServoDaBase) * FA_SUAVIZACAO;
    atualServoDaGarra  += (alvoServoDaGarra - atualServoDaGarra) * FA_SUAVIZACAO;
    
    // Envio dos sinais físicos para o hardware
    servoDeBaixo.write((int)atualServoDeBaixo);
    servoDeCima.write((int)atualServoDeCima);
    servoDaBase.write((int)atualServoDaBase);
    servoDaGarra.write((int)atualServoDaGarra);
  }
}

void atualizarAlvo3D(float x, float y, float z, float garra) {
  // 1. Calcular distância total em espaço 3D (Esfera de alcance)
  float d = sqrt(x*x + y*y + z*z);
  
  if (d < DIST_MIN || d > DIST_MAX) {
    Serial.print("ERRO: Coordenadas fora de alcance! Distancia linear: ");
    Serial.print(d); Serial.println(" cm");
    return; 
  }
  
  // 2. Calcular o ângulo do Servo da Base
  float thetaBase = atan2(y, x);
  float thetaBaseGraus = thetaBase * 180.0 / M_PI;

  // 3. Decompor para o plano vertical do braço (Raio projetado)
  float r = sqrt(x*x + y*y); 

  // 4. IK Planar usando R (como distância horizontal) e Z (como altura)
  float cosTheta2 = (r*r + z*z - L1*L1 - L2*L2) / (2.0 * L1 * L2);
  if (cosTheta2 < -1.0) cosTheta2 = -1.0;
  if (cosTheta2 > 1.0) cosTheta2 = 1.0;
  
  float sinTheta2 = -sqrt(1.0 - cosTheta2 * cosTheta2); // Cotovelo para Cima
  float theta2 = atan2(sinTheta2, cosTheta2); 
  
  float k1 = L1 + L2 * cosTheta2;
  float k2 = L2 * sinTheta2;
  float theta1 = atan2(z, r) - atan2(k2, k1); // atan2(z, r) substitui o antigo atan2(y, x)
  
  float theta1Graus = theta1 * 180.0 / M_PI;
  float theta2Graus = theta2 * 180.0 / M_PI;
  
  // 5. Mapeamento final e calibração dos alvos
  float novoAlvoServoDeBaixo = 180.0 - theta1Graus;
  float novoAlvoServoDeCima  = 90.0 + theta2Graus;
  float novoAlvoServoDaBase   = 90.0 + thetaBaseGraus; // 90° é o centro (X em frente, Y=0)
  float novoAlvoServoDaGarra  = garra;                // Passagem direta do ângulo da garra
  
  // Restrições de segurança do hardware [0, 180]
  alvoServoDeBaixo = constrain(novoAlvoServoDeBaixo, 0, 180);
  alvoServoDeCima  = constrain(novoAlvoServoDeCima, 0, 180);
  alvoServoDaBase   = constrain(novoAlvoServoDaBase, 0, 180);
  alvoServoDaGarra  = constrain(novoAlvoServoDaGarra, 0, 180);
  
  // Feedback visual do comando recebido
  Serial.println("--- Novos Alvos 3D Definidos ---");
  Serial.print("Base: "); Serial.print(alvoServoDaBase);
  Serial.print("° | Baixo: "); Serial.print(alvoServoDeBaixo);
  Serial.print("° | Cima: "); Serial.print(alvoServoDeCima);
  Serial.print("° | Garra: "); Serial.print(alvoServoDaGarra); Serial.println("°");
}