#include <ESP32Servo.h>
#include <math.h>

// Definição dos pinos dos servos
const int pinoServoDeBaixo = 17;
const int pinoServoDeCima = 18;

// Criação dos objetos Servo
Servo servoDeBaixo;
Servo servoDeCima;

// Comprimentos dos braços em cm
const float L1 = 20.0;
const float L2 = 28.0;

// Limites físicos medidos (em cm)
const float DIST_MIN = 14.0;
const float DIST_MAX = 48.0;

// --- VARIÁVEIS PARA A SUAVIZAÇÃO ---
float alvoServoDeBaixo = 90.0;     
float alvoServoDeCima = 90.0;       
float atualServoDeBaixo = 90.0;    
float atualServoDeCima = 90.0;      

const float FA_SUAVIZACAO = 0.05; 

unsigned long ultimoTempo = 0;
const int intervaloAtualizacao = 20; 

void setup() {
  Serial.begin(9600);
  
  servoDeBaixo.attach(pinoServoDeBaixo, 500, 2500);
  servoDeCima.attach(pinoServoDeCima, 500, 2500);
  
  servoDeBaixo.write((int)atualServoDeBaixo);
  servoDeCima.write((int)atualServoDeCima);
  
  Serial.println("Braço Robótico Inicializado (Modo: Cotovelo para Cima).");
  Serial.println("Envia as coordenadas no formato: x,y (ex: 24,24)");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    
    float x = 0.0;
    float y = 0.0;
    
    if (sscanf(input.c_str(), "%f,%f", &x, &y) == 2) {
      atualizarAlvoIK(x, y);
    } else if (input.length() > 0) {
      Serial.println("ERRO: Formato inválido! Usa: x,y");
    }
  }

  unsigned long tempoAtual = millis();
  if (tempoAtual - ultimoTempo >= intervaloAtualizacao) {
    ultimoTempo = tempoAtual;
    
    atualServoDeBaixo += (alvoServoDeBaixo - atualServoDeBaixo) * FA_SUAVIZACAO;
    atualServoDeCima += (alvoServoDeCima - atualServoDeCima) * FA_SUAVIZACAO;
    
    servoDeBaixo.write((int)atualServoDeBaixo);
    servoDeCima.write((int)atualServoDeCima);
  }
}

void atualizarAlvoIK(float x, float y) {
  float d = sqrt(x*x + y*y);
  
  if (d < DIST_MIN || d > DIST_MAX) {
    Serial.print("ERRO: Alvo fora de alcance! Distancia: ");
    Serial.print(d); Serial.println(" cm");
    return; 
  }
  
  float cosTheta2 = (x*x + y*y - L1*L1 - L2*L2) / (2.0 * L1 * L2);
  if (cosTheta2 < -1.0) cosTheta2 = -1.0;
  if (cosTheta2 > 1.0) cosTheta2 = 1.0;
  
  // MUDANÇA CRUCIAL: Sinal negativo força a geometria de "Cotovelo para Cima"
  float sinTheta2 = -sqrt(1.0 - cosTheta2 * cosTheta2); 
  float theta2 = atan2(sinTheta2, cosTheta2); // Retorna um valor negativo
  
  float k1 = L1 + L2 * cosTheta2;
  float k2 = L2 * sinTheta2;
  float theta1 = atan2(y, x) - atan2(k2, k1);
  
  float theta1Graus = theta1 * 180.0 / M_PI;
  float theta2Graus = theta2 * 180.0 / M_PI;
  
  // Aplicação da calibração baseada nos teus dados
  float novoAlvoServoDeBaixo = 180.0 - theta1Graus;
  float novoAlvoServoDeCima = 90.0 + theta2Graus; // Como theta2Graus é negativo, vai subtrair de 90°
  
  // Restringe aos limites de hardware do servo [0, 180]
  alvoServoDeBaixo = constrain(novoAlvoServoDeBaixo, 0, 180);
  alvoServoDeCima = constrain(novoAlvoServoDeCima, 0, 180);
  
  Serial.print("Novos alvos definidos -> Servo de Baixo: "); Serial.print(alvoServoDeBaixo);
  Serial.print("° | Servo de Cima: "); Serial.print(alvoServoDeCima); Serial.println("°");
}
