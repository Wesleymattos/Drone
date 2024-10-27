#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// Configuração do MPU6050
MPU6050 mpu;
double anguloX, anguloY;

// Configuração PID
double setpointX = 0, setpointY = 0; // O ângulo desejado (0 é estável)
double inputX, inputY;               // Valores atuais de ângulo
double outputX, outputY;             // Saídas PID para controle de motores
double Kp = 1.5, Ki = 0.2, Kd = 0.1; // Constantes do PID (ajuste conforme necessário)
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);

// Pinos dos motores
const int motorFrente = 5;
const int motorTras = 18;

void setup() {
  Serial.begin(115200);
  
  // Inicializar MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Erro ao conectar MPU6050");
    while (1);
  }
  Serial.println("MPU6050 conectado");

  // Configurar PID
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);

  // Configurar pinos dos motores como saída
  pinMode(motorFrente, OUTPUT);
  pinMode(motorTras, OUTPUT);
}

void loop() {
  // Leitura dos ângulos do MPU6050
  mpu.getRotation(&anguloX, &anguloY, NULL);

  // Atualizar entrada dos PIDs
  inputX = anguloX / 131.0; // Conversão dos dados do sensor para graus
  inputY = anguloY / 131.0;

  // Calcular saídas PID
  pidX.Compute();
  pidY.Compute();

  // Ajuste dos motores de acordo com a saída do PID
  int velocidadeMotorFrente = constrain(1500 + outputX, 1000, 2000); // ajuste o valor conforme necessário
  int velocidadeMotorTras = constrain(1500 - outputX, 1000, 2000);

  // Enviar PWM para os motores
  analogWrite(motorFrente, velocidadeMotorFrente);
  analogWrite(motorTras, velocidadeMotorTras);

  // Exibir dados para depuração
  Serial.print("Angulo X: "); Serial.print(inputX);
  Serial.print(" | Output PID X: "); Serial.println(outputX);
  delay(50); // Ajuste conforme necessário
}