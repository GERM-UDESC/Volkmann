#define SD4 PA0   //Sensor da direita mais longe 4
#define SD3 PA1   //Sensor da direita 3
#define SD2 PA2   //Sensor da direita 2
#define SD1 PA3   //Sensor da direita mais ao centro 1
#define SE1 PA4   //Sensor da esquerda mais ao centro 1
#define SE2 PA5   //Sensor da esquerda 2
#define SE3 PA6   //Sensor da esquerda 3
#define SE4 PA7   //Sensor da esquerda mais longe 4
#define SCR PB0   //Sensor para ler as fitas de indicação de curva/Reta
#define SIF PB1   //Sensor para ler as fitas de início e fim da volta

//Portas para leitura dos encoderes

//Encoder ESQUERDA - Timer 1
#define Enc_E_C1 PA8    //Canal 1 do encoder ESQUERDO - Timer1
#define Enc_E_C2 PA9    //Canal 2 do encoder ESQUERDO - Timer1
//Encoder DIREITA - Timer 4
#define Enc_D_C1 PB6    //Canal 1 do encoder DIREITO - Timer4
#define Enc_D_C2 PB7    //Canal 2 do encoder DIREITO - Timer4

//Portas PWM para controle de velocidade

#define Motor_Esquerda PB8      //BINs
#define Motor_Direita PA10      //AINs

//Portas para controle da ponte H

#define AIN2 PB15               //Motor Direito
#define AIN1 PB14
#define STBY PB13
#define BIN1 PB12               //Motor Esquerdo
#define BIN2 PB9

//LEDs
#define LED_Placa PC13

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//constantes para medidas de tempo
long int t_inicial = 10, t_final = 110, t_loop = 100;

//Constantes utilizadas para cálculos de erro
long int erro = 0, erro_anterior = 0, erro_P = 0, erro_I = 0, erro_D = 0, erro_total = 0;

//Valores para as Retas
long int V_Reta = 65500, KP_Reta = 0, KI_Reta = 0, KD_Reta = 0;

//velocidades
int Velocidade_Media = 0, V_Desejada = V_Reta, V_max = 65500, V_min = 0, Velocidade_E, Velocidade_D;

//constantes do controlador, considerando igual à Reta pq geralmente se começa em uma Reta
long int KP = KP_Reta, KI = KI_Reta, KD = KD_Reta;

//Constantes para armazenar os valores lidos dos sensores
int valor_sensores[8];

int calib_max[8], calib_min[8], range[8], calib_sensores = 150;

int leituras_positivas[8];

//Constantes usadas para a lógica da função de paradas
long int t_parada = 9 * 1000000;

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Não lembro se precisa definir as funções antes ou não, vou deixar comentado para garantir
void calibra_sensores();                   //Função para calibrar os sensores
void leitura_sensores();                   //função para fazer a leitura dos sensores
void calculo_erros_PID();                  //calcula os erros
void seta_velocidade();                    //seta a velocidade
void Blink_LED_Delay(int t_delay);         //Função para piscar o LED usando delay
void Parada();                             //Função para definir quando o robô deverá parar
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  //configuração pinos para leitura analógica
  pinMode(SD4, INPUT);
  pinMode(SD3, INPUT);
  pinMode(SD2, INPUT);
  pinMode(SD1, INPUT);
  pinMode(SE1, INPUT);
  pinMode(SE2, INPUT);
  pinMode(SE3, INPUT);
  pinMode(SE4, INPUT);
  pinMode(SCR, INPUT);
  pinMode(SIF, INPUT);


  /*Antes de configurar o PWM, vou configurar já os pinos de direção, pois quando os pinos de PWM são
      configurados como tais, o STM já começa por padrão com um valor de PWM
      Por isso, para garantir que o pulso não prejudique o início do robô, é bom previnir e configurar o que for possível antes.
  */

  //configuração pinos Ponte H
  // HIGH HIGH            ---> "freio"
  // HIGH LOW or LOW HIGH ---> troca os sentidos
  // LOW LOW              ---> Para
  // STBY tem q ser HIGH para ligar a ponte H, se for LOW vai entrar em modo de stand by

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  //Agora, configurando os pinos de saída do PWM do motor e já os desligando em seguida
  pinMode(Motor_Direita, PWM);
  pwmWrite(Motor_Direita, 0);
  pinMode(Motor_Esquerda, PWM);
  pwmWrite(Motor_Esquerda, 0);

  // Ligar o led da placa só pra saber se foi o programa
  pinMode(LED_Placa, OUTPUT);
  calibra_sensores();
  Blink_LED_Delay(1000);
  t_parada += micros();
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  t_inicial = micros();
  leitura_sensores();
  calculo_erros_PID();
  seta_velocidade();
  Parada();
  t_final = micros();
  t_loop = t_final - t_inicial; //lembrando que o valor está em micro segundos
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//função para fazer a leitura dos sensores e calcular o erro
void leitura_sensores()
{
  erro_anterior = erro;
  erro = 0;

  int cont_leituras_positivas = 0;
  long int media = 0;
  int soma = 0;

  for (int i = 0; i < 8; i++)
  {
    leituras_positivas[i] = 0;
    valor_sensores[i] = 0;
  }

  valor_sensores[0] = 1000 * (calib_max[0] - analogRead(SD4)) / range[0];
  valor_sensores[1] = 1000 * (calib_max[1] - analogRead(SD3)) / range[1];
  valor_sensores[2] = 1000 * (calib_max[2] - analogRead(SD2)) / range[2];
  valor_sensores[3] = 1000 * (calib_max[3] - analogRead(SD1)) / range[3];
  valor_sensores[4] = 1000 * (calib_max[4] - analogRead(SE1)) / range[4];
  valor_sensores[5] = 1000 * (calib_max[5] - analogRead(SE2)) / range[5];
  valor_sensores[6] = 1000 * (calib_max[6] - analogRead(SE3)) / range[6];
  valor_sensores[7] = 1000 * (calib_max[7] - analogRead(SE4)) / range[7];

  for (int i = 0; i < 8; i++)
  {
    if (valor_sensores[i] > calib_sensores)
    {
      leituras_positivas[i] = 1;
      cont_leituras_positivas++;
    }
    else
    {
      leituras_positivas[i] = 0;
    }
  }

  for (int i = 0; i < 8; i++)
  {
    if (leituras_positivas[i] == 1)
    {
      if (valor_sensores[i] > 1000) valor_sensores[i] = 1000;
      media += i * (valor_sensores[i]);
      soma += valor_sensores[i];
    }
  }

  if (cont_leituras_positivas == 0)
  {
    erro = erro_anterior;
  }
  else
  {
    erro = ((1000 * media) / soma) - 3500;
  }

  //Verificação dos sensores
  //  Serial.print("E4 = ");
  //  Serial.print(valor_sensores[7]);
  //  Serial.print("\tE3 = ");
  //  Serial.print(valor_sensores[6]);
  //  Serial.print("\tE2 = ");
  //  Serial.print(valor_sensores[5]);
  //  Serial.print("\tE1 = ");
  //  Serial.print(valor_sensores[4]);
  //  Serial.print("\tD1 = ");
  //  Serial.print(valor_sensores[3]);
  //  Serial.print("\tD2 = ");
  //  Serial.print(valor_sensores[2]);
  //  Serial.print("\tD3 = ");
  //  Serial.print(valor_sensores[1]);
  //  Serial.print("\tD4 = ");
  //  Serial.println(valor_sensores[0]);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//calcula os erros e o PID
/* O erro está sendo calculado baseado no arredondamento do seno do angulo entre o centro de
  rotação do robô e a posição do sensor multiplicado por 100 para deixar o valor melhor
  exemplo: sensor 1 (sensor mais da direita) - angulo = 16,80º -> sin(16,80)*100= 28,90
*/
void calculo_erros_PID()
{
  V_Desejada = V_Reta ;//- 1 * abs(erro);
  KP = KP_Reta + 20;
  KI = KI_Reta + 0.6;        //não sei pq, se for >=1 da treta
  KD = KD_Reta + 1000;

  //calculo dos erros
  erro_P = erro;
  erro_I = erro_I + ((erro + erro_anterior) / 2) * t_loop;
  erro_D = ((erro - erro_anterior) * 1000) / (t_loop);
  erro_total = KP * erro_P + KI * erro_I + KD * erro_D;

  //Verificação dos erros
  //  Serial.print("Erro anterior =  ");
  //  Serial.print(erro_anterior);
  //  Serial.print("\tErro = ");
  //  Serial.print(erro);
  //  Serial.print("\terro_total = ");
  //  Serial.print(erro_total);
  //  Serial.print("\tV_Desejada = ");
  //  Serial.println(V_Desejada);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//seta a velocidade
void seta_velocidade()
{
  if (Velocidade_Media < V_Desejada)
  {
    Velocidade_Media += 120;
  }
  else
  {
    Velocidade_Media = V_Desejada;
  }

  Velocidade_E = Velocidade_Media + erro_total;
  Velocidade_D = Velocidade_Media - erro_total;

  if (Velocidade_E <= 0)                          //Inverte a rotação do motor para reduzir a velocidade mais rápido
  {
    Velocidade_E = -Velocidade_E;
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }

  if (Velocidade_D <= 0)                        //Inverte a rotação do motor para reduzir a velocidade mais rápido
  {
    Velocidade_D = -Velocidade_D;
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }

  if (Velocidade_E >= V_max)
  {
    Velocidade_E = V_max;
    //    erro_I = erro_I - ((erro + erro_anterior) / 2) * t_loop;  //anti windup
  }
  if (Velocidade_D >= V_max)
  {
    Velocidade_D = V_max;
    //    erro_I = erro_I - ((erro + erro_anterior) / 2) * t_loop;  //anti windup
  }

  pwmWrite(Motor_Direita, Velocidade_D);
  pwmWrite(Motor_Esquerda, Velocidade_E);

  //  Verificação das velocidades
  //    Serial.print("V_media = ");
  //    Serial.print(Velocidade_Media);
  //    Serial.print("\tV_Desejada = ");
  //    Serial.print(V_Desejada);
  //    Serial.print("\tV. Esquerda = ");
  //    Serial.print(Velocidade_E);
  //    Serial.print("\tV. Direita = ");
  //    Serial.println(Velocidade_D);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Função para definir quando o robô deverá parar, seja por tempo ou pelas fitas de indicação
//Como haverão cruzamentos, o sensor será acionado diversas vezes durante o percurso, a variável n_linhas é quantas linhas o robô contará até chegar na última
void Parada()
{
  //Parada por tempo
  if (micros() > t_parada)            //Desliga os motores se já tiver passado o tempo - LEMBRAR Q ESSE TEMPO TAMBÉM CONTA O TEMPO DE PISCAR LED e calibrar sensores
  {
    pwmWrite(Motor_Direita, 0);
    pwmWrite(Motor_Esquerda, 0);
    while (1 > 0);                      //Fica nessa parte infinitamente
  }

}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void calibra_sensores()
{
  int i, j;

  for (i = 0; i < 8; i++)       //determina um valor inicial médio para os vetores, para que depois possam ser comparados e alterados
  {
    calib_max[i] = 2000;
    calib_min[i] = 2000;
  }

  for (i = 0; i < 40000; i++)
  {
    valor_sensores[0] = analogRead(SD4);
    valor_sensores[1] = analogRead(SD3);
    valor_sensores[2] = analogRead(SD2);
    valor_sensores[3] = analogRead(SD1);
    valor_sensores[4] = analogRead(SE1);
    valor_sensores[5] = analogRead(SE2);
    valor_sensores[6] = analogRead(SE3);
    valor_sensores[7] = analogRead(SE4);
    for (j = 0; j < 8; j++)
    {
      if (valor_sensores[j] < calib_min[j])
      {
        calib_min[j] = valor_sensores[j];
      }
      if (valor_sensores[j] > calib_max[j])
      {
        calib_max[j] = valor_sensores[j];
      }
    }
  }

  for (i = 0; i < 8; i++)
  {
    range[i] = calib_max[i] - calib_min[i];
  }


  //  for (i=0;i<8;i++)
  //  {
  //    Serial.print("\tmax");
  //    Serial.print(calib_max[i]);
  //  }
  //  Serial.println(" ");
  //for (i=0;i<8;i++)
  //  {
  //    Serial.print("\tmin");
  //    Serial.print(calib_min[i]);
  //  }


}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Função para piscar o LED usando delay
void Blink_LED_Delay(int t_delay)
{
  digitalWrite(LED_Placa, LOW);            //Ligado -> LOW --- Desligado -> HIGH
  delay(t_delay);
  digitalWrite(LED_Placa, HIGH);
  delay(t_delay);
  digitalWrite(LED_Placa, LOW);
  delay(t_delay);
  digitalWrite(LED_Placa, HIGH);
  delay(t_delay);
  digitalWrite(LED_Placa, LOW);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

