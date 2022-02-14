//Experimentos Hardware in the loop - MCP do MICAsh
// Autor: Igor André Houri e Costa
// Orientador: Anísio R. Braga
// Fevereiro de 2022
//

#include <driver/dac.h>
#include <HardwareSerial.h>

float valor_recebidoA = 0; // saida A do processo
float valor_recebidoB = 0; // saida B do processo

int AN_Pot1 = 35;
double yk = 0;
double yf = 0;
double ym = 0;

bool VarIndex = false; // Indice que identifica a qual saida do processo a porcao da mensagem serial recebida corresponde.

bool stringRxCompleteA = false;
bool stringRxStartA = false;
int incomingValueA = 0;

bool stringRxCompleteB = true;
bool stringRxStartB = false;
int incomingValueB = 0;

uint8_t outcomingMilhar = 0;
uint8_t outcomingCentena = 0;
uint8_t outcomingDezena = 0;
uint8_t outcomingUnidade = 0;

float in_minOUT = 0; // valor minimo na escala (range) do numero que vem do matlab. Deve ser igual ao out_min do bloco map da saída do processo no simulink.
float in_maxOUT = 4095; // valor maximo na escala (range) do numero que vem do matlab . Deve ser igual ao out_max ddo bloco map da saída do processo no simulink.

float out_minOUTA = 0; // valor minimo da escala da saída A do processo simulado. Deve ser igual ao maximo_saida do simulink.
float out_maxOUTA = 70;// valor maximo da escala da saída A do processo simulado. Deve ser igual ao minimo_saida do simulink.

float out_minOUTB = 0; // valor minimo da escala da saída B do processo simulado. Deve ser igual ao maximo_saida do simulink.
float out_maxOUTB = 15;// valor maximo da escala da saída B do processo simulado. Deve ser igual ao minimo_saida do simulink.

float In_processo = 0.0; // Entrada do processo, recebida do clp para ser enviada ao processo simulado.
float in_maxIN = 3; // Valor máximo na escala (range) da entrada do processo.
float in_minIN = 0; // Valor minimo na escala (range) da saida do processo

unsigned long h= 100;
unsigned long t_atual;
unsigned long t_alvo;

unsigned long mCtr = 0; // Contador para decimação por m
unsigned long m = 2; // Fator de decimação que define constante de tempo do filtro digital
double alfa_f = 2.0/(2.0*m + 1.0); // Função de Transferência do compensador em s
                                   // G = (b*s + 1)/(a*s + 1) e as respectivas constantes com a aproximação de Tustin


HardwareSerial MySerial(1);

//DAC_CHANNEL_1 is attached to GPIO25.


void serialEvent() {
  if (MySerial.available()> 0) { // Verifica se tem algo a ser lido na porta serial
    // Obtem um novo byte:
    uint8_t inChar = MySerial.read();

    if(inChar == 65){ // Verifica se a saida a qual a parte da msg atual corresponde é a saída A e seta a flag
      VarIndex = false;
    }
    if(inChar == 66){// Verifica se a saida a qual a parte da msg atual corresponde é a saída B e seta a flag
      VarIndex = true;
    }
    if (VarIndex == false){ // decodifica saida A
      if (stringRxStartA == true){ // Verifica se a string já começou
        if (inChar == 13) { // se o caracter for um newline (\n), identifica que a string terminou.
          stringRxCompleteA = true;
          stringRxStartA = false;
        }
        else {
          incomingValueA = incomingValueA*10 +  int2ascii(inChar); // se o caracter NAO for um newline (\n), le o caracter que chega e compoe a mensagem.
        }
      }
      if (inChar == 32){ // verifica se a nova string começou. Detecta o espaco e seta a flag.
        stringRxStartA = true;
        incomingValueA = 0;
      }
    }

  if (VarIndex == true){// decodifica saida A
      if (stringRxStartB == true){ // Verifica se a string ja começou
        if (inChar == 13) { // se o caracter for um newline (\n), identifica que a string terminou.
          stringRxCompleteB = true;
          stringRxStartB = false;
        }
        else {
          incomingValueB = incomingValueB*10 +  int2ascii(inChar); // se o caracter NAO for um newline (\n), le o caracter que chega e compoe a mensagem.
        }
      }
      if (inChar == 32){ // verifica se a nova string começou. Detecta o espaco e seta a flag.
        stringRxStartB = true;
        incomingValueB = 0;
      }
    }
    
  }
}

int int2ascii (uint8_t inChar){ // função utilizada para decifrar os caracteres recebidos via porta Serial vindos do simulink.
  switch (inChar){              // converte o código ASCII ao caracter.
    case 48:
      return 0;
    case 49:
      return 1;
    case 50:
      return 2;
    case 51:
      return 3;
    case 52:
      return 4;
    case 53:
      return 5;
    case 54:
      return 6;
    case 55:
      return 7;
    case 56:
      return 8;
    case 57:
      return 9;
  }
}

float Map (int x, int in_min, int in_max, int out_min, int out_max){ // mapeia um numero de um range(escala) a outro.
  return ((float)x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

void leituraEntrada(){ // Le a saida do plc, que e a entrada do processo simulado.
  yk = analogRead(AN_Pot1); // Lr a entrada ADC
  yf = yf + alfa_f*(yk-yf); // filtro digital 1a. ordem sem atraso de amostragem

  if (mCtr % m ==0){ // amostra a variavel filtrada decimada por m.
    ym = yf;
  }

  // Codificacão para o formato necessario para envio ao simulink
   In_processo = ((float)ym * in_maxIN)/4095.0;
   In_processo = Map(In_processo, in_minIN, in_maxIN,in_minOUT,in_maxOUT);
  
   int i = In_processo/1000;
   outcomingMilhar = (uint8_t)i;
   In_processo = In_processo - (float)i*1000;
   i = In_processo/100;
   outcomingCentena = (uint8_t)i;
   In_processo = In_processo - (float)i*100;
   i = In_processo/10;
   outcomingDezena = (uint8_t)i;
   In_processo = In_processo - (float)i*10;
   outcomingUnidade = (uint8_t)In_processo;  

  // Envio das mensagens ao simulink
   MySerial.write(outcomingMilhar);
   MySerial.write(outcomingCentena);
   MySerial.write(outcomingDezena);
   MySerial.write(outcomingUnidade);
}

void setup() {
  Serial.begin(115200); // configura comunicação serial com o monitor
  MySerial.begin(115200, SERIAL_8N1, 16, 17); // configura o canal serial utilizado para a comunicação com Raspberry pi
  dac_output_enable(DAC_CHANNEL_1); //

  h=100; // intervalo de amostragem h=100ms
  //--- inicia a cronometragem do loop
  t_atual = millis();
  t_alvo = t_atual+h; // cálculo do próximo instante t_alvo para amostragem
}

void loop() {
    t_atual = millis(); //lê relógio
    if (t_atual >= t_alvo){ // verifica se o t_alvo foi ultrapassado
      //*** Tempo ativo de comando (TAC)
      t_alvo = t_alvo + h;
      mCtr = mCtr + 1; // incrementa o contador de decimação
      //* Le entrada do processo disponível no conversor analogico digital e a envia para o simulink
     leituraEntrada();
     //* Le saída do processo disponível no conversor
    }
      //*** Tempo ocioso (TOC)
      serialEvent(); // lê saida do processo no simulink
      if (stringRxCompleteA == true && stringRxCompleteB == true){ // Se a mensagem lida esta completa, mapeia para a escala dos valores do processo e plota no monitor serial
      stringRxCompleteA = false;
      stringRxCompleteB = false;
      valor_recebidoA = Map(incomingValueA, in_minOUT, in_maxOUT, out_minOUTA, out_maxOUTA);
      valor_recebidoB = Map(incomingValueB, in_minOUT, in_maxOUT, out_minOUTB, out_maxOUTB);
      Serial.print(valor_recebidoA);
      Serial.print(' ');
      Serial.print(valor_recebidoB);
      Serial.print('\n');
  }
  
}
