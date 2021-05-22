/*
 * Sprint_9.c
 *
 * Created: 11/05/2021 11:00:12
 * Author : João Pedro do Santos Silva
 * Matricula :118110469
 */ 

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#include "nokia5110.h"


void freq_resp_pwm(uint8_t cont);
void plot_sinaisVitais(uint8_t bpm,uint8_t temperatura, uint8_t spo2,char press[]);
void plot_parametros(uint8_t FreqRespiracao,uint8_t nivelo2, uint8_t volume);
void plot_agendamento(uint8_t FreqRespiracao,uint8_t nivelo2, uint8_t volume,uint8_t temp_ag);
void plot_grafico(uint8_t cont, uint8_t x);
void plot_tx();
void alerta(uint8_t temperatura, uint8_t spo2);
void valvula_o2(uint8_t nivel);
void USART_init(unsigned int ubrr);
void usart_tx(uint8_t data);
int sensor_temperatura(uint16_t lt_ADC);
int sensor_spo2(uint16_t lt_ADC);

uint8_t FreqRespiracao = 5,sel = 0, NivelO2 = 50, VolO2 = 4;
uint8_t FreqRes_ag = 5 , nivelo2_ag = 50, volume_ag = 4, temp_ag =1;
uint8_t bpm = 0,cursor = 0,ag_set = 0,x_pass=0,set_tx=0,dado;
uint16_t tbat=0,tbat_ant = 0,leitura_ADC = 0;
uint32_t tempo_ms=0, tempo_ant = 0 ,tempAnt_ag = 0,temAntTx = 0;
uint8_t temperatura = 0, spo2 = 0;
char press[8] = "HHHxLLL";
char presstemp[8] = "       ";
char clear[8] = "       ";
uint8_t cont_recp = 0,ind_x=0;
uint8_t contpwm = 1,indpw = 1;
uint8_t pack_init= 0,erro_init=0,formatx=0,cpy_c_recpt=0;

// receber informação pela serial
ISR(USART_RX_vect)
{
	//;HHHxLLL:
	
	char recebido;
	recebido = UDR0;
	
	if(pack_init == 1)
	{
		
		if( (recebido >= '0' && recebido <= '9') | (recebido == 'x'))
		{
			presstemp[cont_recp] = recebido;
			cont_recp++;
			cpy_c_recpt = cont_recp;
			
			if(cont_recp==8)
			{
				strcpy(press ,"ERRO!  ");
				strcpy(presstemp ,clear);
				pack_init = 0;
			}
			
			
			cont_recp = (cont_recp == 8) ? 0:cont_recp;
		}
		else if(recebido == ':' && cpy_c_recpt>=4)
		{
			cpy_c_recpt =0;
			pack_init = 0;
			for(uint8_t i = 0; i<7; i++)
			{
				formatx += (presstemp[i] == 'x') ? 1:0;
				ind_x = (presstemp[i] == 'x')? i:ind_x;
			}
			
			if( ind_x > 3)
			{
				strcpy(press ,"ERRO!   ");
				strcpy(presstemp ,clear);
				pack_init = 0;
				formatx =0;
				cont_recp = 0;
			}
			
			
			if(formatx!= 1 )
			{
				
				strcpy(press ,"ERRO!   ");
				strcpy(presstemp ,clear);
				pack_init = 0;
				formatx =0;
				cont_recp = 0;
				
			}
			else
			{
				strcpy(press , presstemp);
				strcpy(presstemp ,clear);
				pack_init = 0;
				formatx =0;
				cont_recp =0;
			}
			
		}
		else
		{
			strcpy(press ,"ERRO!  ");
			strcpy(presstemp ,clear);
			pack_init = 0;
			formatx =0;
			cont_recp = 0;
		}
	}

	pack_init = (recebido == ';') ? 1:pack_init;
	erro_init = (pack_init == 0 && recebido !=';' && recebido !=':')? 1:0;
	
	if(erro_init)
	{
		strcpy(press ,"ERRO!  ");
		strcpy(presstemp ,clear);
		erro_init=0;
	}
	
	
}

ISR(TIMER0_COMPA_vect)
{

	tempo_ms++;

}

// ativa na borda de descida do pino D2(INT0)
ISR(INT0_vect)
{
	// tela parametros
	if((cursor == 1) & (sel == 1)){
		FreqRespiracao ++;
		FreqRespiracao = (FreqRespiracao == 31) ? 30:FreqRespiracao;
	}
	else if((cursor == 2) & (sel == 1))
	{
		NivelO2 += 10;
		NivelO2 = (NivelO2 == 110) ? 100:NivelO2;
		valvula_o2(NivelO2);
	}
	else if ((cursor == 3) & (sel == 1))
	{
		VolO2 ++;
		VolO2 = (VolO2 == 9)? 8:VolO2;
		contpwm = 0;
		indpw = 0;
	}
	// tela agendamento
	if((cursor == 1) & (sel == 2)){
		FreqRes_ag ++;
		FreqRes_ag = (FreqRes_ag == 31) ? 30:FreqRes_ag;
	}
	else if((cursor == 2) & (sel == 2))
	{
		nivelo2_ag += 10;
		nivelo2_ag = (nivelo2_ag == 110) ? 100:nivelo2_ag;
	
	}
	else if ((cursor == 3) & (sel == 2))
	{
		volume_ag ++;
		volume_ag = (volume_ag == 9)? 8:volume_ag;
	}
	else if ((cursor == 4) & (sel == 2))
	{
		temp_ag ++;
		temp_ag = (temp_ag == 61)? 60:temp_ag;
	}

}

// ativa na borda de descida do pino D3(INT1)
ISR(INT1_vect)
{
	//tela parametros
	if((cursor == 1) & (sel == 1))
	{
		FreqRespiracao --;
		FreqRespiracao = (FreqRespiracao == 4) ? 5:FreqRespiracao;
	}
	else if((cursor == 2) & (sel ==1))
	{
		NivelO2 -= (NivelO2 == 0) ? 0:10;
		valvula_o2(NivelO2);
		
	}
	else if ((cursor == 3) & (sel ==1))
	{
		VolO2 --;
		VolO2 = (VolO2 == 0)? 1:VolO2;
		contpwm = 0;
		indpw = 0;
	}
	// tela agendamento
	
		if((cursor == 1) & (sel == 2))
		{
			FreqRes_ag --;
			FreqRes_ag = (FreqRes_ag == 4) ? 5:FreqRes_ag;
		}
		else if((cursor == 2) & (sel ==2))
		{
			nivelo2_ag -= (nivelo2_ag == 0) ? 0:10;
			
		}
		else if ((cursor == 3) & (sel ==2))
		{
			volume_ag --;
			volume_ag = (volume_ag == 0)? 1:volume_ag;
		}
	    else if ((cursor == 4) & (sel == 2))
	    {
			temp_ag --;
			temp_ag = (temp_ag == 0)? 1:temp_ag;  
	    }	
	
	
}

ISR(PCINT2_vect)
{

	if(PIND &(1<<4))
	{
		
		tbat = tempo_ms - tbat_ant ;
		tbat_ant = tempo_ms;
		
	}
	
	bpm =  (1./tbat)*60000;
	
	//seletor de função
	if(!(PIND &(1<<5)))
	{
		sel+= 1;
		sel = (sel == 5) ? 0:sel;
		cursor = (sel==0) ? 0:1;
		if(sel==3){
		x_pass = 0;	
		nokia_lcd_clear();
		}

	}
	// navegação
	if(!(PIND &(1<<6)))
	{
	   cursor -= (cursor != 1) ? 1:0;
	}
	
	if(!(PIND &(1<<7)))
	{
		if(sel == 1){
			cursor+= 1;
			cursor = (cursor == 4) ? 3:cursor;
		    }
		if(sel == 2){
			cursor+= 1;
			cursor = (cursor == 6) ? 5:cursor;
		}	
		
		if(sel == 4){
			cursor+= 1;
			cursor = (cursor == 4) ? 5:cursor;
		}
		
	}

}

ISR(ADC_vect)
{
	
	leitura_ADC = ADC;
}



int main(void)
{
	// definições de entrada e saída
	
	DDRB |= 0b00001110;
	DDRD = 0b00000011;
	DDRC = 0b1111100;
	PORTD |= 0b11101100;
	PORTC &= 0b1111100;
	
	//configuração PWM top = (F_CPU)/(pre*fpwm)-1 = 39999; pre = 8 , fpwm = 50Hz;
	
	ICR1 = 39999; // top
	TCCR1A = 0b10100010; // habilita o pwm de comparação não invertida para ocra e ocrb e com a contagem especifica top
	TCCR1B = 0b00011010; // complemento para seleção de modo e configuração do pre =8
	
	OCR1A = 2000; // posição inicial do servo em B1(BVM); 0°
	OCR1B = 3000; // posição inicial do servo em B2	(válvula de O2); 90°
	
	// configuração de timers
	
	TCCR0A = 0b00000010; //habilita o modo ctc do TC0
	TCCR0B = 0b00000011; //habilita o TC0 com prescaler 64
	OCR0A = 249; //ajustando comparador
	
	// configuração do ADC
	
	ADMUX = 0b01000000; //Vref = Vcc entrada C0
	ADCSRA = 0b11101111; //habilita interrupção, modo de conversão continua,prescaler 128
	ADCSRB = 0x00; //converção continua
	DIDR0 = 0b00111110; //habilita C0 como entrada do ADC
	
	// Configurações de interrupção
	
	TIMSK0 = 0b00000010; // habilita a interrupção na igualdade de comparação com OCR0A
	
	EICRA = 0b00001010;// ativa na borda de descida do pino D2(INT0) e D3(INT1)
	EIMSK = 0b00000011; // habilita as interrupções devido os pinos D2(INT0) e D3(INT1)
	
	PCICR = 0b00000100; //seleciona as interrupções para os variações nos pinos D
	PCMSK2 |= 0b00010000; // habilita a interrupção para o pino D0
	
	sei();
	
	
	//lcd
	nokia_lcd_init();
	
	//USART
	USART_init(MYUBRR);
	
	// variaveis auxiliares
	
	
	uint32_t temp_ant_lcd = 0;
	uint8_t mux = 0;
	uint32_t temp_ant_mux = 0;
	
	while (1)
	{
		
		
		//agendamento
		if((tempo_ms - tempAnt_ag >= (temp_ag*60000)) & (ag_set == 1))
		{
			FreqRespiracao = FreqRes_ag;
			
			NivelO2 = nivelo2_ag;
			valvula_o2(NivelO2);
			
			VolO2 = volume_ag;
			indpw = 0;
			contpwm = 0;
			
			ag_set = 0;
			FreqRes_ag = 5;
			nivelo2_ag =50;
			volume_ag = 4;
			temp_ag = 1;
		}
		
		//multiplexação ADC
		if((tempo_ms-temp_ant_mux)>=150)
		{
			switch(mux)
			{
				case 0:
				
				temperatura = sensor_temperatura (leitura_ADC);
				
				ADMUX ^= 0b00000001;
				DIDR0 ^= 0b00000011;
				mux = 1;
				
				break;
				
				case 1:
				
				spo2 = sensor_spo2 (leitura_ADC);
				
				ADMUX ^= 0b00000001;
				DIDR0 ^= 0b00000011;
				mux = 0;
				
				break;
			}
			
			temp_ant_mux = tempo_ms;
			alerta(temperatura, spo2);
		}
		
		// telas do LCD
		if((tempo_ms-temp_ant_lcd)>=200)
		{
			if(sel==0)
			plot_sinaisVitais(bpm,temperatura,spo2,press);
			else if(sel==1)
			plot_parametros(FreqRespiracao,NivelO2,VolO2);
			else if(sel==2)
			plot_agendamento(FreqRes_ag,nivelo2_ag,volume_ag,temp_ag);
			else if(sel==4)
			plot_tx();
			
			temp_ant_lcd = tempo_ms;
		}
		
		float delay_def = (60./(FreqRespiracao*2*(VolO2)))*1000;//calculo delay para troca de posições do servo do BVM
		// Servo do BVM
		if ((tempo_ms - tempo_ant) >= delay_def)
		{
			freq_resp_pwm(contpwm);
			indpw ++;
			indpw = (indpw == (2*VolO2)) ? 0:indpw;
			contpwm += ((indpw > VolO2) & (contpwm!=0)) ? -1:1;
			contpwm = (indpw == 0)? 0:contpwm;
			
			// grafico
			
			if(sel==3){
				plot_grafico(contpwm,x_pass);
				
				x_pass ++;
				x_pass=(x_pass==60)? 0:x_pass;
				
			}
			
			tempo_ant = tempo_ms;
		}
		
		// transmissão a cada 1 segundo 
		if((tempo_ms- temAntTx >= 1000) & (set_tx==1))
		{
			usart_tx(dado);		
			temAntTx =tempo_ms;	
		}
		
	}
}


void freq_resp_pwm(uint8_t cont)
{
	uint16_t ocr_val [9] = {2000,2250,2500,2750,3000,3238,3500,3750,4000};
	
	OCR1A = ocr_val[cont];
	//alerta
	if(cont == 0)
	{
		  
		PORTB |= 0b00001000;
	}
	
	else
	{
		PORTB &= 0b11110111 ;
	}
	
}

void plot_sinaisVitais(uint8_t bpm, uint8_t temperatura, uint8_t spo2 ,char press[])
{
	char bpm_s[5],temperatura_s[5],spo2_s[5];
	
	sprintf(bpm_s, "%d", bpm);
	sprintf(temperatura_s, "%d", temperatura);
	sprintf(spo2_s, "%d", spo2);

	
	nokia_lcd_clear();
	
		
	nokia_lcd_set_cursor(0,0);
	nokia_lcd_write_string("Sinais Vitais",1);
		
	nokia_lcd_set_cursor(10,10);
	nokia_lcd_write_string(bpm_s,1);
	nokia_lcd_set_cursor(45,10);
	nokia_lcd_write_string("bpm",1);
		
	nokia_lcd_set_cursor(10,20);
	nokia_lcd_write_string(spo2_s,1);
	nokia_lcd_set_cursor(45,20);
	nokia_lcd_write_string("%SpO2",1);
		
	nokia_lcd_set_cursor(10,30);
	nokia_lcd_write_string(temperatura_s,1);
	nokia_lcd_set_cursor(45,30);
	nokia_lcd_write_string("°C",1);
		
	nokia_lcd_set_cursor(10,40);
	nokia_lcd_write_string(press,1);
	nokia_lcd_set_cursor(52,40);
	nokia_lcd_write_string("mmHg",1);
	
	nokia_lcd_render();
	
	
}
void plot_parametros(uint8_t FreqRespiracao,uint8_t nivelo2, uint8_t volume)
{
	char freq_s[5],nivelo2_s[5],volume_s[5];
	
	sprintf(nivelo2_s, "%d", nivelo2);
	sprintf(volume_s, "%d", volume);
	sprintf(freq_s, "%d", FreqRespiracao);
	
	nokia_lcd_clear();
	
	if (cursor == 1){
		
	nokia_lcd_set_cursor(0,0);
	nokia_lcd_write_string("Parametros",1);
	
	nokia_lcd_set_cursor(0,15);
	nokia_lcd_write_string("resp/min * ",1);
	//nokia_lcd_set_cursor(30,15);
	
	nokia_lcd_set_cursor(70,15);
	nokia_lcd_write_string(freq_s,1);

	nokia_lcd_set_cursor(0,25);
	nokia_lcd_write_string("Nivel02",1);
	nokia_lcd_set_cursor(55,25);
	nokia_lcd_write_string(nivelo2_s,1);
	nokia_lcd_set_cursor(75,25);
	nokia_lcd_write_string("%",1);

	nokia_lcd_set_cursor(0,35);
	nokia_lcd_write_string("Volume",1);
	nokia_lcd_set_cursor(70,35);
	nokia_lcd_write_string(volume_s,1);
	nokia_lcd_set_cursor(75,35);
	nokia_lcd_write_string("V",1);
	}
	else if (cursor == 2){
		
		nokia_lcd_set_cursor(0,0);
		nokia_lcd_write_string("Parametros",1);
		
		nokia_lcd_set_cursor(0,15);
		nokia_lcd_write_string("resp/min ",1);
	
		
		nokia_lcd_set_cursor(70,15);
		nokia_lcd_write_string(freq_s,1);

		nokia_lcd_set_cursor(0,25);
		nokia_lcd_write_string("Nivel02 *",1);
		nokia_lcd_set_cursor(55,25);
		nokia_lcd_write_string(nivelo2_s,1);
		nokia_lcd_set_cursor(75,25);
		nokia_lcd_write_string("%",1);

		nokia_lcd_set_cursor(0,35);
		nokia_lcd_write_string("Volume",1);
		nokia_lcd_set_cursor(70,35);
		nokia_lcd_write_string(volume_s,1);
		nokia_lcd_set_cursor(75,35);
		nokia_lcd_write_string("V",1);
	}
	else if (cursor == 3){
		
		nokia_lcd_set_cursor(0,0);
		nokia_lcd_write_string("Parametros",1);
		
		nokia_lcd_set_cursor(0,15);
		nokia_lcd_write_string("resp/min ",1);
		//nokia_lcd_set_cursor(30,15);
		
		nokia_lcd_set_cursor(70,15);
		nokia_lcd_write_string(freq_s,1);

		nokia_lcd_set_cursor(0,25);
		nokia_lcd_write_string("Nivel02 ",1);
		nokia_lcd_set_cursor(55,25);
		nokia_lcd_write_string(nivelo2_s,1);
		nokia_lcd_set_cursor(75,25);
		nokia_lcd_write_string("%",1);

		nokia_lcd_set_cursor(0,35);
		nokia_lcd_write_string("Volume *",1);
		nokia_lcd_set_cursor(70,35);
		nokia_lcd_write_string(volume_s,1);
		nokia_lcd_set_cursor(75,35);
		nokia_lcd_write_string("V",1);
	}
	
	
	nokia_lcd_render();
	
}

void plot_agendamento(uint8_t FreqRespiracao,uint8_t nivelo2, uint8_t volume, uint8_t temp_ag)
{
	
	char freq_s[5],nivelo2_s[5],volume_s[5],temp_s[5];
	
	sprintf(nivelo2_s, "%d", nivelo2);
	sprintf(volume_s, "%d", volume);
	sprintf(freq_s, "%d", FreqRespiracao);
	sprintf(temp_s, "%d", temp_ag);
	
	nokia_lcd_clear();
		
	if (cursor == 1){
			
		nokia_lcd_set_cursor(0,0);
		nokia_lcd_write_string("Agendamento",1);
			
		nokia_lcd_set_cursor(0,10);
		nokia_lcd_write_string("resp/min * ",1);
			
		nokia_lcd_set_cursor(70,10);
		nokia_lcd_write_string(freq_s,1);

		nokia_lcd_set_cursor(0,20);
		nokia_lcd_write_string("Nivel02",1);
		nokia_lcd_set_cursor(55,20);
		nokia_lcd_write_string(nivelo2_s,1);
		nokia_lcd_set_cursor(75,20);
		nokia_lcd_write_string("%",1);

		nokia_lcd_set_cursor(0,30);
		nokia_lcd_write_string("Volume",1);
		nokia_lcd_set_cursor(65,30);
		nokia_lcd_write_string(volume_s,1);

		nokia_lcd_set_cursor(75,30);
		nokia_lcd_write_string("V",1);
		
		nokia_lcd_set_cursor(0,40);
		nokia_lcd_write_string("Tempo",1);
		nokia_lcd_set_cursor(45,40);
		nokia_lcd_write_string(temp_s,1);

		nokia_lcd_set_cursor(65,40);
		nokia_lcd_write_string("min",1);
		
		}
	else if (cursor == 2){
			
		nokia_lcd_set_cursor(0,0);
		nokia_lcd_write_string("Agendamento",1);
					
		nokia_lcd_set_cursor(0,10);
		nokia_lcd_write_string("resp/min ",1);
		nokia_lcd_set_cursor(70,10);
		nokia_lcd_write_string(freq_s,1);

		nokia_lcd_set_cursor(0,20);
		nokia_lcd_write_string("Nivel02 *",1);
		nokia_lcd_set_cursor(55,20);
		nokia_lcd_write_string(nivelo2_s,1);
		nokia_lcd_set_cursor(75,20);
		nokia_lcd_write_string("%",1);

		nokia_lcd_set_cursor(0,30);
		nokia_lcd_write_string("Volume",1);
		nokia_lcd_set_cursor(65,30);
		nokia_lcd_write_string(volume_s,1);

		nokia_lcd_set_cursor(75,30);
		nokia_lcd_write_string("V",1);
		
		nokia_lcd_set_cursor(0,40);
		nokia_lcd_write_string("Tempo",1);
		nokia_lcd_set_cursor(45,40);
		nokia_lcd_write_string(temp_s,1);

		nokia_lcd_set_cursor(65,40);
		nokia_lcd_write_string("min",1);
		
		}
	else if (cursor == 3){
			
		nokia_lcd_set_cursor(0,0);
		nokia_lcd_write_string("Agendamento",1);
			
		nokia_lcd_set_cursor(0,10);
		nokia_lcd_write_string("resp/min ",1);
		nokia_lcd_set_cursor(70,10);
		nokia_lcd_write_string(freq_s,1);

		nokia_lcd_set_cursor(0,20);
		nokia_lcd_write_string("Nivel02 ",1);
		nokia_lcd_set_cursor(55,20);
		nokia_lcd_write_string(nivelo2_s,1);
		nokia_lcd_set_cursor(75,20);
		nokia_lcd_write_string("%",1);

		nokia_lcd_set_cursor(0,30);
		nokia_lcd_write_string("Volume *",1);
		nokia_lcd_set_cursor(65,30);
		nokia_lcd_write_string(volume_s,1);

		nokia_lcd_set_cursor(75,30);
		nokia_lcd_write_string("V",1);
		
		nokia_lcd_set_cursor(0,40);
		nokia_lcd_write_string("Tempo",1);
		nokia_lcd_set_cursor(45,40);
		nokia_lcd_write_string(temp_s,1);

		nokia_lcd_set_cursor(65,40);
		nokia_lcd_write_string("min",1);
		}
		
		else if (cursor == 4){
			
			nokia_lcd_set_cursor(0,0);
			nokia_lcd_write_string("Agendamento",1);
			
			nokia_lcd_set_cursor(0,10);
			nokia_lcd_write_string("resp/min ",1);
			nokia_lcd_set_cursor(70,10);
			nokia_lcd_write_string(freq_s,1);

			nokia_lcd_set_cursor(0,20);
			nokia_lcd_write_string("Nivel02 ",1);
			nokia_lcd_set_cursor(55,20);
			nokia_lcd_write_string(nivelo2_s,1);
			nokia_lcd_set_cursor(75,20);
			nokia_lcd_write_string("%",1);

			nokia_lcd_set_cursor(0,30);
			nokia_lcd_write_string("Volume ",1);
			nokia_lcd_set_cursor(65,30);
			nokia_lcd_write_string(volume_s,1);

			nokia_lcd_set_cursor(75,30);
			nokia_lcd_write_string("V",1);
			
			nokia_lcd_set_cursor(0,40);
			nokia_lcd_write_string("Tempo *",1);
			nokia_lcd_set_cursor(45,40);
			nokia_lcd_write_string(temp_s,1);

			nokia_lcd_set_cursor(65,40);
			nokia_lcd_write_string("min",1);
		}
		else if (cursor == 5)
		{
			nokia_lcd_set_cursor(0,20);
			nokia_lcd_write_string("Agendamento",1);
			
			nokia_lcd_set_cursor(0,30);
			nokia_lcd_write_string("Realizado ",1);
			
			ag_set = 1;
			tempAnt_ag = tempo_ms;
		}
		
		
		nokia_lcd_render();
	
}

void plot_grafico(uint8_t cont, uint8_t x)
{
	char vol_s[3];
	sprintf(vol_s, "%d", cont);
	
	nokia_lcd_set_cursor(0,1);
	nokia_lcd_write_string("8|",1);
	nokia_lcd_set_cursor(0,8);
	nokia_lcd_write_string(" |",1);
	nokia_lcd_set_cursor(0,14);
	nokia_lcd_write_string(" |",1);
	nokia_lcd_set_cursor(0,20);
	nokia_lcd_write_string("0|",1);
	nokia_lcd_set_cursor(0,28);
	nokia_lcd_write_string("_____________",1);
	nokia_lcd_set_cursor(0,40);
	nokia_lcd_write_string(vol_s,1);
	nokia_lcd_write_string(" vol O2",1);
	
	nokia_lcd_set_pixel(x+10,25 - 3*cont, 100);
	nokia_lcd_set_pixel(x+11,25 - 3*cont, 100);
	nokia_lcd_render();
	if(x == 59)
	nokia_lcd_clear();
}
void plot_tx()
{
	nokia_lcd_clear();
	if (cursor == 1)
	{
		nokia_lcd_set_cursor(0,0);
		nokia_lcd_write_string("Saida Serial",1);
		nokia_lcd_set_cursor(0,20);
		nokia_lcd_write_string("Parametro:",1);
		nokia_lcd_set_cursor(0,30);
		nokia_lcd_write_string("Nenhum",1);
		
		set_tx = 0;
		
	}
	else if(cursor == 2)
	{
	   
	   nokia_lcd_set_cursor(0,0);
	   nokia_lcd_write_string("Saida Serial",1);
	   nokia_lcd_set_cursor(0,20);
	   nokia_lcd_write_string("Parametro:",1);
	   nokia_lcd_set_cursor(0,30);
	   nokia_lcd_write_string("BPM",1);
	   dado = bpm;
	   set_tx = 1;
		
	}
	else if(cursor == 3)
	{
		
		nokia_lcd_set_cursor(0,0);
		nokia_lcd_write_string("Saida Serial",1);
		nokia_lcd_set_cursor(0,20);
		nokia_lcd_write_string("Parametro:",1);
		nokia_lcd_set_cursor(0,30);
		nokia_lcd_write_string("Satur02",1);
		dado = spo2;
		set_tx = 1;
		
	}
	else if(cursor == 4)
	{
		
		nokia_lcd_set_cursor(0,0);
		nokia_lcd_write_string("Saida Serial",1);
		nokia_lcd_set_cursor(0,20);
		nokia_lcd_write_string("Parametro:",1);
		nokia_lcd_set_cursor(0,30);
		nokia_lcd_write_string("Temperatura",1);
		dado = temperatura;
		set_tx = 1;
		
	}
	
	nokia_lcd_render();
}


int sensor_temperatura(uint16_t lt_ADC)
{
	
	float Vin = 5*(lt_ADC/1024.);
	return 10*Vin + 10;

}
int sensor_spo2(uint16_t lt_ADC)
{
	
	float Vin = 5*(lt_ADC/1024.);
	return 25*Vin;
	
}

void alerta(uint8_t temperatura, uint8_t spo2)
{
	if(temperatura < 35 || temperatura > 41 || spo2 < 60)
	{
		
		PORTB |= 0b00001000;
	}
	
	else
	{
		PORTB &= 0b11110111 ;
	}
	
}

void USART_init(unsigned int ubrr)
{
	//Ajusta a taxa de transmissão
	
	UBRR0H = (unsigned int)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	// habilita o receptor e a interrupção
	UCSR0B = (1<<RXEN0)|(1<<RXCIE0)|(1<<TXEN0);
	
	// 1 stop bit 8 bits de dados
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
	
}
void valvula_o2(uint8_t nivel)
{
	uint8_t  ind = nivel/10;
	uint16_t orcb_val[11] = {2000,2200,2400,2600,2800,3000,3200,3400,3600,3800,4000};
	
	OCR1B = orcb_val[ind];
}

void usart_tx(uint8_t data)
{
	while(!(UCSR0A & (1<<UDRE0)));// verifica se o registrador de transmissão está vazio
	
	UDR0 = data; //envia 
}