/*
 * W25Q16.h
 *
 *  Created on: 13 de ago de 2022
 *      Author: Fagner
 */

#ifndef W25Q16_H_
#define W25Q16_H_

//Prot�tipos de fun��es de acesso � mem�ria Flash W25Q16
void SPI1_W25Q16_Init(void);				//inicializa��o da interface SPI1
uint8_t Read_Status_Register1(void);		//leitura do registrador de status 1
void Write_Enable(void);					//habilita a escrita
void Write_Disable(void);					//habilita prote��o de escrita
uint8_t Read_Data(uint32_t address);		//leitura de um byte
void Sector_Erase(uint32_t address);		//apaga o conte�do de um setor 4kB
void _32kB_Block_Erase(uint32_t address);	//apagamento de bloco de 32 kB
void _64kB_Block_Erase(uint32_t address);	//apagamento de bloco de 64 kB
void Chip_Erase(void);								//apaga o conte�do do chip
void Page_Program(uint32_t address, uint8_t data);	//grava��o de um byte
uint64_t Read_Unique_ID_Number(void);				//leitura do ID �nico de 64 bits

//Declara��o de fun��es de acesso � mem�ria Flash W25Q16
//Inicializa��o da interface SPI1 para a mem�ria
void SPI1_W25Q16_Init()
{
	//inicializando a interface SPI
	//configurando os pinos PB0 como sa�da CS e (PB3, PB4 e PB5) da interface SPI no modo de fun��o alternativa
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	//habilita o clock do GPIOB
	GPIOB->ODR |= 1;						//pino PB0 inicialmente em n�vel alto
	//modo de sa�da em PB0 (CS) e fun��o alternativa (SPI1) em PB3, PB4 e PB5
	GPIOB->MODER |= (0b10 << 10) | (0b10 << 8) | (0b10 << 6) | (0b01) ;
	GPIOB->AFR[0] |= (0b0101 << 20) | (0b0101 << 16) | (0b0101 << 12);

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;	//habilita o clock da interface SPI1
	SPI1->CR1 &= ~(SPI_CR1_CPOL |		//polaridade LOW
			       SPI_CR1_CPHA |		//dado lido na primeira borda (modo 0)
				   SPI_CR1_DFF |		//8 bits de dados
				   SPI_CR1_LSBFIRST);	//bit mais significativo transmitido primeiro
	//(pino SS controlado por software)
	//baud rate 10,5 Mbps
	//configura o modo master e habilita a interface
	SPI1->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_SPE | (0b010 << 3));
}

//Leitura do registrador de status 1
uint8_t Read_Status_Register1(void)
{
	GPIOB->ODR &= ~1;					//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));  	//aguarda o buffer Tx estar vazio
	SPI1->DR = 0x05;					//envia comando para leitura do SR1 (05h)
	while(!(SPI1->SR & SPI_SR_RXNE));	//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;						//l� o dummy byte enviado pelo escravo
	SPI1->DR = 0xFF;					//envia dummy byte para receber o valor do SR1
	while(!(SPI1->SR & SPI_SR_RXNE));	//aguarda o valor de SR1 ser recebido
	GPIOB->ODR |= 1;					//faz o pino CS ir para n�vel alto (encerra o comando)
	return SPI1->DR;					//retorna o valor do SR1
}

//habilita��o de escrita na mem�ria
void Write_Enable(void)
{
	while(Read_Status_Register1() & 1);			//aguarda a mem�ria estar dispon�vel
	GPIOB->ODR &= ~1;							//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));			//aguarda o buffer Tx estar vazio
	SPI1->DR = 0x06;							//envia comando para habilita��o de escrita (06h)
	while(!(SPI1->SR & SPI_SR_RXNE));			//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;								//l� o dummy byte enviado pelo escravo
	GPIOB->ODR |= 1;							//faz o pino CS ir para n�vel alto (encerra o comando)
	while(!(Read_Status_Register1() & 0b10));	//aguarda a grava��o estar habilitada
}

//habilita prote��o de escrita na mem�ria
void Write_Disable(void)
{
	while(Read_Status_Register1() & 1);			//aguarda a mem�ria estar dispon�vel
	GPIOB->ODR &= ~1;							//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));			//aguarda o buffer Tx estar vazio
	SPI1->DR = 0x04;							//envia comando para habilita��o da prote��o de escrita (04h)
	while(!(SPI1->SR & SPI_SR_RXNE));			//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;								//l� o dummy byte enviado pelo escravo
	GPIOB->ODR |= 1;							//faz o pino CS ir para n�vel alto (encerra o comando)
	while((Read_Status_Register1() & 0b10));	//aguarda a grava��o estar desabilitada
}

//leitura de um byte na mem�ria
uint8_t Read_Data(uint32_t address)
{
	while(Read_Status_Register1() & 1);		//aguarda a mem�ria estar dispon�vel
	GPIOB->ODR &= ~1;						//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI1->DR = 0x03;						//envia comando para ler um dado (03h)
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	uint8_t RX = SPI1->DR;					//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF0000) >> 16;	//envia o byte 3 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	RX = SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF00) >> 8;		//envia o byte 2 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	RX = SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF);			//envia o byte 1 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	RX = SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = 0xFF;						//envia um dummy byte
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o byte armazenado na mem�ria ser recebido
	RX = SPI1->DR;							//l� o byte gravado na mem�ria
	GPIOB->ODR |= 1;						//faz o pino CS ir para n�vel alto (encerra o comando)

	return RX;								//retorna com o byte armazenado
}

//apagamento de setor
void Sector_Erase(uint32_t address)
{
	Write_Enable();							//habilita a escrita na mem�ria
	while(Read_Status_Register1() & 1);		//aguarda a mem�ria estar dispon�vel
	GPIOB->ODR &= ~1;						//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI1->DR = 0x20;						//envia comando para apagar o setor (20h)
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF0000) >> 16;	//envia o byte 3 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF00) >> 8;		//envia o byte 2 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF);			//envia o byte 1 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	GPIOB->ODR |= 1;						//faz o pino CS ir para n�vel alto (encerra o comando)
}

//apagamento de bloco de 32 kB
void _32kB_Block_Erase(uint32_t address)
{
	Write_Enable();							//habilita a escrita na mem�ria
	while(Read_Status_Register1() & 1);		//aguarda a mem�ria estar dispon�vel
	GPIOB->ODR &= ~1;						//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI1->DR = 0x52;						//envia comando para apagar o bloco (52h)
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF0000) >> 16;	//envia o byte 3 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF00) >> 8;		//envia o byte 2 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF);			//envia o byte 1 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	GPIOB->ODR |= 1;						//faz o pino CS ir para n�vel alto (encerra o comando)
}

//apagamento de bloco de 64 kB
void _64kB_Block_Erase(uint32_t address)
{
	Write_Enable();							//habilita a escrita na mem�ria
	while(Read_Status_Register1() & 1);		//aguarda a mem�ria estar dispon�vel
	GPIOB->ODR &= ~1;						//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI1->DR = 0xD8;						//envia comando para apagar o bloco (D8h)
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF0000) >> 16;	//envia o byte 3 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF00) >> 8;		//envia o byte 2 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF);			//envia o byte 1 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	GPIOB->ODR |= 1;						//faz o pino CS ir para n�vel alto (encerra o comando)
}

//apagamento completo do chip
void Chip_Erase(void)
{
	Write_Enable();						//habilita a escrita na mem�ria
	while(Read_Status_Register1() & 1);	//aguarda a mem�ria estar dispon�vel
	GPIOB->ODR &= ~1;					//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));	//aguarda o buffer Tx estar vazio
	SPI1->DR = 0xC7;					//envia comando para apagar o chip (C7h)
	while(!(SPI1->SR & SPI_SR_RXNE));	//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;						//l� o dummy byte do escravo
	GPIOB->ODR |= 1;					//faz o pino CS ir para n�vel alto (encerra o comando)
}

//grava��o de um byte na mem�ria
void Page_Program(uint32_t address, uint8_t data)
{
	Write_Enable();							//habilita a escrita na mem�ria
	while(Read_Status_Register1() & 1);		//aguarda a mem�ria estar dispon�vel
	GPIOB->ODR &= ~1;						//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI1->DR = 0x02;						//envia comando para escrever um dado (02h)
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF0000) >> 16;	//envia o byte 3 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF00) >> 8;		//envia o byte 2 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = (address & 0xFF);			//envia o byte 1 do endere�o
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = data;						//envia o dado a ser gravado
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	GPIOB->ODR |= 1;						//faz o pino CS ir para n�vel alto (encerra o comando)
}

//leitura do ID �nico de 64 bits
uint64_t Read_Unique_ID_Number(void)
{
	while(Read_Status_Register1() & 1);		//aguarda a mem�ria estar dispon�vel
	GPIOB->ODR &= ~1;						//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI1->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI1->DR = 0x4B;						//envia comando para ler o ID de 64 bits (4Bh)
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	SPI1->DR = 0xFF;						//envia o dummy byte 1
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo
	SPI1->DR = 0xFF;						//envia o dummy byte 2
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo
	SPI1->DR = 0xFF;						//envia o dummy byte 3
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo
	SPI1->DR = 0xFF;						//envia o dummy byte 4
	while(!(SPI1->SR & SPI_SR_RXNE));		//aguarda o dummy byte do escravo ser recebido
	(void)SPI1->DR;							//l� o dummy byte do escravo

	uint64_t ID = 0;			//vari�vel que vai receber o ID
	uint8_t cont = 1;
	while(cont <= 8)
	{
		SPI1->DR = 0xFF;							//envia um dummy byte
		while(!(SPI1->SR & SPI_SR_RXNE));			//aguarda um byte do ID ser recebido
		ID |= ((uint64_t)SPI1->DR << (8*(8-cont)));	//l� o byte do ID
		++cont;
	}

	GPIOB->ODR |= 1;	//faz o pino CS ir para n�vel alto (encerra o comando)
	return ID;			//retorna com o ID
}


#endif /* W25Q16_H_ */
