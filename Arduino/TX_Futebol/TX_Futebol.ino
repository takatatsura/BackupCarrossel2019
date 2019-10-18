#include <SPI.h>
#include "API.h"
#include "nRF24L01.h"

//#define DEBUG
//***************************************************
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  9  // 4 unsigned chars TX payload

unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = { 'D', 'C', 'o', 'F', 'C' }; // Define a static TX address

unsigned char rx_buf[TX_PLOAD_WIDTH] = {
  0
}; // initialize value
unsigned char tx_buf[TX_PLOAD_WIDTH] = {
  0
};
//***************************************************
void setup() {
  Serial.begin(115200);
  pinMode(CE,  OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(IRQ, INPUT);
  SPI.begin();
  delay(50);
  init_io();                        // Initialize IO port
  unsigned char sstatus=SPI_Read(STATUS);
  Serial.print("TX ");
  Serial.println(sstatus,HEX);     // There is read the mode’s status register, the default value should be ‘E’
  TX_Mode();                       // set TX mode
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
}

void loop() {
  boolean comandoDisponivel = false, dataSent = false, maxRetransmits = false;
  int dataReady = 0;
  unsigned char sstatus;
  for(;;) {
    sstatus = SPI_Read(STATUS);                   // read register STATUS's value
    if (sstatus&TX_DS)
      dataSent = true;
    if (sstatus&RX_DR)
      dataReady++;
    if (sstatus&MAX_RT)
      maxRetransmits = true;
    SPI_RW_Reg(WRITE_REG+STATUS,sstatus);                     // clear RX_DR or TX_DS or MAX_RT interrupt flag
    
    if (leComando()) {
      //Serial.write(tx_buf, TX_PLOAD_WIDTH);
      comandoDisponivel = true;
      digitalWrite(7, LOW);
      digitalWrite(8, LOW);
    }
    if (comandoDisponivel) {
      //Serial.print("\n0");
      if(dataSent) {                                          // if receive data ready (TX_DS) interrupt
        dataSent = false;
        SPI_RW_Reg(FLUSH_TX,0);
        SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);       // write playload to TX_FIFO
        digitalWrite(7, HIGH);
        #ifdef DEBUG
        Serial.println("1");        
        #endif
        comandoDisponivel = false;
      }
      if(maxRetransmits) {                                        // if receive data ready (MAX_RT) interrupt, this is retransmit than  SETUP_RETR
        maxRetransmits = false;
        digitalWrite(8, HIGH);
        #ifdef DEBUG
        Serial.print("2");
        #endif
        SPI_RW_Reg(FLUSH_TX,0);
        //tx_buf[2]='R';
        SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);      // disable standy-mode
        comandoDisponivel = false;
      }
    }
    if(dataReady--) {        
      SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);             // read playload to rx_buf
      SPI_RW_Reg(FLUSH_RX,0);                                        // clear RX_FIFO
      //for (int i=0; i<TX_PLOAD_WIDTH; i++)
      //if (rx_buf[i] == 0)
      //  rx_buf[i]=0xfe;
      #ifdef DEBUG                                       // if receive data ready (TX_DS) interrupt
      Serial.print("\n!");
      Serial.write(rx_buf, TX_PLOAD_WIDTH);
      #endif
    }
  }
}

//rotina serial para receber o pacote de comandos do computador e envia-lo ao robs atravs do tranceiver (nRF24L01)

boolean leComando() {
  static int dado = 0;
  if (dado != 0x80) {
    dado = Serial.read();
    //Serial.write(dado);
  }
  if (dado == 0x80) {
    digitalWrite(6, HIGH);
    if (Serial.available() >= TX_PLOAD_WIDTH-1) {
      digitalWrite(6, LOW);
      Serial.readBytes((char*)&tx_buf[1], TX_PLOAD_WIDTH-1);
      tx_buf[0]=0x80;
      dado = -1;
      return true;
    }
  }
  return false;
}

/*********************************************************************
 **  Device:  nRF24L01+                                              **
 **  File:   EF_nRF24L01_TX.c                                        **
 **                                                                  **
 **                                                                  **
 **  Copyright (C) 2011 ElecFraks.                                   **
 **  This example code is in the public domain.                      **
 **                                                                  **
 **  Description:                                                    **
 **  This file is a sample code for your reference.                  **
 **  It's the v1.0 nRF24L01+ Hardware SPI by arduino                 **
 **  Created by ElecFreaks. Robi.W,11 June 2011                      **
 **                                                                  **
 **  http://www.elecfreaks.com                                       **
 **                                                                  **
 **   SPI-compatible                                                 **
 **   CS - to digital pin 2               - preto                    **
 **   CSN - to digital pin 3  (SS pin)    - marrom                   **
 **   MOSI - to digital pin 11 (MOSI pin) - laranja                  **
 **   MISO - to digital pin 12 (MISO pin) - amarelo                  **
 **   CLK - to digital pin 13 (SCK pin)   - vermelho                 **
 *********************************************************************/

//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void) {
  digitalWrite(IRQ, 0);
  digitalWrite(CE, 0);			// chip enable
  digitalWrite(CSN, 1);                 // Spi disable
}

/************************************************************************
 **   * Function: SPI_RW();
 *
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 ************************************************************************/
unsigned char SPI_RW(unsigned char Byte) {
  return SPI.transfer(Byte);
}

/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 *
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value) {
  unsigned char status;

  digitalWrite(CSN, 0);                   // CSN low, init SPI transaction
  SPI_RW(reg);                            // select register
  SPI_RW(value);                          // ..and write value to it..
  digitalWrite(CSN, 1);                   // CSN high again

  return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 *
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
/**************************************************/
unsigned char SPI_Read(unsigned char reg) {
  unsigned char reg_val;

  digitalWrite(CSN, 0);                // CSN low, initialize SPI communication...
  SPI_RW(reg);                         // Select register to read from..
  reg_val = SPI_RW(0);                 // ..then read register value
  digitalWrite(CSN, 1);                // CSN high, terminate SPI communication

  return(reg_val);                     // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 *
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes) {
  unsigned char sstatus,i;

  digitalWrite(CSN, 0);                   // Set CSN low, init SPI tranaction
  sstatus = SPI_RW(reg);       	    // Select register to write to and read status unsigned char

  for(i=0; i<bytes; i++) {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  digitalWrite(CSN, 1);                   // Set CSN high again

  return(sstatus);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 *
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes) {
  unsigned char sstatus,i;

  digitalWrite(CSN, 0);                   // Set CSN low, init SPI tranaction
  sstatus = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0; i<bytes; i++) {          // then write all unsigned char in buffer(*pBuf)
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSN, 1);                   // Set CSN high again
  return(sstatus);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: TX_Mode();
 *
 * Description:
 * This function initializes one nRF24L01 device to
 * TX mode, set TX address, set RX address for auto.ack,
 * fill TX payload, select RF channel, datarate & TX pwr.
 * PWR_UP is set, CRC(2 unsigned chars) is enabled, & PRIM:TX.
 *
 * ToDo: One high pulse(>10us) on CE will now send this
 * packet and expext an acknowledgment from the RX device.
 **************************************************/
void TX_Mode(void) {
  digitalWrite(CE, 0);

  SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack

  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...
  SPI_RW_Reg(WRITE_REG + RF_CH, 'R');        // Select RF channel 'R'
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
  #ifdef DEBUG
  SPI_RW_Reg(WRITE_REG + FEATURE, 6);       //Enables Dynamic Payload Length e Payload with ACK
  SPI_RW_Reg(WRITE_REG + DYNPD, 1);
  #endif

  SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);
  
  
  digitalWrite(CE, 1);
}


