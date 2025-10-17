#include <SPI.h>

#define PACKET_SIZE 2
volatile byte spi_rx_buffer[PACKET_SIZE]; 
volatile byte byte_count = 0;             
volatile bool packet_received = false;    


void setup() {
    pinMode(MISO, OUTPUT);        
    pinMode(10, INPUT_PULLUP);   

    SPCR = _BV(SPE) | _BV(SPIE); 
    
    Serial.begin(115200); 
    Serial.println("Arduino SPI Slave Ready.");
}

ISR(SPI_STC_vect) {
   
    byte received_data = SPDR; 

  
    if (byte_count < PACKET_SIZE) {
        spi_rx_buffer[byte_count] = received_data;
    }
    

    if (byte_count == 0) {
      
        SPDR = 0xAA; 
    } else if (byte_count == 1) {
        
        SPDR = 0x55; 
    }
    


 
    if (byte_count == PACKET_SIZE) {
        packet_received = true;
        byte_count = 0; 
    }
}

void loop() {

    if (packet_received) {
        
        noInterrupts();
        
        Serial.print("Received: 0x");
        Serial.print(spi_rx_buffer[0], HEX); 
        Serial.print(" 0x");
        Serial.println(spi_rx_buffer[1], HEX); 
        
        
        packet_received = false;
        
        interrupts(); 
    }
}
