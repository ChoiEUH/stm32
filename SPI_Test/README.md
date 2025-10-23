# SPI통신 (STM32 + Arduino)

## SPI 통신이란? 
Serial Peripheral Interface의 약자인 SPI로서, I2C와 같은 동기식 직렬 통신이며, 마스터와 슬레이브 간에 데이터를 동시에 교환하는 풀듀플렉스(Full-Duplex) 직렬통신방식이다.

## 통신 순서

1.CS LOW 상태로 슬레이브를 선택하고 통신 시작

2. STM32 To Arduino로 0xAA 데이터 전송(MOSI)
   
3.Arduino To Stm32로 응답 준비(MISO)

4. STM32 1바이트 데이터 수신
   
5.STM32 TO Arduino로 0x55 데이터 전송(MOSI)

6.Arduino To Stm32로 응답 준비(MISO)

7. STM32 2바이트 데이터 수신
   
8. CS핀 HIGH 상태로 슬레이브 비활성화
