# 센서 데이터를 이용한 TCP 통신

## TCP 통신 원리
1. 우선 데이터 전송 전 3-Way HandShake를 시작함.
(STM32 TO PC로 SYN 전송) -> (PC TO STM32로 요청을 수락하고 응답) -> (STM32 TO PC 응답 확인)-> (센서 데이터 송수신) 코드는 다음과 같다

   `tcp_connect(client_pcb, &server_ip, 5000, send_To_tcp);`
2. 센서 데이터를 송 수신
   
