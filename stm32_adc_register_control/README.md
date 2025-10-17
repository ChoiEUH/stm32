# ADC 레지스터를 이용한 단순 센서데이터 수집

### STM32F429ZI사용, NTSF-4 써미스터(온도센서) 이용


1. 폴링 방식 접근

2. 전압값으로 변경 후 온도값 출력을 위한 수식 성립
<img width="907" height="106" alt="image" src="https://github.com/user-attachments/assets/40e11c6b-42d0-43e5-882c-a33664215d15" />

식을 증명 한다면, NTC써미스터로서, 온도가 증가하면 저항이 감소하는 특성을 가지기에, 식을 세우면 이렇게 지수 함수로 표현 된다..

<img width="499" height="122" alt="image" src="https://github.com/user-attachments/assets/1a2ddc65-c8cc-404f-be1f-fbb59f8594ba" />



우리가 원하는 것은 현재 온도 T이기에, 이 T를 구하기 위해서는 지수함수를 선형 함수로 바꿔야 하기에, 곱셈 로그 공식을 사용한 후 양변에 대해 정리하면 온도 T를 구할 수 있다.
고로 나온 식은.
<img width="482" height="159" alt="image" src="https://github.com/user-attachments/assets/5ce480f1-6ca9-447a-bd78-6a90f0435615" />





하지만, 이 온도는 화씨이므로, 섭씨로 변경하기 위해선, 도출된 온도에서 -273.15를 뺀다.
<img width="265" height="46" alt="image" src="https://github.com/user-attachments/assets/8b056ab1-5343-4721-8ed6-35979b8aba17" />


