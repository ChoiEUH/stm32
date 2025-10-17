import openai
import serial

client = openai.OpenAI(api_key="내 openai의 apikey 직접 입력")

ser = serial.Serial('COM5', 9600, timeout=1)

while True:
    line = ser.readline().decode().strip()
    if line:
        print("센서값 수신:", line)

        prompt = f"""
센서에서 받은 쌩 자기장 값은 다음과 같아:
{line}

이 값을 바탕으로, 현재 방향이 어디인지 알려줘. (예를들면 북쪽, 남동쪽 등)
가능하다면 대략적인 각도도 함께 말해줘.
"""

        try:
            response = client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "너는 나침반 전문가야. 자기장 값을 바탕으로 방향을 분석해."},
                    {"role": "user", "content": prompt}
                ]
            )

            print("\n 방향 추론 결과:")
            print(response.choices[0].message.content)

        except openai.APIError as e:
            print("OpenAI 에러:", e)
        except Exception as e:
            print("Normal 에러:", e)
