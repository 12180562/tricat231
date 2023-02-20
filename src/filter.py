import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 데이터 프레임 가져오기 및 데이터 전처리

data = pd.read_csv('rotation_data.csv')
data = data.drop(0, axis = 0)
partial_data = data[1:100]
partial_data = partial_data.drop('x', axis = 1)
partial_data = partial_data.drop('y', axis = 1)
partial_data = partial_data.drop('z', axis = 1)
print(partial_data) 
print(partial_data.index)

# 필터 함수
# input: 데이터프레임 한 칼럼
# output: 리스트

def moving_average(input_data):
    filtered_data_list = []
    dataframe = input_data
    for i in dataframe.index:
        if i == dataframe.index[0] or i == dataframe.index[1]:
            filtered_data = 0
        else:
            filtered_data = (dataframe[i-2] + dataframe[i-1] + dataframe[i])/3
        filtered_data_list.append(filtered_data)
    return filtered_data_list

# input: 데이터프레임 한 칼럼, 지수 평활 계수
# output: 리스트

def exponential(input_data,alpha):
    filtered_data_list = []
    dataframe = input_data
    for i in dataframe.index:
        if i == dataframe.index[0] or i == dataframe.index[1]:
            filtered_data = 0
        else:
            filtered_data = ((dataframe[i-2]*(1-alpha)**2) + (dataframe[i-1]*(1-alpha)) + (dataframe[i]*alpha))/3
        filtered_data_list.append(filtered_data)
    return filtered_data_list

# 결과 확인
x = moving_average(partial_data['roll'])
y = exponential(partial_data['roll'],0.1)

plt.figure()
plt.plot(partial_data.index,partial_data['roll'])
plt.plot(partial_data.index, x)
plt.plot(partial_data.index, y)
plt.show()

# 고찰
# 주기성을 보이는 데이터에는 이동평균 평활법을 이용하는 것이 더 효과적이라고 판단
# 지수 평활법은 주기성 데이터보다는 랜덤성이 강한 경우에 사용하는 것이 더 적합해 보임


