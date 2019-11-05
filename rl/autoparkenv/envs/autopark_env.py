import gym
from gym import error, spaces, utils
from gym.utils import seeding
import random
import numpy as np
import math
import copy
import tkinter as tk


# SIZE 16*18
WIDTH = 15
HEIGHT = 25
avmw = 80
avmh = 120
# 단위 격자 (10 cm) 블록 크기
BS = 5

# 1m 를 10 등분
UNIT = 10
## Map  Digital 형태 , Map 파라미터
Ori_map = [[0] * WIDTH * UNIT for _ in range(HEIGHT*UNIT)]
map = [[0] * WIDTH * UNIT for _ in range(HEIGHT*UNIT)]
avm = np.zeros((avmh, avmw))
## 전역 함수


# 회전변환
def rot_convert(Occupy_map, theta):
    convert_Occupy_map = [[0, 0] for __ in range(len(Occupy_map))]

    for index in range(len(Occupy_map)):
        convert_Occupy_map[index][0] = math.cos(theta) * Occupy_map[index][0] - math.sin(theta) * Occupy_map[index][1]
        convert_Occupy_map[index][1] = math.sin(theta) * Occupy_map[index][0] + math.cos(theta) * Occupy_map[index][1]

    return convert_Occupy_map



## 자동차 Agent 클래스

class Car_Agent():

    # 초기 설정
    def __init__(self):

        self.rp = [50, 30]  # 위치 기준점 x , y
        self.pv_rp = [0, 0]

        self.gear = 0
        self.step = 10

        self.w = 90 * math.pi / 180

        self.width = int(1.8 * UNIT)
        self.height = int(4.8 * UNIT)

        # Action Space 정의
        self.action_w = [0, 15 * math.pi / 180, 30 * math.pi / 180, -15 * math.pi / 180, -30 * math.pi / 180]

        self.area = (self.width) * (self.height)
        self.Occupied = [[0, 0] for __ in range(int(self.area))]
        self.pv_Occupied = [[0, 0] for __ in range(int(self.area))]

        for j in range(0, self.height):
            for i in range(0, self.width):
                self.Occupied[i + j * (self.width)][0] = i  # + self.rp[0]
                self.Occupied[i + j * (self.width)][1] = j  # + self.rp[1]

        # self.Occupied = rot_convert(self.Occupied, 0.75)
        #self.update_map()
        # print(len(self.Occupied))

    # AVM 이미지를 활용하려면 타겟 주차선 근처까지 이동을 가정
    def move_target(self, target_line):
        pass

    # Map 데이터를 받아 현재 차 점유공간 마킹
    def update_map(self):

        # 회귀

        for index in range(self.area):
            x = round(self.pv_Occupied[index][0]) + self.pv_rp[0]
            y = round(self.pv_Occupied[index][1]) + self.pv_rp[1]
            map[y][x] = Ori_map[y][x]

        # 새로운 점유 공간지도 대로 다시 마킹

        for index in range(self.area):
            x = round(self.Occupied[index][0]) + self.rp[0]
            y = round(self.Occupied[index][1]) + self.rp[1]
            map[y][x] = 4

    # 실제 행동 수행 0 - 전진 , 1 - 좌향  2 - 우향
    def aaction(self, action_num, state):

        self.pv_rp = self.rp
        self.pv_Occupied = self.Occupied

        # 전진기어
        if self.gear == 0:

            # 회전
            self.Occupied = rot_convert(self.Occupied, self.action_w[action_num])


            # 방향 벡터 회전
            self.w += self.action_w[action_num]

            # 기준점 이동

            self.rp[0] = self.rp[0] + round(math.cos(self.w) * self.step)
            self.rp[1] = self.rp[1] + round(math.sin(self.w) * self.step)


        # 후진기어
        else:
            # 회전
            self.Occupied = rot_convert(self.Occupied, self.action_w[action_num])


            # 방향 벡터 회전
            self.w += self.action_w[action_num]

            # 기준점 이동

            self.rp[0] = self.rp[0] - int(math.cos(self.w - self.action_w[action_num]) * self.step)
            self.rp[1] = self.rp[1] - int(math.sin(self.w - self.action_w[action_num]) * self.step)



        print(" 현재 기어 :" , self.gear , " 현재 방향각 : " , self.w * 180 / math.pi)
        self.update_map()
        state = rot_convert(state, self.action_w[action_num])
        return np.array(state,dtype=int)

    # AVM 형태 로 데이터 받아들이기
    def AVM_cognition(self,avmxy):
        avm = np.zeros((avmh,avmw))
        i = 0
        for y in range(avmh):
            for x in range(avmw):
                avm[y][x] = map[avmxy[i][0]][avmxy[i][1]]
                i+=1
        return avm
    def AVM_xy(self):
        avmxy = np.zeros((avmw*avmh,2),dtype=int)
        i = 0
        for y in range(self.rp[1]-30,self.rp[1]+90):
            for x in range(self.rp[0]-30,self.rp[0]+50):
                avmxy[i][0] = y
                avmxy[i][1] = x
                i+=1
        return avmxy

class AutoParkEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self):
        self.car = Car_Agent()
        self.statexy = self.car.AVM_xy()
        self.state = self.car.AVM_cognition(self.statexy)
        self.isOpened = False
        self.root = tk.Tk()
        self.root.title("Map")

        # canvas = tk.Canvas(root, bg='black',
        #                    height=HEIGHT * UNIT,
        #                    width= (WIDTH) * UNIT)

        self.canvas = tk.Canvas(self.root, bg='white',
                                height=120 * BS,
                                width=80 * BS)
        self.canvas.pack()

        # 전진/후진 각각 5 Action
        # 키 입력은
        self.canvas.focus_set()
        self.canvas.bind("<Up>", lambda _: self.step(0))
        self.canvas.bind("<Left>", lambda _: self.step(3))
        self.canvas.bind("<Right>", lambda _: self.step(1))
        self.canvas.bind("<Down>", lambda _: self.change_gear())

        self.rect_Occupied = []

        # self.DrawMap()

        self.Vallet_size = [ int(2.5*UNIT), 5*UNIT]
        self.Vallet_point = [ [2, 5*UNIT], [2, int(7.5*UNIT)], [2, 10*UNIT], [2, int(12.5*UNIT)],
                         [WIDTH*UNIT-self.Vallet_size[1]-1, 5*UNIT], [WIDTH*UNIT-self.Vallet_size[1]-1, int(7.5*UNIT)], [WIDTH*UNIT-self.Vallet_size[1]-1, 10*UNIT], [WIDTH*UNIT-self.Vallet_size[1]-1, int(12.5*UNIT)] ]
        self.Vallet_width = 0.2*UNIT


    def step(self, action):
        self.statexy = self.car.aaction(action,self.statexy)
        self.state = self.car.AVM_cognition(self.statexy)
        self.DrawMap()
        print(self.car.rp)
        # done 결정
#         if(차선에 닿음 감지 or 주차 완료)
#             done = True
#         #reward 결정
#         if(차선에 닿음)
#             reward = -1
#         else (주차 완료 / direction 일치 / )
#             reward = 1
#         reward = reward - 0.001*frame

#          # 현재 상태 벡터, 보상 여부, 완료 여부 반환
#         return np.array(self.state), reward, done, {}


    def reset(self):

        self.car = Car_Agent()

        map = [[0] * WIDTH * UNIT for _ in range(HEIGHT*UNIT)]
        ## 주차장 환경 생성
        # 주차장 전체 라인
        self.Fill_Rect_Area([0,0], HEIGHT*UNIT , WIDTH*UNIT , 2 , HEIGHT*UNIT, WIDTH*UNIT , 2)

        # 주차선 라인
        self.Vallet_size = [ int(2.5*UNIT), 5*UNIT]
        self.Vallet_point = [ [2, 5*UNIT], [2, int(7.5*UNIT)], [2, 10*UNIT], [2, int(12.5*UNIT)],
                         [WIDTH*UNIT-self.Vallet_size[1]-1, 5*UNIT], [WIDTH*UNIT-self.Vallet_size[1]-1, int(7.5*UNIT)], [WIDTH*UNIT-self.Vallet_size[1]-1, 10*UNIT], [WIDTH*UNIT-self.Vallet_size[1]-1, int(12.5*UNIT)] ]
        self.Vallet_width = 0.2*UNIT
        # 주차선 마킹
        for index in range(len(self.Vallet_point)):
            self.Fill_Rect_Area(self.Vallet_point[index], self.Vallet_size[0] , self.Vallet_size[1] , 1 , HEIGHT*UNIT, WIDTH*UNIT , 1)

        # 목표 주차선 선정
        Target = random.randrange(8)
        self.Fill_Rect_Area(self.Vallet_point[Target], self.Vallet_size[0] , self.Vallet_size[1] , 1 , HEIGHT*UNIT, WIDTH*UNIT , 3)

        # 랜덤으로 주차된 차량 생성
        random_count = random.randrange(7)


        # 정적 맵 기억
        Ori_map = copy.deepcopy(map)

        self.statexy = self.car.AVM_xy()
        self.state = self.car.AVM_cognition(self.statexy)

    def render(self, modes='human', close=False):
        if self.isOpened == False:
            self.DrawMap()
            self.isOpened = True
        else:
            self.root.update()

        # 직사각형 형태의 Boundary 를 FILL 성분으로 만듬
    def Fill_Rect_Area(self, REF, AREA_HEIGHT, AREA_WIDTH, LINE_WIDTH, MAP_HEIGHT, MAP_WIDTH, FILL):
        LU = [REF[0], REF[1]]
        RU = [REF[0] + AREA_WIDTH - 1, REF[1]]
        LD = [REF[0], REF[1] + AREA_HEIGHT - 1]
        RD = [REF[0] + AREA_WIDTH - 1, REF[1] + AREA_HEIGHT - 1]

        for i in range(LU[0], RU[0] + 1):
            for j in range(LINE_WIDTH):
                map[j + REF[1]][i] = FILL

        for j in range(LU[1], LD[1] + 1):
            for i in range(LINE_WIDTH):
                map[j][i + REF[0]] = FILL

        for j in range(RU[1], RD[1] + 1):
            for i in range(LINE_WIDTH):
                map[j][RU[0] - i] = FILL

        for i in range(LD[0], RD[0] + 1):
            for j in range(LINE_WIDTH):
                map[LD[1] - j][i] = FILL


# Visualization 관련 함수
    def DrawMap(self):
        print(self.state.shape)
        for x in range(80):
            for y in range(120):

                # 0 -> 빈 공간 , 1 -> 장애물 공간 , 2 -> 주차선 ,  3 -> 목표 주차선 ,  4 -> 차량

                if self.state[y][x] == 0:
                    color = "white"
                elif self.state[y][x] == 1:
                    color = "black"
                elif self.state[y][x] == 2:
                    color = "green"
                elif self.state[y][x] == 3:
                    color = "blue"
                elif self.state[y][x] == 4:
                    color = "red"

                rect = self.canvas.create_rectangle(x * BS, y * BS, (x + 1) * BS, (y + 1) * BS
                                                    , fill=color, tag="rect")

    # 동적인 부분만 시각 업데이트 (맵 전체를 다시 그리면 느림)
    def MoveCar(self):

        # 기존 Car 영역 삭제
        for i in self.rect_Occupied:
            self.canvas.delete(i)
        self.rect_Occupied = []

        # 새로운 Occupied 영역 갱신
        for i in range(len(self.car.Occupied)):

            x = round(self.car.Occupied[i][0] + self.car.rp[0])
            y = round(self.car.Occupied[i][1] + self.car.rp[1])
            #print(x, y)
            if map[y][x] == 0:
                color = "white"
            elif map[y][x] == 1:
                color = "black"
            elif map[y][x] == 2:
                color = "green"
            elif map[y][x] == 3:
                color = "blue"
            elif map[y][x] == 4:
                color = "red"

            rect = self.canvas.create_rectangle(x * BS, y * BS, (x + 1) * BS, (y + 1) * BS
                                                , fill=color, tag="rect")
            self.rect_Occupied.append(rect)

            pass

        pass

    def change_gear(self):
        print("기어 변환")
        if self.car.gear == 0:
            self.car.gear = 1
        else:
            self.car.gear = 0

    def check_collision(self):


        return True

    ## 메인문

if __name__ == "__main__":
        env = AutoParkEnv()
        env.reset()
        while True:
            env.render()

