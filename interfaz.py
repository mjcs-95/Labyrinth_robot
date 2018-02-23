# coding=utf-8
import Tkinter as tk
import serial
import time
import threading

BTaddr = 'COM6'  #"98:D3:31:FD:31:5B"
BTaddr2= 'COM5'
# import serial
# import time

WIDTH = 100  # size of every rectangle in grid
LABYRINTH_SIZE = 5  # Size of the dimensions of the labyrinth


class Application(tk.Frame):
    # Constructor
    def __init__(self, master=None):
        # Robot's position and direction
        self.robot_y = LABYRINTH_SIZE - 1
        self.robot_x = LABYRINTH_SIZE - 1
        self.direction_ = 0  # 0 up, 1 right, 2 down, 3 left
        # UInterface
        tk.Frame.__init__(self, master)
        # grid
        self.grid()
        self.create_widgets()
        self.draw_labyrinth(self.robot_y, self.robot_x)
        # Bluetooth

    def bluetooth_start_robot(self):
        self.bluetooth2.write(b"s")
        threading.Thread(target=self.read_bluetooth).start() #self.read_bluetooth()

    def connectBT(self):
        #self.bluetooth1 = serial.Serial(BTaddr, 9600) #Entrada
        self.bluetooth2 = serial.Serial(BTaddr2, 9600)#salida
        time.sleep(2)
        #self.bluetooth1.flushInput()
        self.bluetooth2.flushInput()
        print("Connected")
        #self.read_bluetooth()


    def read_bluetooth(self):
        fin=0;
        while True:
            self.data = self.bluetooth2.read(1)
            #self.data.decode()
            #print(self.data)
            if (self.data== b"r"):
                print("right")
                self.direction(0)
                #print("right")
            elif (self.data == b"l"):
                self.direction(1)
                print("left")
            elif (self.data == b"b"):
                self.direction(2)
                print("back")
            elif (self.data == b"m"):
                self.movement()
                self.draw_labyrinth(self.robot_y, self.robot_x) #threading.Thread(target=self.worker).start()
                print("move")
            time.sleep(0.01)


    def create_widgets(self):
        # Labels
        """
        self.LcellsCount= tk.Label(self, text="Celdas Recorridas(se cuentan las repetidas):")
        self.LcellsCount.grid(sticky="W")
        self.L_Obstacles= tk.Label(self, text="Obstaculos:")
        self.L_Obstacles.grid(sticky="W")
        self.L_Speed = tk.Label(self, text="Velocidad:")
        self.L_Speed.grid(sticky="W")
        self.LDistance  = tk.Label(self, text="Distancia:")
        self.LDistance.grid(sticky="W")
        self.LTime   = tk.Label(self, text="Tiempo:")
        self.LTime.grid(sticky="W")
        self.LDirection = tk.Label(self, text="Dirección:")
        self.LDirection.grid(sticky="W")
        """
        # Buttons
        self.BTButton = tk.Button(self, text='Conectar Bluetooth', command=lambda: self.connectBT())
        self.BTButton.grid()
        self.directionButton = tk.Button(self, text='Empezar  recorrido', command=lambda: self.bluetooth_start_robot())
        self.directionButton.grid()

        #self.directionButton = tk.Button(self, text='Turn', command=lambda: self.direction(0))
        #self.directionButton.grid()
        #self.moveButton = tk.Button(self, text='Move', command=self.movement)
        #self.moveButton.grid()
        self.quitButton = tk.Button(self, text='Cerrar la Aplicación', command=quit)
        self.quitButton.grid()
        self.UIGrid = tk.Canvas(self, width=500, height=500, bg="black")
        self.UIGrid.grid()
        for i in range(LABYRINTH_SIZE):
            for j in range(LABYRINTH_SIZE):
                self.UIGrid.create_rectangle(j * WIDTH, i * WIDTH, j * WIDTH + WIDTH, i * WIDTH + WIDTH, fill='white')

    def draw_labyrinth(self, i, j):
        self.UIGrid.create_rectangle(j * WIDTH, i * WIDTH, j * WIDTH + WIDTH, i * WIDTH + WIDTH, fill='yellow')

    def direction(self, turnID):
        if  ( turnID == 0 ):       # der
            self.direction_ = (self.direction_ + 1)
        elif( turnID == 1 ):     # izq
            self.direction_ = (self.direction_ - 1)
        elif( turnID == 2 ):     # 180º
            self.direction_ = (self.direction_ - 2)
        self.direction_ = (self.direction_ + 4) % 4
        print(self.direction_)

    def movement(self):
        if  (self.direction_ == 0):
            self.robot_y = self.robot_y-1
        elif(self.direction_ == 1):
            self.robot_x = self.robot_x+1
        elif(self.direction_ == 2):
            self.robot_y = self.robot_y+1
        elif(self.direction_ == 3):
            self.robot_x = self.robot_x-1
        #threading.Thread(target=self.worker).start()
        print('{0} {1}'.format(self.robot_y, self.robot_x))

    def worker(self):
        self.draw_labyrinth(self.robot_y, self.robot_x)
        return


#Ejecución del Programa
app = Application()
app.master.title('Robot Laberinto')
app.mainloop()
