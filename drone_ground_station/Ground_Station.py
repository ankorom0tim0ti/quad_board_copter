# inport needed library
import os
import sys
import time
import udp_module
import pygame
from pygame.locals import *
import subprocess
import csv

# function for gettting path of resource file
def resource_path(relative_path):
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except AttributeError:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)

# function for getting index of factor in dictionary
def get_index_dictionary(dictionary, target_value):
    keys = [key for key, value in dictionary.items() if value[0] == target_value]
    return keys[0]

# class for showing navigation on pygame screen
class ChannelAssignment:
    #initialize channel assignment
    def __init__(self, screen, position, size, title, button_map, default_value):
        # read input
        self._screen = screen
        self._position = position
        self._size = size
        self._title = title
        self._button_map = button_map
        self._current_index = get_index_dictionary(button_map, default_value)
    
        # definition of font of widget
        self._font = pygame.font.Font(None, 36)
        self._title_font = pygame.font.Font(None, 24)
        
        # definiton of arrow button
        self._arrow_color = (0, 255, 0)
        self._arrow_left_points = [
            (self._position[0], self._position[1] + self._size[1] // 2),
            (self._position[0] + 20, self._position[1] + self._size[1] // 2 - 20),
            (self._position[0] + 20, self._position[1] + self._size[1] // 2 + 20)
        ]
        self._arrow_right_points = [
            (self._position[0] + self._size[0], self._position[1] + self._size[1] // 2),
            (self._position[0] + self._size[0] - 20, self._position[1] + self._size[1] // 2 - 20),
            (self._position[0] + self._size[0] - 20, self._position[1] + self._size[1] // 2 + 20)
        ]
    
    # detect event
    def handle_events(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # left click
                if self.point_in_polygon(event.pos, self._arrow_left_points):
                    self._current_index = (self._current_index - 1) % len(self._button_map)
                elif self.point_in_polygon(event.pos, self._arrow_right_points):
                    self._current_index = (self._current_index + 1) % len(self._button_map)
        return(self._button_map[self._current_index][0])
    
    # judge whether cursor is in the arrow button
    def point_in_polygon(self, point, polygon):
        x, y = point
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n+1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    # draw widget component
    def draw(self):
        # draw title widget
        title_surface = self._title_font.render(self._title, True, (0, 0, 0))
        title_rect = title_surface.get_rect(topleft=(self._position[0] + 20, self._position[1] - 20))
        self._screen.blit(title_surface, title_rect)
        window_rect = pygame.Rect(self._position[0] + 25, self._position[1] + self._size[1] // 2 - 25, self._size[0] -50, self._size[1] - 10)
        pygame.draw.rect(self._screen, (0, 0, 0), window_rect)
        
        # draw
        text_surface = self._font.render(self._button_map[self._current_index][0], True, (0, 255, 0))
        text_rect = text_surface.get_rect(center=(self._position[0] + self._size[0] // 2, self._position[1] + self._size[1] // 2))
        self._screen.blit(text_surface, text_rect)
        
        # draw arrow widget
        pygame.draw.polygon(self._screen, self._arrow_color, self._arrow_left_points)
        pygame.draw.polygon(self._screen, self._arrow_color, self._arrow_right_points)

#class for showing graph on pygame screen
class Graph:
    # initialize graph
    def __init__(self, screen, top_left, size, title):
        # read input
        self._screen = screen
        self._top_left = top_left
        self._size = size
        self._max_y_value = -1
        self._min_y_value = 1
        self._title = title
    
    # draw graph componet
    def draw(self, data1, data2, color=(255, 0, 0)):
        # define scale of y axis
        max_abs_value = max(max(data2), abs(min(data2)), abs(self._max_y_value), abs(self._min_y_value))
        self._max_y_value = max_abs_value
        self._min_y_value = -max_abs_value

        # define background of graph
        x, y = self._top_left
        width, height = self._size
        pygame.draw.rect(self._screen, (0, 0, 0), (x, y, width, height))

        # calculate range of graph scare
        x_range = max(data1) - min(data1)
        y_range = self._max_y_value - self._min_y_value

        # set minimum range of graph
        if x_range == 0:
            x_range = 1
        if y_range == 0:
            y_range = 1

        # def nest for calculate graph
        def scale_x(value):
            return x + (value - min(data1)) / x_range * width

        def scale_y(value):
            return y + height - (value - self._min_y_value) / y_range * height
        
        # match length of data array
        data_length = min(len(data1), len(data2))

        # draw widgets for line
        for i in range(data_length - 1):
            start_pos = (scale_x(data1[i]), scale_y(data2[i]))
            end_pos = (scale_x(data1[i + 1]), scale_y(data2[i + 1]))
            pygame.draw.line(self._screen, color, start_pos, end_pos, 2)

        # define font of widget
        font = pygame.font.Font(None, 24)

        # draw widget of scale of x axis
        for i in range(0, data_length, max(1, data_length // 4)):
            label = font.render(str(data1[i]), True, (0, 0, 0))
            label_pos = (scale_x(data1[i]), y + height + 5)
            self._screen.blit(label, label_pos)

        # draw widget of scale of y axis
        num_y_labels = 5
        for i in range(num_y_labels + 1):
            value = self._min_y_value + i * (y_range / num_y_labels)
            #value = i * (y_range / num_y_labels)
            label = font.render(f"{value:.1f}", True, (0, 0, 0))
            label_pos = (x - 40, scale_y(value)  - 10)
            self._screen.blit(label, label_pos)
        
        #draw title widget
        title_font = pygame.font.Font(None, 36)
        title_surface = title_font.render(self._title, True, (0, 0, 0))
        title_rect = title_surface.get_rect(center=(x + width // 2, y + height + 40))
        self._screen.blit(title_surface, title_rect)

#class for showing button on pygame screen
class Button:
    # initialize button
    def __init__(self, screen, text, rect, color, hover_color, font, font_color):
        # read input
        self._screen = screen
        self._text = text
        self._rect = rect
        self._color = color
        self._hover_color = hover_color
        self._font = font
        self._font_color = font_color

    # draw button component
    def draw(self):
        # get mouse position
        mouse_pos = pygame.mouse.get_pos()

        # judge whether cursor is on the button or not
        is_hovered = self._rect.collidepoint(mouse_pos)

        # draw button according to situation
        if is_hovered:
            pygame.draw.rect(self._screen, self._hover_color, self._rect)
        else:
            pygame.draw.rect(self._screen, self._color, self._rect)
        
        # draw text on the button
        text_surf = self._font.render(self._text, True, self._font_color)
        text_rect = text_surf.get_rect(center=self._rect.center)
        self._screen.blit(text_surf, text_rect)
        
        return is_hovered

class UDP:
    # initialize udp module
    def __init__(self):
        # prepare value for clock
        self._prev_time = 0
        self._currennt_time = 0
        self._clocks = []
        self._clock = 0
        # wifi flag
        self._wifi_flag = False
        # wifi lost countor
        self._lost_cnt = 0
        # udp send pack
        self._send_pack = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # udp receive pack
        self._receive_pack = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        # initialize udp communication
        udp_module.py_init_udp("192.168.20.2", "192.168.20.3", 5000)
    
    # function for set send packet
    def update_send_pack(self, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10):
        # store input data to send pack
        self._send_pack[0] = ch1
        self._send_pack[1] = ch2
        self._send_pack[2] = ch3
        self._send_pack[3] = ch4
        self._send_pack[4] = ch5
        self._send_pack[5] = ch6
        self._send_pack[6] = ch7
        self._send_pack[7] = ch8
        self._send_pack[8] = ch9
        self._send_pack[9] = ch10

    # function for send packet
    def send(self):
        # send data
        udp_module.py_send_udp(self._send_pack[0], 
                               self._send_pack[1], 
                               self._send_pack[2], 
                               self._send_pack[3], 
                               self._send_pack[4], 
                               self._send_pack[5], 
                               self._send_pack[6], 
                               self._send_pack[7], 
                               self._send_pack[8], 
                               self._send_pack[9])
    
    # function for receive packet
    def receive(self):
        # receive data
        received_data = udp_module.py_receive_udp()

        # check length of received data
        if len(received_data) == 10:
            # data length is correct
            self._currennt_time = time.time()
            # try calculate clock
            try:
                # calculate frequency. this may occur error of divided by zero
                self._clocks.append(1 / (self._currennt_time - self._prev_time))
                if len(self._clocks) > 10:
                    del self._clocks[0]
                # calculate average of clocks <- This may be nonsense
                self._clock = str(int(sum(self._clocks) / len(self._clocks)))
            except:
                # set _clock as - if _clock can't be calcrated
                self._clock = "-"
            self._prev_time = self._currennt_time
    
            # reset lost countor
            self._lost_cnt = 0

            # store received data to receive pack
            for i in range(0, 10):
                self._receive_pack[i] = received_data[i]
            
            # return True if receive process is done correctly
            return True
        else:
            #count lost countor
            self._lost_cnt += 1
                
            #set wifi flag False. if lost counter is more than 100
            if self._lost_cnt >= 100:
                # return False if connection is lost
                return False
            
            # return True if lost counter is below 100
            return True
    
    # function for getting value of clock
    def get_clock(self):
        return self._clock
    
    # function for getting value of receive pack
    def get_receive_data(self):
        return self._receive_pack

class GUI:
    # initialize GUI
    def __init__(self):
        # prepare resource path
        self._path_conf = resource_path("resources/config.csv") # registory of config which decide channel assignments
        self._path_log = resource_path("resources/log_path.txt") # registory of directory for log data

        # set time epoch
        self.epoch_screen = time.time() # epoch time for graph on the screen
        self.epoch_log = 0 # epoch time for log data
        
        # flags
        self._main_flag = True
        self._wifi_flag = False 
        self._controller_flag = False
        self._logger_flag = False

        # button map for dualsense. first index of each value represent button name. second index represents value of button
        self.button_map = {
            0: ["Cross", 0],
            1: ["Circle", 0],
            2: ["Square", 0],
            3: ["Triangle", 0],
            4: ["create", 0],# \|/button on controller up left
            5: ["PS", 0],
            6: ["options", 0],# =button on controller up right
            7: ["R2 Button", 0],
            8: ["Share", 0],
            9: ["L1", 0],
            10: ["R1", 0],
            11: ["hat up", 0],
            12: ["hat down", 0],
            13: ["hat left", 0],
            14: ["hat right", 0],
            15: ["center pad", 0],
            16: ["l_stick x", 0],
            17: ["l_stick y", 0],
            18: ["r_stick x", 0],
            19: ["r_stick y", 0],
            20: ["l2_stick", 0],
            21: ["r2_stick", 0],
            22: ["nan", 0]
        }

        # prapare arrays for data
        self.time_arr  =[0]
        self.ch1_arr = [0]
        self.ch2_arr = [0]
        self.ch3_arr = [0]
        self.ch4_arr = [0]
        self.ch5_arr = [0]
        self.ch6_arr = [0]
        self.ch7_arr = [0]
        self.ch8_arr = [0]
        self.ch9_arr = [0]
        self.ch10_arr = [0]

        # initialize pygame
        pygame.init()

        # initialize screen comfiguration
        self.screen = pygame.display.set_mode((1200, 820))
        self.button_font = pygame.font.Font(None, 36)
        self.text_font = pygame.font.Font(None, 40)
        pygame.display.set_caption("Ground Station")
        self.screen.fill((255, 255, 255))

        ### initialize widget componets

        ## initialize button componets

        # initialize reset button
        self.reset_rect = pygame.Rect(50, 60, 420, 50)
        self.reset_button = Button(self.screen, "Reset", self.reset_rect, (0, 0, 0), (100, 100, 100), self.button_font, (255, 0, 0))
        # initialize connect wifi button
        self.wifi_rect = pygame.Rect(50, 150, 420, 50)
        self.wifi_button = Button(self.screen, "Connect Wifi", self.wifi_rect, (0, 0, 0), (100, 100, 100), self.button_font, (0, 255, 0))
        # initialize connect controller button
        self.controller_rect = pygame.Rect(50, 220, 420, 50)
        self.controller_button = Button(self.screen, "Connect Controller", self.controller_rect, (0, 0, 0), (100, 100, 100), self.button_font, (20, 255, 0))
        # initialize log start button
        self.logger_start_rect = pygame.Rect(50, 290, 420, 50)
        self.logger_start_button = Button(self.screen, "log start", self.logger_start_rect, (0, 0, 0), (100, 100, 100), self.button_font, (0, 255, 0))
        # initialize log stop button
        self.logger_stop_rect = pygame.Rect(50, 290, 420, 50)
        self.logger_stop_button = Button(self.screen, "log stop", self.logger_stop_rect, (0, 0, 0), (100, 100, 100), self.button_font, (0, 255, 0))

        ## initialize wifi Hz viewer
        self.wifi_rate = self.text_font.render("Wifi rate : ", True, (0, 128, 0))
        self.controller_rate = self.text_font.render("Wifi rate : ", True, (0, 128, 0))

        ## initailize channel assignment

        # load saved configuration
        with open(self._path_conf, "r") as f:
            reader = csv.reader(f)
            data_list = list(reader)
        
        # initialize widget components
        self.ch1_assingment = ChannelAssignment(self.screen, (50, 400), (180, 60), "Ch 1", self.button_map, data_list[0][0])
        self.ch2_assingment = ChannelAssignment(self.screen, (50, 480), (180, 60), "Ch 2", self.button_map, data_list[1][0])
        self.ch3_assingment = ChannelAssignment(self.screen, (50, 560), (180, 60), "Ch 3", self.button_map, data_list[2][0])
        self.ch4_assingment = ChannelAssignment(self.screen, (50, 640), (180, 60), "Ch 4", self.button_map, data_list[3][0])
        self.ch5_assingment = ChannelAssignment(self.screen, (50, 720), (180, 60), "Ch 5", self.button_map, data_list[4][0])
        self.ch6_assingment = ChannelAssignment(self.screen, (320, 400), (180, 60), "Ch 6", self.button_map, data_list[5][0])
        self.ch7_assingment = ChannelAssignment(self.screen, (320,480), (180, 60), "Ch 7", self.button_map, data_list[6][0])
        self.ch8_assingment = ChannelAssignment(self.screen, (320, 560), (180, 60), "Ch 8", self.button_map, data_list[7][0])
        self.ch9_assingment = ChannelAssignment(self.screen, (320, 640), (180, 60), "Ch 9", self.button_map, data_list[8][0])
        self.ch10_assingment = ChannelAssignment(self.screen, (320, 720), (180, 60), "Ch 10", self.button_map, data_list[9][0])

        ## initialize graphs
        self.graph_ch1 = Graph(self.screen, (600, 20), (250, 100), "ch1")
        self.graph_ch2 = Graph(self.screen, (600, 180), (250, 100), "ch2")
        self.graph_ch3 = Graph(self.screen, (600, 340), (250, 100), "ch3")
        self.graph_ch4 = Graph(self.screen, (600, 500), (250, 100), "ch4")
        self.graph_ch5 = Graph(self.screen, (600, 660), (250, 100), "ch5")
        self.graph_ch6 = Graph(self.screen, (900, 20), (250, 100), "ch6")
        self.graph_ch7 = Graph(self.screen, (900, 180), (250, 100), "ch7")
        self.graph_ch8 = Graph(self.screen, (900, 340), (250, 100), "ch8")
        self.graph_ch9 = Graph(self.screen, (900, 500), (250, 100), "ch9")
        self.graph_ch10 = Graph(self.screen, (900, 660), (250, 100), "ch10")
    
    # function for initialize controller
    def init_controller(self):
        try:
            #initialize pygame joycon instance
            pygame.joystick.init()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self._controller_flag = True
            return True
        except:
            return False
    
    # main function of ground station
    def mainloop(self):
        # set counter. this is used to control refresh rate of screen
        cnt = 0
        clock = "-" # initialize clock value. this may be not needed
        # do repeatedly
        while self._main_flag:
            # refresh screen
            self.screen.fill((255, 255, 255))
            # count　up countor
            cnt += 1
            # communicate with remote drone if wifi flag is True
            if self._wifi_flag:
                # load configuration of channel assignment
                with open(self._path_conf, "r") as f:
                    reader = csv.reader(f)
                    data_list = list(reader)
                # set udp packet
                self.udp.update_send_pack(self.button_map[get_index_dictionary(self.button_map, data_list[0][0])][1],
                                          self.button_map[get_index_dictionary(self.button_map, data_list[1][0])][1],
                                          self.button_map[get_index_dictionary(self.button_map, data_list[2][0])][1],
                                          self.button_map[get_index_dictionary(self.button_map, data_list[3][0])][1],
                                          self.button_map[get_index_dictionary(self.button_map, data_list[4][0])][1],
                                          self.button_map[get_index_dictionary(self.button_map, data_list[5][0])][1],
                                          self.button_map[get_index_dictionary(self.button_map, data_list[6][0])][1],
                                          self.button_map[get_index_dictionary(self.button_map, data_list[7][0])][1],
                                          self.button_map[get_index_dictionary(self.button_map, data_list[8][0])][1],
                                          self.button_map[get_index_dictionary(self.button_map, data_list[9][0])][1])
                # send packet to remote drone
                self.udp.send()
                # get packet from remote drone. this function omit boolean value
                isdata = self.udp.receive()
                # judge whether received data have no problem or not
                if isdata:
                    # get receive data and clock if receive data is avalable
                    receive_data = self.udp.get_receive_data()
                    # get clock
                    try:
                        clock = self.udp.get_clock()
                    except:
                        clock = "-"
                else:
                    # set data if receive data is not avalable
                    receive_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                    clock = "-"
                    self.wifi_flag = False
            else:
                # set data if wifi is not avalable
                receive_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                clock = "-"
            
            # get current time
            current_time = round(time.time() - self.epoch_screen, 2)

            # append data to arr
            self.time_arr.append(current_time)
            self.ch1_arr.append(receive_data[0])
            self.ch2_arr.append(receive_data[1])
            self.ch3_arr.append(receive_data[2])
            self.ch4_arr.append(receive_data[3])
            self.ch5_arr.append(receive_data[4])
            self.ch6_arr.append(receive_data[5])
            self.ch7_arr.append(receive_data[6])
            self.ch8_arr.append(receive_data[7])
            self.ch9_arr.append(receive_data[8])
            self.ch10_arr.append(receive_data[9])

            # limit the array length to be less than the maximum value
            if len(self.time_arr) >= 200:
                del self.time_arr[0]
                del self.ch1_arr[0]
                del self.ch2_arr[0]
                del self.ch3_arr[0]
                del self.ch4_arr[0]
                del self.ch5_arr[0]
                del self.ch6_arr[0]
                del self.ch7_arr[0]
                del self.ch8_arr[0]
                del self.ch9_arr[0]
                del self.ch10_arr[0]
            
            # process for writing log
            if self._logger_flag:
                # store current time
                string_data = str(round(time.time() - self.epoch_log, 3))

                #store received data to receive pack
                for i in range(0, 10):
                    string_data += " ," + str(receive_data[i])
                string_data += "\n"

                #save received data into logfile
                with open(self._file_path, 'a', encoding='utf-8') as file:
                    file.write(string_data)
                
            
            # turn controller flag into False if controller is lost
            if self._controller_flag:
                joycount= pygame.joystick.get_count()
                if joycount == 0:
                    self._controller_flag = False
            
            # get event of pygame
            eventlist = pygame.event.get()

            # treat with event 
            for event in eventlist:
                # termination process
                if event.type == QUIT:
                        self.main_flag = False
                        self.wifi_flag = False
                        self.controller_flag = False
                        return
                
                # click process
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # left click
                        # read values of channel assignment
                        ch1_conf = self.ch1_assingment.handle_events(event)
                        ch2_conf = self.ch2_assingment.handle_events(event)
                        ch3_conf = self.ch3_assingment.handle_events(event)
                        ch4_conf = self.ch4_assingment.handle_events(event)
                        ch5_conf = self.ch5_assingment.handle_events(event)
                        ch6_conf = self.ch6_assingment.handle_events(event)
                        ch7_conf = self.ch7_assingment.handle_events(event)
                        ch8_conf = self.ch8_assingment.handle_events(event)
                        ch9_conf = self.ch9_assingment.handle_events(event)
                        ch10_conf = self.ch10_assingment.handle_events(event)

                        # format data of configuration of channel assignment
                        config = ""
                        config += ch1_conf + "\n"
                        config += ch2_conf + "\n"
                        config += ch3_conf + "\n"
                        config += ch4_conf + "\n"
                        config += ch5_conf + "\n"
                        config += ch6_conf + "\n"
                        config += ch7_conf + "\n"
                        config += ch8_conf + "\n"
                        config += ch9_conf + "\n"
                        config += ch10_conf + "\n"

                        # write config data into confing file
                        with open(self._path_conf, "w") as f:
                            f.write(config)

                        # process for resetbutton
                        if self.reset_rect.collidepoint(event.pos):
                            # reset all data 
                            self.epoch_screen = time.time()
                            self.time_arr = []
                            self.ch1_arr = []
                            self.ch2_arr = []
                            self.ch3_arr = []
                            self.ch4_arr = []
                            self.ch5_arr = []
                            self.ch6_arr = []
                            self.ch7_arr = []
                            self.ch8_arr = []
                            self.ch9_arr = []
                            self.ch10_arr = []
                        
                        # process for wifi conntect
                        if self.wifi_rect.collidepoint(event.pos):
                            # initialize udp module
                            self.udp = UDP()
                            self._wifi_flag = True
                        
                        # process for controller connect
                        if self.controller_rect.collidepoint(event.pos):
                            self.controller_flag = self.init_controller()
                        
                        # process for log
                        if self._logger_flag != True:
                            if self.logger_start_rect.collidepoint(event.pos):
                                with open(self._path_log, "r") as f:
                                    file_path = f.read()
                                # try create log file
                                try:
                                    # set current time as log epoch
                                    self.epoch_log = time.time()
                                    # get current date for log file name
                                    current_time = time.localtime()
                                    # format name of log file
                                    formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', current_time)
                                    # define path of log file
                                    self._file_path = os.path.join(file_path, formatted_time + ".csv")
                                    # create log file
                                    with open(self._file_path, 'w', encoding='utf-8') as f:
                                        f.write("time, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,ch9, ch10\n")#initialize log file header
                                    self._logger_flag = True
                                except:
                                    # if process fail to create log file, open path editor. process for multi OS may not be needed
                                    with open(self._path_log, "w") as f:
                                        f.write("replace this message with path to your log directory")
                                    if subprocess.os.name == 'nt':  # Windows
                                        subprocess.run(['start', self._path_log], shell=True)
                                    elif subprocess.sys.platform == 'darwin':  # macOS
                                        subprocess.run(['open', self._path_log])
                                    else:  # Linux and other Unix-like OS
                                        subprocess.run(['xdg-open', self._path_log])
                        else:
                            if self.logger_stop_rect.collidepoint(event.pos):
                                self._logger_flag = False

                # joystick process
                if event.type == pygame.locals.JOYAXISMOTION:
                    #read values of controller
                    self.button_map[16][1], self.button_map[17][1], self.button_map[18][1], self.button_map[19][1], self.button_map[20][1], self.button_map[21][1]  = self.joystick.get_axis(2), -1 * self.joystick.get_axis(3), self.joystick.get_axis(0), -1 * self.joystick.get_axis(1), self.joystick.get_axis(5), self.joystick.get_axis(4)

                # button process  
                if event.type == pygame.locals.JOYBUTTONDOWN:
                    # The value alternates between 0 and 1 each time the button is pressed.
                    if self.button_map[event.button][1] == 1:
                        self.button_map[event.button][1] = 0
                    elif self.button_map[event.button][1] == 0:
                        self.button_map[event.button][1] = 1
            
            if cnt % 5 == 0:
                #show graph
                self.graph_ch1.draw(self.time_arr, self.ch1_arr)
                self.graph_ch2.draw(self.time_arr, self.ch2_arr)
                self.graph_ch3.draw(self.time_arr, self.ch3_arr)
                self.graph_ch4.draw(self.time_arr, self.ch4_arr)
                self.graph_ch5.draw(self.time_arr, self.ch5_arr)
                self.graph_ch6.draw(self.time_arr, self.ch6_arr)
                self.graph_ch7.draw(self.time_arr, self.ch7_arr)
                self.graph_ch8.draw(self.time_arr, self.ch8_arr)
                self.graph_ch9.draw(self.time_arr, self.ch9_arr)
                self.graph_ch10.draw(self.time_arr, self.ch10_arr)

                # show wifi Hz
                text_surface = self.text_font.render("Wifi freqency : " + str(clock) + " Hz", True, (0, 0, 0))
                text_rect = text_surface.get_rect(topleft=(50, 10)) 
                self.screen.blit(text_surface, text_rect)

                # show channel assignment
                self.ch1_assingment.draw()
                self.ch2_assingment.draw()
                self.ch3_assingment.draw()
                self.ch4_assingment.draw()
                self.ch5_assingment.draw()
                self.ch6_assingment.draw()
                self.ch7_assingment.draw()
                self.ch8_assingment.draw()
                self.ch9_assingment.draw()
                self.ch10_assingment.draw()

                # show button
                self.reset_button.draw()

                if self._wifi_flag != True:
                    self.wifi_button.draw()
                
                if self._controller_flag != True:
                    self.controller_button.draw()
                
                if self._logger_flag != True:
                    self.logger_start_button.draw()
                else:
                    self.logger_stop_button.draw()
                

                #update screen
                pygame.display.flip()

# launch main purocess           
main = GUI()
main.mainloop()