import time


class Target:
    """Class that represent current target object that will be our destiny"""
    def __init__(self):
        # Na razie tylko koncepcja klasy Celu z listą priorytetów która wybiera cel do którego mamy jechać
        self.priority_list = [0, 1]  # Lista priorytetów
        self.priority_list_pointer = 0  # Która lista priorytetów jest wybrana
        self.last_time = time.time()  # Czas kiedy cel był osatnio widziany
        self.max_time = 1000  # Maksymalny czas bez wykrycia celu
        self.obstacle = []  # Lista przeszkód widzianych przez kamere
        self.obstacle_to_avoid = []
        self.position = []
        self.distance = []
        self.flag = False
        self.first_view_time = None
        self.cam = None

    def update_target_position(self, objects_detected_frame):
        target_flag = False
        self.obstacle.clear()
        for obj in objects_detected_frame[0]:
            if obj[0] == self.priority_list[self.priority_list_pointer]:
                self.position = obj[1:]
                target_flag = True
                self.cam = 0
                if self.first_view_time is None:
                    self.first_view_time = time.time()
            else:
                self.obstacle.append(obj)
        for obj in objects_detected_frame[1]:
            if obj[0] == self.priority_list[self.priority_list_pointer]:
                target_flag = True
                self.cam = 1
                if self.first_view_time is None:
                    self.first_view_time = time.time()
            else:
                self.obstacle.append(obj)
        if target_flag:
            self.last_time = time.time()
            self.flag = True
        else:
            if (time.time() - self.last_time) > self.max_time:
                self.change_target()
        self.check_obstacles()

    """Return
        True - koniec celi
        False - mamy kolejny cel
        """
    def change_target(self):
        if self.priority_list_pointer < 1:
            self.priority_list_pointer += 1  # Zwiększenie priorytetu o 1
            self.first_view_time = None
            self.position.clear()
            self.distance.clear()
            self.cam = None
            self.flag = False
            return False
        else:
            return True

    """Sprawdzanie czy obiekt jest na drodze"""
    def check_obstacles(self):
        pass # tutaj warunek, że obiekt jest na naszej drodze i trzeba go ominąć, dodanie so self.obstacles_to_avoid

    """Return Obstacles to avoid data"""
    def get_obstacles_to_avoid(self):
        return self.obstacle_to_avoid

    """Return target position and camera"""
    def get_target_position(self):
        return self.position, self.cam

    """Retrun target distance and camera"""
    def get_target_distance(self):
        return self.distance, self.cam

    """Return time of view target"""
    def get_time_of_view(self):
        if self.first_view_time is None:
            return 0
        else:
            return time.time() - self.first_view_time

    """Return find flag"""
    def get_flag(self):
        return self.flag


