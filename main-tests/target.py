import time


class Target:
    """Class that represent current target object that will be our destiny"""
    def __init__(self):
        # Na razie tylko koncepcja klasy Celu z listą priorytetów która wybiera cel do którego mamy jechać
        self.priority_list = [[1, 0], [0, 1]]  # Lista priorytetów
        self.priority_list_pointer = 0  # Która lista priorytetów jest wybrana
        self.last_time = time.time()  # Czas kiedy cel był osatnio widziany
        self.max_time = 1000  # Maksymalny czas bez wykrycia celu
        self.position = []
        self.distance = []

    def update_target_position(self, objects_detected_frame):
        pass

    def update_target(self):
        if self.priority_list_pointer < 1:
            self.priority_list_pointer += 1  # Zwiększenie priorytetu o 1
            return False
        else:
            return True

    def check_obstacle(self):
        pass

    def avoid_obstacle(self):
        pass

    def get_target_position(self):
        return self.target_position

    def get_target_distance(self):
        pass



