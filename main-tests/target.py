import time


class Target:
    """Class that represent current target object that will be our destiny"""
    def __init__(self):
        # Na razie tylko koncepcja klasy Celu z listą priorytetów która wybiera cel do którego mamy jechać
        self.priority_list = [[1, 0], [0, 1]]  # lista priorytetów
        self.priority_list_pointer = 0  # która lista priorytetów jest wybrana
        self.last_time = time.time()  # czas kiedy obiket był osatnio widziany
        self.max_time = 1000  # maksymalny czas bez szukanego obiektu
        self.target_position = []

    def update_target_position(self, objects_detected_frame):
        pass

    def update_target(self):
        if self.priority_list_pointer < 1:
            self.priority_list_pointer += 1  # Zwiększenie priorytetu o 1
            return False
        else:
            return True

    def get_target_position(self):
        return self.target_position



