# Odroid

04.12.2019 - Ernest Szypula
- add plot_data send method in PIDThread

06.11.2019 - Michał Winciorek
- add new class in autonomy.py file -> a prototype of class 
which implements all abstract autonoumy and steering methods using objects from MainTests.py
for now, later we should modify previous concept of main file

04.11.2019 - Michał Winciorek
- add pad steering functionality:
  - connection with PC/Jetson for pad
  - new Thread (PadSteeringThread) for data handling and motors updating in MainTests.py
- add all files used by Odroid